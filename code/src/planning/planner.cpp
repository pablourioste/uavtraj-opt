#include "ofc/planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>

#include "ofc/types.hpp"

namespace ofc::plan {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// Normalised lengths (t, p, q) of one Dubins word, in units of the radius.
// alpha, beta: start/end headings relative to the start->goal line; d: distance / radius.
bool word(int w, double alpha, double beta, double d, double& t, double& p, double& q) {
    const double sa = std::sin(alpha), sb = std::sin(beta), ca = std::cos(alpha), cb = std::cos(beta);
    const double cab = std::cos(alpha - beta);
    switch (w) {
    case 0: { // LSL
        const double p2 = 2.0 + d * d - 2.0 * cab + 2.0 * d * (sa - sb);
        if (p2 < 0.0) return false;
        const double th = std::atan2(cb - ca, d + sa - sb);
        t = mod2pi(th - alpha), p = std::sqrt(p2), q = mod2pi(beta - th);
        return true;
    }
    case 1: { // LSR
        const double p2 = -2.0 + d * d + 2.0 * cab + 2.0 * d * (sa + sb);
        if (p2 < 0.0) return false;
        p = std::sqrt(p2);
        const double th = std::atan2(-ca - cb, d + sa + sb) - std::atan2(-2.0, p);
        t = mod2pi(th - alpha), q = mod2pi(th - mod2pi(beta));
        return true;
    }
    case 2: { // RSL
        const double p2 = -2.0 + d * d + 2.0 * cab - 2.0 * d * (sa + sb);
        if (p2 < 0.0) return false;
        p = std::sqrt(p2);
        const double th = std::atan2(ca + cb, d - sa - sb) - std::atan2(2.0, p);
        t = mod2pi(alpha - th), q = mod2pi(beta - th);
        return true;
    }
    case 3: { // RSR
        const double p2 = 2.0 + d * d - 2.0 * cab + 2.0 * d * (sb - sa);
        if (p2 < 0.0) return false;
        const double th = std::atan2(ca - cb, d - sa + sb);
        t = mod2pi(alpha - th), p = std::sqrt(p2), q = mod2pi(th - beta);
        return true;
    }
    case 4: { // RLR
        const double c = (6.0 - d * d + 2.0 * cab + 2.0 * d * (sa - sb)) / 8.0;
        if (std::fabs(c) > 1.0) return false;
        const double phi = std::atan2(ca - cb, d - sa + sb);
        p = mod2pi(2.0 * kPi - std::acos(c));
        t = mod2pi(alpha - phi + mod2pi(p / 2.0));
        q = mod2pi(alpha - beta - t + mod2pi(p));
        return true;
    }
    case 5: { // LRL
        const double c = (6.0 - d * d + 2.0 * cab + 2.0 * d * (sb - sa)) / 8.0;
        if (std::fabs(c) > 1.0) return false;
        const double phi = std::atan2(ca - cb, d + sa - sb);
        p = mod2pi(2.0 * kPi - std::acos(c));
        t = mod2pi(-alpha - phi + p / 2.0);
        q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p));
        return true;
    }
    }
    return false;
}

constexpr int kDirs[6][3] = {{1, 0, 1}, {1, 0, -1}, {-1, 0, 1}, {-1, 0, -1}, {-1, 1, -1}, {1, -1, 1}};

bool inside(const Bounds& b, const Pose2& p) { return p.x >= b.xMin && p.x < b.xMax && p.y >= b.yMin && p.y < b.yMax; }

double pathLength(const Path& path) {
    double L = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) L += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    return L;
}

} // namespace

double mod2pi(double a) {
    const double r = std::fmod(a, 2.0 * kPi);
    return r < 0.0 ? r + 2.0 * kPi : r;
}

Pose2 advance(const Pose2& p, double curvature, double len) {
    if (std::fabs(curvature) < 1e-12) return {p.x + len * std::cos(p.psi), p.y + len * std::sin(p.psi), p.psi};
    const double R = 1.0 / curvature, psi1 = p.psi + curvature * len;
    return {p.x + R * (std::sin(psi1) - std::sin(p.psi)), p.y - R * (std::cos(psi1) - std::cos(p.psi)), psi1};
}

std::vector<DubinsPath> dubinsAll(const Pose2& a, const Pose2& b, double radius) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double d = std::hypot(dx, dy) / radius;
    const double th = mod2pi(std::atan2(dy, dx));
    const double alpha = mod2pi(a.psi - th), beta = mod2pi(b.psi - th);
    std::vector<DubinsPath> out;
    for (int w = 0; w < 6; ++w) {
        double t, p, q;
        if (!word(w, alpha, beta, d, t, p, q)) continue;
        DubinsPath dp;
        dp.valid = true;
        dp.seg[0] = t * radius, dp.seg[1] = p * radius, dp.seg[2] = q * radius;
        for (int i = 0; i < 3; ++i) dp.dir[i] = kDirs[w][i];
        dp.length = dp.seg[0] + dp.seg[1] + dp.seg[2];
        dp.start = a;
        dp.radius = radius;
        out.push_back(dp);
    }
    return out;
}

DubinsPath dubins(const Pose2& a, const Pose2& b, double radius) {
    DubinsPath best;
    best.length = kInf;
    for (const DubinsPath& d : dubinsAll(a, b, radius))
        if (d.length < best.length) best = d;
    return best;
}

Path sampleDubins(const DubinsPath& d, double step) {
    Path out{d.start};
    Pose2 p = d.start;
    for (int i = 0; i < 3; ++i) {
        const int n = std::max(1, static_cast<int>(std::ceil(d.seg[i] / step)));
        const double ds = d.seg[i] / n, k = d.dir[i] / d.radius;
        if (d.seg[i] <= 0.0) continue;
        for (int j = 0; j < n; ++j) {
            p = advance(p, k, ds);
            out.push_back(p);
        }
    }
    return out;
}

PlanResult hybridAStar(const Pose2& start, const Pose2& goal, double radius, const Bounds& bounds,
                       const FreeFn& isFree, const PlannerConfig& cfg) {
    const auto t0 = std::chrono::steady_clock::now();
    PlanResult res;
    auto finish = [&]() {
        res.length = pathLength(res.path);
        res.millis = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        return res;
    };

    const int nx = static_cast<int>(std::ceil((bounds.xMax - bounds.xMin) / cfg.cell));
    const int ny = static_cast<int>(std::ceil((bounds.yMax - bounds.yMin) / cfg.cell));
    const int nb = cfg.headingBins;
    auto key = [&](const Pose2& p) -> long long {
        if (!inside(bounds, p)) return -1;
        const int ix = static_cast<int>((p.x - bounds.xMin) / cfg.cell);
        const int iy = static_cast<int>((p.y - bounds.yMin) / cfg.cell);
        const int ib = static_cast<int>(mod2pi(p.psi) / (2.0 * kPi) * nb) % nb;
        return (static_cast<long long>(ix) * ny + iy) * nb + ib;
    };
    auto heuristic = [&](const Pose2& p) {
        return std::max(std::hypot(goal.x - p.x, goal.y - p.y), dubins(p, goal, radius).length);
    };
    auto pathFree = [&](const Path& path) {
        return std::all_of(path.begin(), path.end(),
                           [&](const Pose2& q) { return inside(bounds, q) && isFree(q.x, q.y); });
    };

    if (!inside(bounds, start) || !isFree(start.x, start.y) || !isFree(goal.x, goal.y)) return finish();

    struct Node {
        Pose2 pose;
        double g;
        int parent;
        double steer;
    };
    std::vector<Node> nodes{{start, 0.0, -1, 0.0}};
    std::vector<float> bestG(static_cast<std::size_t>(nx) * ny * nb, std::numeric_limits<float>::infinity());
    std::vector<char> closed(bestG.size(), 0);
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    open.push({heuristic(start), 0});
    const double steers[] = {-1.0, -0.5, 0.0, 0.5, 1.0};
    const int nSub = std::max(1, static_cast<int>(std::ceil(cfg.step / cfg.sampleStep)));

    auto reconstruct = [&](int i) {
        std::vector<int> chain;
        for (; i >= 0; i = nodes[i].parent) chain.push_back(i);
        std::reverse(chain.begin(), chain.end());
        Path path{nodes[chain.front()].pose};
        for (std::size_t c = 1; c < chain.size(); ++c) {
            Pose2 p = nodes[chain[c - 1]].pose;
            for (int j = 0; j < nSub; ++j) {
                p = advance(p, nodes[chain[c]].steer / radius, cfg.step / nSub);
                path.push_back(p);
            }
        }
        return path;
    };

    while (!open.empty()) {
        const int i = open.top().second;
        open.pop();
        const long long k = key(nodes[i].pose);
        if (closed[k]) continue;
        closed[k] = 1;
        if (++res.expansions > cfg.maxExpansions) break;

        // Analytic expansion: a collision-free Dubins shot closes the search.
        const Node node = nodes[i];
        const double dGoal = std::hypot(goal.x - node.pose.x, goal.y - node.pose.y);
        if (res.expansions % cfg.analyticEvery == 0 || dGoal < 4.0 * radius) {
            const DubinsPath shot = dubins(node.pose, goal, radius);
            if (shot.valid) {
                const Path tail = sampleDubins(shot, cfg.sampleStep);
                if (pathFree(tail)) {
                    res.path = reconstruct(i);
                    res.path.insert(res.path.end(), tail.begin() + 1, tail.end());
                    res.found = true;
                    return finish();
                }
            }
        }

        for (double steer : steers) {
            Pose2 c = node.pose;
            bool ok = true;
            for (int j = 0; j < nSub && ok; ++j) {
                c = advance(c, steer / radius, cfg.step / nSub);
                ok = inside(bounds, c) && isFree(c.x, c.y);
            }
            if (!ok) continue;
            const long long kc = key(c);
            if (kc < 0 || closed[kc]) continue;
            const double g = node.g + cfg.step * (1.0 + cfg.turnPenalty * std::fabs(steer)) +
                             cfg.switchPenalty * std::fabs(steer - node.steer);
            if (g < bestG[kc]) {
                bestG[kc] = static_cast<float>(g);
                nodes.push_back({c, g, i, steer});
                open.push({g + heuristic(c), static_cast<int>(nodes.size()) - 1});
            }
        }
    }
    return finish();
}

PlanResult planMission(const Pose2& start, const std::vector<env::Waypoint>& waypoints, const Pose2& goal,
                       double radius, const std::vector<env::NoFlyZone>& zones, double hLow, double hHigh,
                       double safety, const PlannerConfig& cfg) {
    std::vector<const env::NoFlyZone*> active;
    for (const env::NoFlyZone& z : zones)
        if (z.activeBetween(hLow, hHigh)) active.push_back(&z);
    const FreeFn isFree = [&](double x, double y) {
        return std::all_of(active.begin(), active.end(),
                           [&](const env::NoFlyZone* z) { return z->signedDistance(x, y) >= z->margin + safety; });
    };

    // Bounds: every point of interest, padded so the search can go around obstacles.
    Bounds b{start.x, start.x, start.y, start.y};
    auto grow = [&b](double x, double y) {
        b.xMin = std::min(b.xMin, x), b.xMax = std::max(b.xMax, x);
        b.yMin = std::min(b.yMin, y), b.yMax = std::max(b.yMax, y);
    };
    grow(goal.x, goal.y);
    for (const env::Waypoint& w : waypoints) grow(w.position.x, w.position.y);
    for (const env::NoFlyZone* z : active)
        for (const env::Vec2& v : z->polygon) grow(v.x, v.y);
    const double pad = 3.0 * radius + 300.0;
    b = {b.xMin - pad, b.xMax + pad, b.yMin - pad, b.yMax + pad};

    // Leg targets: each waypoint, headed towards the next target, then the goal.
    std::vector<Pose2> targets;
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        const env::Vec2& p = waypoints[i].position;
        const env::Vec2 next = i + 1 < waypoints.size() ? waypoints[i + 1].position : env::Vec2{goal.x, goal.y};
        targets.push_back({p.x, p.y, std::atan2(next.y - p.y, next.x - p.x)});
    }
    targets.push_back(goal);

    PlanResult total;
    total.found = true;
    Pose2 from = start;
    for (const Pose2& to : targets) {
        const PlanResult leg = hybridAStar(from, to, radius, b, isFree, cfg);
        total.expansions += leg.expansions;
        total.millis += leg.millis;
        if (!leg.found) {
            total.found = false;
            break;
        }
        total.path.insert(total.path.end(), total.path.empty() ? leg.path.begin() : leg.path.begin() + 1,
                          leg.path.end());
        from = to;
    }
    total.length = pathLength(total.path);
    return total;
}

} // namespace ofc::plan
