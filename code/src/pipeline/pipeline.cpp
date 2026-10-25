#include "ofc/pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "ofc/numerics.hpp"
#include "ofc/ocp_fc.hpp"
#include "ofc/ocp_uf.hpp"
#include "ofc/point_mass.hpp"
#include "ofc/rigid_body.hpp"

namespace ofc::pipeline {

namespace {

// The no-fly-zone rows hold at the nodes. A chord of length l whose two ends
// keep a distance r from a zone corner stays at least sqrt(r^2 - l^2/4) away
// from it, so enforcing r = sqrt(m^2 + (l/2)^2) at the nodes keeps the whole
// segment outside the real margin m. l is the node spacing of a path 50 %
// longer than the mission's reference length (detours around the zones).
std::vector<env::NoFlyZone> segmentSafeZones(std::vector<env::NoFlyZone> zones, const ocp::Reference& ref, int N) {
    const double chord = 1.5 * ref.length / (N - 1);
    for (env::NoFlyZone& z : zones) z.margin = std::sqrt(z.margin * z.margin + 0.25 * chord * chord);
    return zones;
}

// Solves one stage and fills its report; returns the final point.
Vec solveStage(const ocp::CollocationProblem& p, const Config& cfg, Stage& s) {
    s.ran = true;
    const Vec z0 = p.initialPoint();
    const Vec g0 = p.constraints(z0);
    s.guessDefect = maxAbs(Vec(g0.begin(), g0.begin() + p.layout().numDefects()));

    const nlp::Result r = nlp::solve(p, cfg.solver);
    s.status = nlp::toString(r.status);
    s.message = r.message;
    s.millis = r.millis;
    s.iterations = r.iterations;
    s.objective = p.cost(r.z);
    s.check = nlp::verify(p, r.z, cfg.verifyTol);
    s.ok = r.ok() && s.check.ok();
    return r.z;
}

} // namespace

std::string Config::label() const {
    std::string s = seed == Seed::WarmStart ? "WS" : "COLD";
    if (uf) s += ">UF";
    if (fc) s += ">FC";
    return s;
}

Config Config::withLabel(const std::string& label) const {
    Config c = *this;
    std::vector<std::string> parts;
    std::size_t from = 0;
    for (std::size_t i = 0; i <= label.size(); ++i)
        if (i == label.size() || label[i] == '>') parts.push_back(label.substr(from, i - from)), from = i + 1;
    if (parts.empty() || (parts[0] != "WS" && parts[0] != "COLD"))
        throw std::invalid_argument("pipeline label '" + label + "': must start with WS or COLD");
    c.seed = parts[0] == "WS" ? Seed::WarmStart : Seed::Cold;
    c.uf = c.fc = false;
    for (std::size_t i = 1; i < parts.size(); ++i) {
        if (parts[i] == "UF")
            c.uf = true;
        else if (parts[i] == "FC")
            c.fc = true;
        else
            throw std::invalid_argument("pipeline label '" + label + "': unknown stage '" + parts[i] + "'");
    }
    c.fcFromUf = c.uf;
    return c;
}

Config Config::fromJson(const nlohmann::json& j) {
    Config c;
    c.nodesUf = j.value("nodes_uf", c.nodesUf);
    c.nodesFc = j.value("nodes_fc", c.nodesFc);
    c.alpha = j.value("alpha", c.alpha);
    c.betaControl = j.value("beta_control", c.betaControl);
    c.betaState = j.value("beta_state", c.betaState);
    c.sideslipMax = j.value("sideslip_max_deg", c.sideslipMax);
    c.verifyTol = j.value("verify_tol", c.verifyTol);
    const std::string seed = j.value("seed", std::string("warm"));
    if (seed != "warm" && seed != "cold") throw std::invalid_argument("pipeline.seed must be 'warm' or 'cold'");
    const std::string stages = j.value("stages", std::string("UF>FC"));
    c = c.withLabel((seed == "warm" ? "WS>" : "COLD>") + stages);
    c.fcFromUf = j.value("fc_from_uf", c.fcFromUf);
    c.warmStart = ws::Config::fromJson(j.value("planner", nlohmann::json::object()));
    c.solver = nlp::Options::fromJson(j.value("solver", nlohmann::json::object()));
    return c;
}

bool Result::ok() const {
    for (const Stage* s : {&warmStage, &uf, &fc})
        if (s->ran && !s->ok) return false;
    return true;
}

Result run(const Aircraft& ac, const MissionSpec& m, const Config& cfg) {
    const auto t0 = std::chrono::steady_clock::now();
    Result res;
    const bool warm = cfg.seed == Seed::WarmStart;

    // 1. Warm start.
    if (warm) {
        res.warm = ws::build(ac, m, cfg.warmStart);
        Stage& s = res.warmStage;
        s.ran = true;
        s.ok = res.warm.ok;
        s.status = s.ok ? "ok" : "failed";
        s.message = res.warm.message;
        s.millis = res.warm.plannerMillis + res.warm.guidanceMillis;
    }
    const bool haveWarm = warm && res.warm.ok;
    // Scales, cost references and t_f bounds come from the mission, never from
    // the seed, so that every pipeline solves the same problems. (Only the
    // node each waypoint is pinned to follows the initial point.)
    const ocp::Reference ref{m.referenceLength(), m.referenceLength() / m.flight.V};

    // 2. Point-mass stage.
    if (cfg.uf) {
        res.ufGuess = haveWarm ? ws::fromRun(res.warm.uf, cfg.nodesUf) : ws::coldUf(ac, m, cfg.nodesUf);
        res.uf.seed = haveWarm ? "warm start (UF guidance run)" : "straight line";
        Mission ms = m.ufMission(cfg.alpha, cfg.betaControl, cfg.betaState);
        ms.nfz = segmentSafeZones(m.zones, ref, cfg.nodesUf);
        const auto wps = ocp::assignWaypoints(res.ufGuess, m.waypoints, uf::X, uf::Y);
        const ocp::UfProblem p(ac, ms, res.ufGuess, wps, ref);
        const Vec z = solveStage(p, cfg, res.uf);
        res.ufOpt = p.trajectory(z);
        ms.betaControl = ms.betaState = 0.0;
        res.uf.penaltyShare = 1.0 - ocp::UfProblem(ac, ms, res.ufGuess, wps, ref).cost(z) / res.uf.objective;
    }

    // 3. Rigid-body stage.
    if (cfg.fc) {
        const int N = cfg.nodesFc;
        if (cfg.fcFromUf && res.uf.ok) {
            res.fcGuess = ws::ufToFc(ac, ocp::resample(res.ufOpt, N));
            res.fc.seed = "UF optimum (projected)";
        } else if (haveWarm && res.warm.fc.completed) {
            res.fcGuess = ws::fromRun(res.warm.fc, N);
            res.fc.seed = "warm start (FC autopilot run)";
        } else if (haveWarm) {
            res.fcGuess = ws::ufToFc(ac, ws::fromRun(res.warm.uf, N));
            res.fc.seed = "warm start (UF guidance run, projected)";
        } else {
            res.fcGuess = ws::coldFc(ac, m, N);
            res.fc.seed = "straight line (trimmed)";
        }
        ocp::FcMission fm;
        fm.x0 = ws::fcStartState(ac, m);
        fm.end = {m.goal.x, m.goal.y, -m.hEnd()};
        fm.headingEnd = m.goal.psi;
        fm.alpha = cfg.alpha;
        fm.betaControl = cfg.betaControl;
        fm.betaState = cfg.betaState;
        fm.sideslipMax = deg2rad(cfg.sideslipMax);
        fm.nfz = segmentSafeZones(m.zones, ref, N);
        fm.hLow = m.hLow(), fm.hHigh = m.hHigh();
        const auto wps = ocp::assignWaypoints(res.fcGuess, m.waypoints, fc::XN, fc::YE);
        const ocp::FcProblem p(ac, fm, res.fcGuess, wps, ref);
        const Vec z = solveStage(p, cfg, res.fc);
        res.fcOpt = p.trajectory(z);
        fm.betaControl = fm.betaState = 0.0;
        res.fc.penaltyShare = 1.0 - ocp::FcProblem(ac, fm, res.fcGuess, wps, ref).cost(z) / res.fc.objective;
    }

    res.totalMillis = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return res;
}

SimulationError ufSimulationError(const Aircraft& ac, const ocp::Trajectory& t) {
    const int N = t.nodes();
    const double dtNode = t.tf / (N - 1);
    auto control = [&](double time) {
        const double s = std::clamp(time / dtNode, 0.0, N - 1.0);
        const int k = std::min(static_cast<int>(s), N - 2);
        const double w = s - k;
        Vec u(uf::NU);
        for (int j = 0; j < uf::NU; ++j) u[j] = (1.0 - w) * t.U[k][j] + w * t.U[k + 1][j];
        return u;
    };
    const num::OdeFn f = [&](double tt, const Vec& xx) { return uf::rhs(ac, xx, control(tt)); };
    const int sub = std::max(1, static_cast<int>(std::ceil(dtNode / 0.01)));
    const double h = dtNode / sub;
    auto distance = [&](const Vec& x, int k) {
        const double dx = x[uf::X] - t.X[k][uf::X], dy = x[uf::Y] - t.X[k][uf::Y], dz = x[uf::Z] - t.X[k][uf::Z];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };
    SimulationError e;
    Vec drift = t.X.front();
    for (int k = 1; k < N; ++k) {
        Vec local = t.X[k - 1];
        const double t0 = (k - 1) * dtNode;
        for (int i = 0; i < sub; ++i) {
            local = num::rk4Step(f, t0 + i * h, local, h);
            drift = num::rk4Step(f, t0 + i * h, drift, h);
        }
        e.local = std::max(e.local, distance(local, k));
        e.drift = std::max(e.drift, distance(drift, k));
    }
    return e;
}

double clearanceAlong(const ocp::Trajectory& t, int idxX, int idxY, const std::vector<env::NoFlyZone>& zones,
                      double hLow, double hHigh) {
    std::vector<env::NoFlyZone> active;
    for (const env::NoFlyZone& z : zones)
        if (z.activeBetween(hLow, hHigh)) active.push_back(z);
    double c = std::numeric_limits<double>::infinity();
    for (int k = 0; k + 1 < t.nodes(); ++k)
        for (int i = 0; i < 10; ++i) {
            const double w = i / 10.0;
            const double x = (1.0 - w) * t.X[k][idxX] + w * t.X[k + 1][idxX];
            const double y = (1.0 - w) * t.X[k][idxY] + w * t.X[k + 1][idxY];
            for (const env::NoFlyZone& z : active) c = std::min(c, z.clearance(x, y));
        }
    return c;
}

double waypointMissAlong(const ocp::Trajectory& t, int idxX, int idxY, const std::vector<env::Waypoint>& wps) {
    double worst = 0.0;
    for (const env::Waypoint& w : wps) {
        double best = std::numeric_limits<double>::infinity();
        for (int k = 0; k + 1 < t.nodes(); ++k)
            for (int i = 0; i <= 10; ++i) {
                const double s = i / 10.0;
                const double x = (1.0 - s) * t.X[k][idxX] + s * t.X[k + 1][idxX];
                const double y = (1.0 - s) * t.X[k][idxY] + s * t.X[k + 1][idxY];
                best = std::min(best, std::hypot(x - w.position.x, y - w.position.y));
            }
        worst = std::max(worst, best);
    }
    return worst;
}

} // namespace ofc::pipeline
