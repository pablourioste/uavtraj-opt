#include "ofc/ocp_problem.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "ofc/types.hpp"

namespace ofc::ocp {

void CollocationProblem::setup(const Layout& layout, ModelInfo info, std::vector<BoundaryRow> boundary, Vec z0,
                               std::vector<WaypointNode> waypoints) {
    layout_ = layout;
    info_ = std::move(info);
    boundary_ = std::move(boundary);
    waypoints_ = std::move(waypoints);
    z0_ = std::move(z0);
    nc_ = static_cast<int>(info_.pathNames.size());
    offBoundary_ = layout_.numDefects();
    offPath_ = offBoundary_ + static_cast<int>(boundary_.size());
    offWaypoints_ = offPath_ + nc_ * layout_.N;
    if (static_cast<int>(z0_.size()) != layout_.numVariables())
        throw std::invalid_argument("CollocationProblem: initial point has the wrong size");
    for (const WaypointNode& w : waypoints_)
        if (w.node < 0 || w.node >= layout_.N) throw std::invalid_argument("waypoint " + w.wp.id + ": bad node");
}

void CollocationProblem::variableBounds(Vec& zl, Vec& zu) const {
    const Layout& L = layout_;
    zl.assign(L.numVariables(), -nlp::kInf);
    zu.assign(L.numVariables(), nlp::kInf);
    zl[L.idxTf()] = info_.tfBox.lo, zu[L.idxTf()] = info_.tfBox.hi;
    for (int k = 0; k < L.N; ++k) {
        for (int i = 0; i < L.nx; ++i)
            zl[L.idxState(k, i)] = info_.stateBox[i].lo, zu[L.idxState(k, i)] = info_.stateBox[i].hi;
        for (int j = 0; j < L.nu; ++j)
            zl[L.idxControl(k, j)] = info_.controlBox[j].lo, zu[L.idxControl(k, j)] = info_.controlBox[j].hi;
    }
}

void CollocationProblem::constraintBounds(Vec& gl, Vec& gu) const {
    const int m = numConstraints();
    gl.assign(m, 0.0);
    gu.assign(m, 0.0);
    for (int i = offPath_; i < m; ++i) gl[i] = -nlp::kInf; // path and waypoint rows: <= 0
}

Vec CollocationProblem::variableScale() const {
    const Layout& L = layout_;
    Vec s(L.numVariables());
    s[L.idxTf()] = info_.tfScale;
    for (int k = 0; k < L.N; ++k) {
        for (int i = 0; i < L.nx; ++i) s[L.idxState(k, i)] = info_.stateScale[i];
        for (int j = 0; j < L.nu; ++j) s[L.idxControl(k, j)] = info_.controlScale[j];
    }
    return s;
}

Vec CollocationProblem::constraints(const Vec& z) const {
    const Layout& L = layout_;
    Vec g = defects(z);
    g.reserve(numConstraints());
    for (const BoundaryRow& b : boundary_) g.push_back((z[b.col] - b.target) / b.scale);
    for (int k = 0; k < L.N; ++k) {
        Vec x(L.nx), u(L.nu);
        for (int i = 0; i < L.nx; ++i) x[i] = z[L.idxState(k, i)];
        for (int j = 0; j < L.nu; ++j) u[j] = z[L.idxControl(k, j)];
        const Vec h = pathAt(x, u);
        g.insert(g.end(), h.begin(), h.end());
    }
    for (const WaypointNode& w : waypoints_) {
        const double dx = z[L.idxState(w.node, info_.idxX)] - w.wp.position.x;
        const double dy = z[L.idxState(w.node, info_.idxY)] - w.wp.position.y;
        g.push_back((dx * dx + dy * dy) / (w.wp.tolerance * w.wp.tolerance) - 1.0);
    }
    return g;
}

std::vector<Triplet> CollocationProblem::constraintJacobian(const Vec& z) const {
    const Layout& L = layout_;
    std::vector<Triplet> J = defectJacobian(z);
    for (std::size_t r = 0; r < boundary_.size(); ++r)
        J.push_back({offBoundary_ + static_cast<int>(r), boundary_[r].col, 1.0 / boundary_[r].scale});

    // Path rows: one dense block per node, central differences on (x_k, u_k).
    const int nv = L.nx + L.nu;
    for (int k = 0; k < L.N; ++k) {
        std::vector<int> cols(nv);
        Vec v(nv), s(nv);
        for (int i = 0; i < L.nx; ++i) cols[i] = L.idxState(k, i), s[i] = info_.stateScale[i];
        for (int j = 0; j < L.nu; ++j) cols[L.nx + j] = L.idxControl(k, j), s[L.nx + j] = info_.controlScale[j];
        for (int c = 0; c < nv; ++c) v[c] = z[cols[c]];
        auto eval = [&](const Vec& w) {
            return pathAt(Vec(w.begin(), w.begin() + L.nx), Vec(w.begin() + L.nx, w.end()));
        };
        std::vector<Vec> block(nv);
        Vec w = v;
        for (int c = 0; c < nv; ++c) {
            const double h = 1e-6 * std::max(std::fabs(v[c]), s[c]);
            w[c] = v[c] + h;
            const Vec hp = eval(w);
            w[c] = v[c] - h;
            const Vec hm = eval(w);
            w[c] = v[c];
            block[c].resize(nc_);
            for (int r = 0; r < nc_; ++r) block[c][r] = (hp[r] - hm[r]) / (2.0 * h);
        }
        for (int r = 0; r < nc_; ++r)
            for (int c = 0; c < nv; ++c) J.push_back({offPath_ + k * nc_ + r, cols[c], block[c][r]});
    }

    for (std::size_t r = 0; r < waypoints_.size(); ++r) {
        const WaypointNode& w = waypoints_[r];
        const int cx = L.idxState(w.node, info_.idxX), cy = L.idxState(w.node, info_.idxY);
        const double t2 = w.wp.tolerance * w.wp.tolerance;
        const int row = offWaypoints_ + static_cast<int>(r);
        J.push_back({row, cx, 2.0 * (z[cx] - w.wp.position.x) / t2});
        J.push_back({row, cy, 2.0 * (z[cy] - w.wp.position.y) / t2});
    }
    return J;
}

std::string CollocationProblem::constraintLabel(int i) const {
    const Layout& L = layout_;
    if (i < offBoundary_) return "defect " + info_.stateNames[i % L.nx] + " @" + std::to_string(i / L.nx);
    if (i < offPath_) return boundary_[i - offBoundary_].label;
    if (i < offWaypoints_) {
        const int r = i - offPath_;
        return info_.pathNames[r % nc_] + " @" + std::to_string(r / nc_);
    }
    const WaypointNode& w = waypoints_[i - offWaypoints_];
    return "wp:" + w.wp.id + " @" + std::to_string(w.node);
}

nlp::Kind CollocationProblem::constraintKind(int i) const {
    if (i < offBoundary_) return nlp::Kind::Dynamics;
    if (i < offPath_) return nlp::Kind::Boundary;
    if (i < offWaypoints_)
        return info_.pathNames[(i - offPath_) % nc_].rfind("nfz:", 0) == 0 ? nlp::Kind::Environment
                                                                           : nlp::Kind::Envelope;
    return nlp::Kind::Environment;
}

std::string CollocationProblem::variableLabel(int j) const {
    const Layout& L = layout_;
    if (j == L.idxTf()) return "tf";
    if (j < L.idxControl(0, 0)) {
        const int r = j - 1;
        return info_.stateNames[r % L.nx] + " @" + std::to_string(r / L.nx);
    }
    const int r = j - L.idxControl(0, 0);
    return info_.controlNames[r % L.nu] + " @" + std::to_string(r / L.nu);
}

std::vector<double> CollocationProblem::worstPath(const Vec& z) const {
    std::vector<double> worst(nc_, -std::numeric_limits<double>::infinity());
    const Vec g = constraints(z);
    for (int k = 0; k < layout_.N; ++k)
        for (int r = 0; r < nc_; ++r) worst[r] = std::max(worst[r], g[offPath_ + k * nc_ + r]);
    return worst;
}

std::vector<WaypointNode> assignWaypoints(const Trajectory& guess, const std::vector<env::Waypoint>& wps, int idxX,
                                          int idxY) {
    std::vector<WaypointNode> out;
    int from = 1; // never the initial node, which is fixed
    const int N = guess.nodes();
    for (std::size_t i = 0; i < wps.size(); ++i) {
        // Leave room for the remaining waypoints and the final node.
        const int last = N - 2 - static_cast<int>(wps.size() - 1 - i);
        int best = std::min(from, last);
        double dBest = std::numeric_limits<double>::infinity();
        for (int k = from; k <= last; ++k) {
            const double d = std::hypot(guess.X[k][idxX] - wps[i].position.x, guess.X[k][idxY] - wps[i].position.y);
            if (d < dBest) dBest = d, best = k;
        }
        out.push_back({wps[i], best});
        from = best + 1;
    }
    return out;
}

double unwrapNear(double angle, double reference) {
    return angle + 2.0 * kPi * std::round((reference - angle) / (2.0 * kPi));
}

} // namespace ofc::ocp
