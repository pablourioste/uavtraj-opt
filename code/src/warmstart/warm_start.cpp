#include "ofc/warm_start.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include "ofc/atmosphere.hpp"
#include "ofc/ocp_problem.hpp"
#include "ofc/point_mass.hpp"
#include "ofc/rigid_body.hpp"
#include "ofc/transcription.hpp"
#include "ofc/trim.hpp"

namespace ofc::ws {

namespace {

double millisSince(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

// Body -> wind rotation for incidence alpha and zero sideslip: maps the
// air-relative body velocity V (cos a, 0, sin a) to (V, 0, 0).
Mat3 bodyToWind(double alpha) {
    const double ca = std::cos(alpha), sa = std::sin(alpha);
    return {{{ca, 0.0, sa}, {0.0, 1.0, 0.0}, {-sa, 0.0, ca}}};
}

Mat3 mul(const Mat3& A, const Mat3& B) {
    Mat3 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k) C[i][j] += A[i][k] * B[k][j];
    return C;
}

// Central differences in time (one-sided at both ends).
Vec timeDerivative(const Vec& v, double dt) {
    const std::size_t n = v.size();
    Vec d(n, 0.0);
    if (n < 2) return d;
    for (std::size_t k = 0; k < n; ++k) {
        const std::size_t a = k == 0 ? 0 : k - 1, b = k + 1 == n ? n - 1 : k + 1;
        d[k] = (v[b] - v[a]) / ((b - a) * dt);
    }
    return d;
}

} // namespace

Config Config::fromJson(const nlohmann::json& j) {
    Config c;
    c.planner.cell = j.value("cell_m", c.planner.cell);
    c.planner.headingBins = j.value("heading_bins", c.planner.headingBins);
    c.planner.step = j.value("step_m", c.planner.step);
    c.bankFraction = j.value("bank_fraction", c.bankFraction);
    c.safety = j.value("safety_m", c.safety);
    return c;
}

Result build(const Aircraft& ac, const MissionSpec& m, const Config& cfg) {
    Result r;
    const scenario::Scenario& sc = m.flight;
    r.radius = sc.V * sc.V / (isa::g0 * std::tan(cfg.bankFraction * sc.bankMax));

    auto t0 = std::chrono::steady_clock::now();
    r.plan = plan::planMission({sc.x0, sc.y0, sc.heading0}, m.waypoints, m.goal, r.radius, m.zones, m.hLow(), m.hHigh(),
                               cfg.safety, cfg.planner);
    r.plannerMillis = millisSince(t0);
    if (!r.plan.found) {
        r.message = "planner found no path after " + std::to_string(r.plan.expansions) + " expansions";
        return r;
    }

    t0 = std::chrono::steady_clock::now();
    scenario::Scenario flown = sc;
    flown.path = r.plan.path;
    r.uf = scenario::fly(ac, flown, scenario::Model::PointMass);
    r.fc = scenario::fly(ac, flown, scenario::Model::RigidBody);
    r.guidanceMillis = millisSince(t0);
    r.ok = r.uf.completed;
    r.message = r.ok ? "ok" : "guidance did not reach the end of the path";
    return r;
}

ocp::Trajectory fromRun(const scenario::Run& r, int N) {
    if (r.t.size() < 2) throw std::invalid_argument("fromRun: run has fewer than two samples");
    ocp::Trajectory t;
    t.tf = r.t.back();
    t.X.resize(N);
    t.U.resize(N);
    for (int k = 0; k < N; ++k) {
        const double tk = t.tf * k / (N - 1);
        const auto it = std::upper_bound(r.t.begin(), r.t.end(), tk);
        const std::size_t i1 = std::min<std::size_t>(it - r.t.begin(), r.t.size() - 1), i0 = i1 ? i1 - 1 : 0;
        const double w = i1 == i0 ? 0.0 : (tk - r.t[i0]) / (r.t[i1] - r.t[i0]);
        t.X[k].resize(r.x[i0].size());
        t.U[k].resize(r.u[i0].size());
        for (std::size_t i = 0; i < t.X[k].size(); ++i) t.X[k][i] = (1.0 - w) * r.x[i0][i] + w * r.x[i1][i];
        for (std::size_t j = 0; j < t.U[k].size(); ++j) t.U[k][j] = (1.0 - w) * r.u[i0][j] + w * r.u[i1][j];
    }
    return t;
}

ocp::Trajectory coldUf(const Aircraft& ac, const MissionSpec& m, int N) {
    const Mission ms = m.ufMission(0.0, 0.0);
    const TrapezoidalUF tr(ac, ms, N, UfScales::fromAircraft(ac, norm(ms.end - ms.start)));
    return ocp::unpack(tr.layout(), coldStartGuess(tr, ac, ms));
}

ocp::Trajectory coldFc(const Aircraft& ac, const MissionSpec& m, int N) {
    const Vec x0 = fcStartState(ac, m);
    const Vec3 a{m.flight.x0, m.flight.y0, -m.hStart()}, b{m.goal.x, m.goal.y, -m.hEnd()};
    const Vec3 d = b - a;
    const double bearing = std::atan2(d[1], d[0]);
    const trim::FcTrim t = trim::rigidBody(ac, {m.flight.V, m.hStart(), 0.0, 0.0, 0.0});
    ocp::Trajectory out;
    out.tf = norm(d) / m.flight.V;
    out.X.assign(N, x0);
    out.U.assign(N, t.u);
    for (int k = 0; k < N; ++k) {
        const double s = static_cast<double>(k) / (N - 1);
        for (int i = 0; i < 3; ++i) out.X[k][i] = a[i] + s * d[i];
        out.X[k][fc::PSI] = k == 0 ? x0[fc::PSI] : ocp::unwrapNear(bearing, x0[fc::PSI]);
    }
    return out;
}

ocp::Trajectory ufToFc(const Aircraft& ac, const ocp::Trajectory& t, const Vec3& wind) {
    const int N = t.nodes();
    const double dt = t.tf / (N - 1);
    const AeroCoefficients& k = ac.aero;

    // Pass 1: incidence from the lift coefficient, attitude from the wind axes.
    Vec alpha(N), phi(N), theta(N), psi(N);
    std::vector<Mat3> Rnb(N);
    auto attitude = [&](int n) {
        const Vec& x = t.X[n];
        Rnb[n] = mul(fc::bodyToNed(x[uf::MU], x[uf::GAMMA], x[uf::CHI]), bodyToWind(alpha[n]));
        theta[n] = std::asin(std::clamp(-Rnb[n][2][0], -1.0, 1.0));
        phi[n] = std::atan2(Rnb[n][2][1], Rnb[n][2][2]);
        psi[n] = ocp::unwrapNear(std::atan2(Rnb[n][1][0], Rnb[n][0][0]), x[uf::CHI]);
    };
    for (int n = 0; n < N; ++n) {
        alpha[n] = (t.U[n][uf::CL] - k.CL0) / k.CLa;
        attitude(n);
    }

    // Pass 2: body rates from the Euler-angle derivatives.
    const Vec dphi = timeDerivative(phi, dt), dtheta = timeDerivative(theta, dt), dpsi = timeDerivative(psi, dt);

    ocp::Trajectory out;
    out.tf = t.tf;
    out.X.assign(N, Vec(fc::NX, 0.0));
    out.U.assign(N, Vec(fc::NU, 0.0));
    const Actuators& act = ac.actuators;
    const double b = ac.geometry.b, c = ac.geometry.c;
    for (int n = 0; n < N; ++n) {
        const Vec& x = t.X[n];
        const double V = x[uf::V];
        const double p = dphi[n] - dpsi[n] * std::sin(theta[n]);
        const double q = dtheta[n] * std::cos(phi[n]) + dpsi[n] * std::cos(theta[n]) * std::sin(phi[n]);
        const double r = -dtheta[n] * std::sin(phi[n]) + dpsi[n] * std::cos(theta[n]) * std::cos(phi[n]);
        const double qS = 0.5 * isa::density(-x[uf::Z]) * V * V * ac.geometry.S;
        const double ph = p * b / (2.0 * V), qh = q * c / (2.0 * V), rh = r * b / (2.0 * V);

        // Pass 3: incidence, elevator and thrust. The point mass pushes along
        // the velocity; the rigid body along its x axis, so the same along-track
        // force needs T / cos(alpha), whose T sin(alpha) relieves the wing.
        // For each thrust, lift and pitch-moment balance are linear in
        // (alpha, de):
        //   CLa alpha  + CLde de = CL - CL0 - CLq q^
        //   Cma alpha  + Cmde de = -(Cm0 + Cmq q^ + zT T / (qS c))
        double T = t.U[n][uf::THRUST], de = 0.0;
        for (int it = 0; it < 3; ++it) {
            T = t.U[n][uf::THRUST] / std::cos(alpha[n]);
            const double cl = t.U[n][uf::CL] - T * std::sin(alpha[n]) / qS - k.CL0 - k.CLq * qh;
            const double cm = -(k.Cm0 + k.Cmq * qh + ac.propulsion.thrust_z_offset * T / (qS * c));
            const double det = k.CLa * k.Cmde - k.CLde * k.Cma;
            de = std::clamp((k.CLa * cm - k.Cma * cl) / det, -act.de_max, act.de_max);
            alpha[n] = (cl - k.CLde * de) / k.CLa;
        }
        attitude(n);

        // Aileron and rudder: zero roll and yaw moment at zero sideslip.
        const double l0 = k.Cl0 + k.Clp * ph + k.Clr * rh, n0 = k.Cn0 + k.Cnp * ph + k.Cnr * rh;
        const double det = k.Clda * k.Cndr - k.Cldr * k.Cnda;
        double da = 0.0, dr = 0.0;
        if (std::fabs(det) > 1e-9) {
            da = (-l0 * k.Cndr + n0 * k.Cldr) / det;
            dr = (-n0 * k.Clda + l0 * k.Cnda) / det;
        }

        const Vec3 vAir{V * std::cos(alpha[n]), 0.0, V * std::sin(alpha[n])};
        const Vec3 vb = vAir + ofc::mul(transpose(Rnb[n]), wind);
        out.X[n] = {x[uf::X], x[uf::Y], x[uf::Z], vb[0], vb[1], vb[2], phi[n], theta[n], psi[n], p, q, r, x[uf::MASS]};
        out.U[n] = {T, de, std::clamp(da, -act.da_max, act.da_max), std::clamp(dr, -act.dr_max, act.dr_max)};
    }
    return out;
}

Vec fcStartState(const Aircraft& ac, const MissionSpec& m) {
    const trim::FcTrim t = trim::rigidBody(ac, {m.flight.V, m.hStart(), 0.0, 0.0, 0.0});
    if (!t.converged) throw std::runtime_error("mission " + m.id + ": rigid-body trim did not converge");
    Vec x = t.x;
    x[fc::XN] = m.flight.x0;
    x[fc::YE] = m.flight.y0;
    x[fc::PSI] = m.flight.heading0;
    return x;
}

} // namespace ofc::ws
