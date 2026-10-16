#include "ofc/scenario.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "ofc/atmosphere.hpp"
#include "ofc/numerics.hpp"
#include "ofc/point_mass.hpp"
#include "ofc/rigid_body.hpp"
#include "ofc/trim.hpp"

namespace ofc::scenario {

namespace {

// ── Guidance (shared by both models) ────────────────────────────────────
constexpr double kAltitude = 0.01; // flight-path angle per metre of altitude error [rad/m]
constexpr double kHeading = 1.5;   // bank per radian of heading error [-]

struct Guidance {
    const Scenario& sc;
    double hCmd() const { return sc.h0 + sc.climb; }
    double chiCmd(double s) const { return s < sc.turnStart ? 0.0 : sc.turn; }
    double gammaCmd(double h) const { return std::clamp(kAltitude * (hCmd() - h), -sc.gammaMax, sc.gammaMax); }
    double bankCmd(double s, double chi) const {
        return std::clamp(kHeading * (chiCmd(s) - chi), -sc.bankMax, sc.bankMax);
    }
};

double timeBudget(const Scenario& sc) { return 3.0 * sc.distance / sc.V + 60.0; }

// ── UF: dynamic inversion of the point-mass equations ───────────────────
Vec pointMassControl(const Aircraft& ac, const Guidance& g, const Vec& x, double s) {
    constexpr double kGamma = 1.0, kBank = 2.0, kSpeed = 0.5;
    const double h = -x[uf::Z], Va = x[uf::V], m = x[uf::MASS], gam = x[uf::GAMMA], mu = x[uf::MU];
    const double qS = 0.5 * isa::density(h) * Va * Va * ac.geometry.S;

    const double gammaDotDes = kGamma * (g.gammaCmd(h) - gam);
    const double L = m * (Va * gammaDotDes + isa::g0 * std::cos(gam)) / std::cos(mu);
    const double CL = std::clamp(L / qS, -0.3, ac.clMax());
    const double p = std::clamp(kBank * (g.bankCmd(s, x[uf::CHI]) - mu), -ac.actuators.p_max, ac.actuators.p_max);
    const double T =
        qS * (ac.aero.CD0 + ac.aero.k * CL * CL) + m * isa::g0 * std::sin(gam) + m * kSpeed * (g.sc.V - Va);
    return {std::clamp(T, 0.0, ac.maxThrust(Va, h)), CL, p};
}

// ── FC: attitude and speed autopilot around the wings-level trim ────────
struct Autopilot {
    const Aircraft& ac;
    const Guidance& g;
    trim::FcTrim trim;
    double CLtrim = 0.0;

    Vec control(const Vec& x, double s, double chi) const {
        using namespace fc;
        constexpr double kTheta = 2.0, kQ = 0.3, kPhi = 1.0, kP = 0.15, kBeta = 0.5, kSpeed = 0.5;
        const AirData air = airData(x, {0.0, 0.0, 0.0});
        const double h = -x[ZD], m = x[MASS];
        const double gammaCmd = g.gammaCmd(h);
        const double phiCmd = g.bankCmd(s, chi);

        // Pitch: trim incidence + extra incidence for the turn's load factor + commanded climb.
        const double nTurn = 1.0 / std::max(std::cos(x[PHI]), 0.5);
        const double thetaCmd = trim.alpha + CLtrim * (nTurn - 1.0) / ac.aero.CLa + gammaCmd;
        const double de = trim.u[ELEVATOR] + kTheta * (x[THETA] - thetaCmd) + kQ * x[Q];
        const double da = kPhi * (phiCmd - x[PHI]) - kP * x[P];
        const double dr = -kBeta * air.beta;
        const double T = trim.u[THRUST] + m * isa::g0 * std::sin(gammaCmd) + m * kSpeed * (g.sc.V - air.Va);

        const Actuators& a = ac.actuators;
        return {std::clamp(T, 0.0, ac.maxThrust(air.Va, h)), std::clamp(de, -a.de_max, a.de_max),
                std::clamp(da, -a.da_max, a.da_max), std::clamp(dr, -a.dr_max, a.dr_max)};
    }
};

Vec3 nedVelocity(const Vec& x) {
    return mul(fc::bodyToNed(x[fc::PHI], x[fc::THETA], x[fc::PSI]), {x[fc::U], x[fc::V], x[fc::W]});
}

} // namespace

Scenario Scenario::fromJson(const nlohmann::json& j, const Aircraft& ac) {
    Scenario sc;
    sc.id = j.value("id", std::string("SC"));
    sc.title = j.value("title", sc.id);
    sc.h0 = j.value("h0_m", 300.0);
    sc.climb = j.value("climb_m", 0.0);
    sc.turn = deg2rad(j.value("turn_deg", 0.0));
    sc.turnStart = j.value("turn_start_m", 1000.0);
    sc.distance = j.value("distance_m", 3000.0);
    sc.V = j.value("V_mps", 0.0);
    sc.bankMax = deg2rad(j.value("bank_deg", 0.0));
    sc.gammaMax = deg2rad(j.value("gamma_deg", 0.0));
    if (sc.V <= 0.0) sc.V = ac.envelope.V_cruise;
    if (sc.bankMax <= 0.0) sc.bankMax = 0.75 * ac.envelope.phi_max;
    if (sc.gammaMax <= 0.0) sc.gammaMax = 0.5 * ac.envelope.gamma_max;
    if (sc.distance <= 0.0) throw std::invalid_argument("scenario " + sc.id + ": distance_m must be > 0");
    return sc;
}

Run fly(const Aircraft& ac, const Scenario& sc, Model model, double dt) {
    const Guidance g{sc};
    Run r;
    r.model = model;
    const double tMax = timeBudget(sc);
    double s = 0.0;

    if (model == Model::PointMass) {
        Vec x = trim::pointMass(ac, {sc.V, sc.h0, 0.0, 0.0, 0.0}).x;
        for (double t = 0.0;; t += dt) {
            const Vec u = pointMassControl(ac, g, x, s);
            r.t.push_back(t);
            r.x.push_back(x);
            r.u.push_back(u);
            r.s.push_back(s);
            if (s >= sc.distance || t > tMax) break;
            x = num::rk4Step([&](double, const Vec& xx) { return uf::rhs(ac, xx, u); }, t, x, dt);
            s += dt * x[uf::V] * std::cos(x[uf::GAMMA]);
        }
    } else {
        Autopilot ap{ac, g, trim::rigidBody(ac, {sc.V, sc.h0, 0.0, 0.0, 0.0})};
        if (!ap.trim.converged) throw std::runtime_error("scenario " + sc.id + ": FC trim did not converge");
        ap.CLtrim = ac.aero.CL0 + ac.aero.CLa * ap.trim.alpha + ac.aero.CLde * ap.trim.u[fc::ELEVATOR];
        Vec x = ap.trim.x;
        double chi = 0.0; // unwrapped course
        for (double t = 0.0;; t += dt) {
            const Vec u = ap.control(x, s, chi);
            r.t.push_back(t);
            r.x.push_back(x);
            r.u.push_back(u);
            r.s.push_back(s);
            if (s >= sc.distance || t > tMax) break;
            x = num::rk4Step([&](double, const Vec& xx) { return fc::rhs(ac, xx, u); }, t, x, dt);
            const Vec3 v = nedVelocity(x);
            const double course = std::atan2(v[1], v[0]);
            chi += std::remainder(course - chi, 2.0 * kPi);
            s += dt * std::hypot(v[0], v[1]);
        }
    }
    r.flightTime = r.t.back();
    const int massIdx = model == Model::PointMass ? static_cast<int>(uf::MASS) : static_cast<int>(fc::MASS);
    r.fuel = r.x.front()[massIdx] - r.x.back()[massIdx];
    r.completed = r.s.back() >= sc.distance;
    return r;
}

Sample sampleOf(const Run& r, std::size_t k) {
    const Vec& x = r.x[k];
    if (r.model == Model::PointMass)
        return {x[uf::X], x[uf::Y], x[uf::Z], x[uf::V], x[uf::GAMMA], x[uf::CHI], x[uf::MU], x[uf::MASS]};
    const Vec3 v = nedVelocity(x);
    const double speed = norm(v);
    return {x[fc::XN],
            x[fc::YE],
            x[fc::ZD],
            fc::airData(x, {0.0, 0.0, 0.0}).Va,
            std::asin(std::clamp(-v[2] / speed, -1.0, 1.0)),
            std::atan2(v[1], v[0]),
            x[fc::PHI],
            x[fc::MASS]};
}

} // namespace ofc::scenario
