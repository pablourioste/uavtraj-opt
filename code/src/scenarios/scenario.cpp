#include "ofc/scenario.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "ofc/atmosphere.hpp"
#include "ofc/numerics.hpp"
#include "ofc/point_mass.hpp"
#include "ofc/rigid_body.hpp"
#include "ofc/trim.hpp"

namespace ofc::scenario {

namespace {

// ── Guidance (shared by both models) ────────────────────────────────────
constexpr double kAltitude = 0.01;        // flight-path angle per metre of altitude error [rad/m]
constexpr double kHeading = 1.5;          // bank per radian of heading error [-]
constexpr double kL1Time = 2.5;           // L1 look-ahead distance = kL1Time * V [s]
constexpr std::size_t kSearchWindow = 60; // path samples searched ahead for the nearest point

class Guidance {
public:
    explicit Guidance(const Scenario& s) : sc(s) {
        cum_.assign(sc.path.size(), 0.0);
        for (std::size_t i = 1; i < sc.path.size(); ++i)
            cum_[i] = cum_[i - 1] + std::hypot(sc.path[i].x - sc.path[i - 1].x, sc.path[i].y - sc.path[i - 1].y);
    }

    const Scenario& sc;

    bool followsPath() const { return sc.path.size() >= 2; }
    double pathLength() const { return followsPath() ? cum_.back() : 0.0; }
    double hCmd() const { return sc.h0 + sc.climb; }
    double chiCmd(double s) const { return sc.heading0 + (s < sc.turnStart ? 0.0 : sc.turn); }
    double gammaCmd(double h) const { return std::clamp(kAltitude * (hCmd() - h), -sc.gammaMax, sc.gammaMax); }

    // Arc length of the point of the path nearest to (x, y), searching forward
    // from the previous match so that loops in the path are not skipped.
    double progress(double x, double y) const {
        const std::size_t last = std::min(sc.path.size() - 1, idx_ + kSearchWindow);
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t i = idx_; i <= last; ++i) {
            const double d = std::hypot(sc.path[i].x - x, sc.path[i].y - y);
            if (d < best) best = d, idx_ = i;
        }
        if (idx_ + 1 >= sc.path.size()) return cum_.back();
        const plan::Pose2 &a = sc.path[idx_], &b = sc.path[idx_ + 1];
        const double seg = cum_[idx_ + 1] - cum_[idx_];
        const double t = seg > 0.0 ? ((x - a.x) * (b.x - a.x) + (y - a.y) * (b.y - a.y)) / (seg * seg) : 0.0;
        return cum_[idx_] + std::clamp(t, 0.0, 1.0) * seg;
    }

    // Point at arc length s; beyond the end the last heading is extended.
    plan::Pose2 at(double s) const {
        if (s >= cum_.back()) return plan::advance(sc.path.back(), 0.0, s - cum_.back());
        const std::size_t i = std::upper_bound(cum_.begin(), cum_.end(), s) - cum_.begin() - 1;
        const double w = (s - cum_[i]) / std::max(cum_[i + 1] - cum_[i], 1e-9);
        return {sc.path[i].x + w * (sc.path[i + 1].x - sc.path[i].x),
                sc.path[i].y + w * (sc.path[i + 1].y - sc.path[i].y), sc.path[i].psi};
    }

    // Bank command: L1 path following when a path is set, heading step otherwise.
    double bankCmd(double s, double x, double y, double chi, double V) const {
        if (!followsPath()) return std::clamp(kHeading * (chiCmd(s) - chi), -sc.bankMax, sc.bankMax);
        const plan::Pose2 target = at(progress(x, y) + kL1Time * V);
        const double dx = target.x - x, dy = target.y - y;
        const double eta = std::remainder(std::atan2(dy, dx) - chi, 2.0 * kPi);
        const double accel = 2.0 * V * V * std::sin(eta) / std::max(std::hypot(dx, dy), 1.0);
        return std::clamp(std::atan(accel / isa::g0), -sc.bankMax, sc.bankMax);
    }

private:
    std::vector<double> cum_;
    mutable std::size_t idx_ = 0;
};

double timeBudget(const Guidance& g) {
    const double dist = g.followsPath() ? g.pathLength() : g.sc.distance;
    return 3.0 * dist / g.sc.V + 60.0;
}

// ── UF: dynamic inversion of the point-mass equations ───────────────────
Vec pointMassControl(const Aircraft& ac, const Guidance& g, const Vec& x, double s) {
    constexpr double kGamma = 1.0, kBank = 2.0, kSpeed = 0.5;
    const double h = -x[uf::Z], Va = x[uf::V], m = x[uf::MASS], gam = x[uf::GAMMA], mu = x[uf::MU];
    const double qS = 0.5 * isa::density(h) * Va * Va * ac.geometry.S;

    const double gammaDotDes = kGamma * (g.gammaCmd(h) - gam);
    const double L = m * (Va * gammaDotDes + isa::g0 * std::cos(gam)) / std::cos(mu);
    const double CL = std::clamp(L / qS, -0.3, ac.clMax());
    const double muCmd = g.bankCmd(s, x[uf::X], x[uf::Y], x[uf::CHI], Va * std::cos(gam));
    const double p = std::clamp(kBank * (muCmd - mu), -ac.actuators.p_max, ac.actuators.p_max);
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

    Vec control(const Vec& x, double s, double chi, double groundSpeed) const {
        using namespace fc;
        constexpr double kTheta = 2.0, kQ = 0.3, kPhi = 1.0, kP = 0.15, kBeta = 0.5, kSpeed = 0.5;
        const AirData air = airData(x, {0.0, 0.0, 0.0});
        const double h = -x[ZD], m = x[MASS];
        const double gammaCmd = g.gammaCmd(h);
        const double phiCmd = g.bankCmd(s, x[XN], x[YE], chi, groundSpeed);

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
    if (j.contains("start")) sc.x0 = j.at("start").at(0).get<double>(), sc.y0 = j.at("start").at(1).get<double>();
    sc.heading0 = deg2rad(j.value("start_heading_deg", 0.0));
    if (sc.V <= 0.0) sc.V = ac.envelope.V_cruise;
    if (sc.bankMax <= 0.0) sc.bankMax = 0.75 * ac.envelope.phi_max;
    if (sc.gammaMax <= 0.0) sc.gammaMax = 0.5 * ac.envelope.gamma_max;
    if (sc.distance <= 0.0) throw std::invalid_argument("scenario " + sc.id + ": distance_m must be > 0");
    return sc;
}

Run fly(const Aircraft& ac, const Scenario& sc, Model model, double dt) {
    const Guidance g(sc);
    Run r;
    r.model = model;
    const double tMax = timeBudget(g);
    const double goal = g.followsPath() ? g.pathLength() - 0.5 : sc.distance;
    double s = 0.0;

    if (model == Model::PointMass) {
        Vec x = trim::pointMass(ac, {sc.V, sc.h0, 0.0, 0.0, 0.0}).x;
        x[uf::X] = sc.x0, x[uf::Y] = sc.y0, x[uf::CHI] = sc.heading0;
        for (double t = 0.0;; t += dt) {
            const Vec u = pointMassControl(ac, g, x, s);
            r.t.push_back(t);
            r.x.push_back(x);
            r.u.push_back(u);
            r.s.push_back(s);
            if (s >= goal || t > tMax) break;
            x = num::rk4Step([&](double, const Vec& xx) { return uf::rhs(ac, xx, u); }, t, x, dt);
            s = g.followsPath() ? g.progress(x[uf::X], x[uf::Y]) : s + dt * x[uf::V] * std::cos(x[uf::GAMMA]);
        }
    } else {
        Autopilot ap{ac, g, trim::rigidBody(ac, {sc.V, sc.h0, 0.0, 0.0, 0.0})};
        if (!ap.trim.converged) throw std::runtime_error("scenario " + sc.id + ": FC trim did not converge");
        ap.CLtrim = ac.aero.CL0 + ac.aero.CLa * ap.trim.alpha + ac.aero.CLde * ap.trim.u[fc::ELEVATOR];
        Vec x = ap.trim.x;
        x[fc::XN] = sc.x0, x[fc::YE] = sc.y0, x[fc::PSI] = sc.heading0;
        double chi = sc.heading0; // unwrapped course
        double groundSpeed = sc.V;
        for (double t = 0.0;; t += dt) {
            const Vec u = ap.control(x, s, chi, groundSpeed);
            r.t.push_back(t);
            r.x.push_back(x);
            r.u.push_back(u);
            r.s.push_back(s);
            if (s >= goal || t > tMax) break;
            x = num::rk4Step([&](double, const Vec& xx) { return fc::rhs(ac, xx, u); }, t, x, dt);
            const Vec3 v = nedVelocity(x);
            chi += std::remainder(std::atan2(v[1], v[0]) - chi, 2.0 * kPi);
            groundSpeed = std::hypot(v[0], v[1]);
            s = g.followsPath() ? g.progress(x[fc::XN], x[fc::YE]) : s + dt * groundSpeed;
        }
    }
    r.flightTime = r.t.back();
    const int massIdx = model == Model::PointMass ? static_cast<int>(uf::MASS) : static_cast<int>(fc::MASS);
    r.fuel = r.x.front()[massIdx] - r.x.back()[massIdx];
    r.completed = r.s.back() >= goal;
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
