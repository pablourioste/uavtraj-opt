#include "ofc/ocp_fc.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "ofc/atmosphere.hpp"
#include "ofc/point_mass.hpp"
#include "ofc/rigid_body.hpp"

namespace ofc::ocp {

using namespace fc;

namespace {

double pathLengthOf(const Trajectory& t) {
    double L = 0.0;
    for (int k = 1; k < t.nodes(); ++k) {
        const double dx = t.X[k][XN] - t.X[k - 1][XN], dy = t.X[k][YE] - t.X[k - 1][YE];
        const double dz = t.X[k][ZD] - t.X[k - 1][ZD];
        L += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return std::max(L, 1.0);
}

} // namespace

FcProblem::FcProblem(const Aircraft& ac, const FcMission& mission, const Trajectory& guess,
                     std::vector<WaypointNode> waypoints, Reference ref)
    : ac_(ac), mission_(mission), layout_{guess.nodes(), NX, NU} {
    if (layout_.N < 2) throw std::invalid_argument("FcProblem: need at least two nodes");
    if (static_cast<int>(mission_.x0.size()) != NX) throw std::invalid_argument("FcProblem: x0 must have 13 states");
    mission_.headingEnd = unwrapNear(mission_.headingEnd, guess.X.back()[PSI]);

    const Envelope& e = ac.envelope;
    const Actuators& a = ac.actuators;
    const double Vref = e.V_cruise, L = ref.length > 0.0 ? ref.length : pathLengthOf(guess);
    sx_ = {L,           L,         L,       Vref,    0.2 * Vref, 0.2 * Vref, e.phi_max,
           e.theta_max, 2.0 * kPi, a.p_max, a.q_max, a.r_max,    ac.mass.m0};
    su_ = {ac.maxThrust(Vref, 0.0), a.de_max, a.da_max, a.dr_max};
    tfRef_ = std::max(ref.time > 0.0 ? ref.time : guess.tf, 1.0);
    fuelRef_ = uf::fuelFlowCoefficient(ac) * su_[THRUST] * Vref * tfRef_;
    for (const env::NoFlyZone& zone : mission_.nfz)
        nfzActive_.push_back(zone.activeBetween(mission_.hLow, mission_.hHigh));

    // Structural sparsity of df/dx and df/du at a generic point.
    const Vec xg{100.0, 50.0, -300.0, Vref, 1.5, 0.8, 0.1, 0.05, 0.3, 0.05, 0.04, 0.03, ac.mass.m0};
    const Vec ug{0.3 * su_[THRUST], 0.02, 0.01, 0.01};
    Vec A, B;
    modelJacobian(xg, ug, A, B);
    for (double v : A) patternA_.push_back(v != 0.0);
    for (double v : B) patternB_.push_back(v != 0.0);

    ModelInfo info;
    info.stateNames = {"x", "y", "z", "u", "v", "w", "phi", "theta", "psi", "p", "q", "r", "m"};
    info.controlNames = {"T", "de", "da", "dr"};
    info.pathNames = {"stall",    "power", "v_ne",      "alpha_max", "alpha_min", "sideslip", "load_max", "load_min",
                      "altitude", "bank",  "theta_max", "theta_min", "rate_p",    "rate_q",   "rate_r",   "thrust_min"};
    for (const env::NoFlyZone& zone : mission_.nfz) info.pathNames.push_back("nfz:" + zone.id);
    info.stateScale = sx_;
    info.controlScale = su_;
    info.tfScale = tfRef_;
    info.stateBox.resize(NX);
    info.stateBox[U] = {0.3 * Vref, 1.3 * e.V_ne};
    info.stateBox[V] = info.stateBox[W] = {-0.5 * e.V_ne, 0.5 * e.V_ne};
    info.stateBox[PHI] = {-deg2rad(80.0), deg2rad(80.0)};
    info.stateBox[THETA] = {-deg2rad(60.0), deg2rad(60.0)};
    info.stateBox[P] = {-2.0 * a.p_max, 2.0 * a.p_max};
    info.stateBox[Q] = {-2.0 * a.q_max, 2.0 * a.q_max};
    info.stateBox[R] = {-2.0 * a.r_max, 2.0 * a.r_max};
    info.stateBox[MASS] = {ac.mass.m_empty, 1.001 * ac.mass.m0};
    info.controlBox = {
        {0.0, ac.maxThrust(0.0, 0.0)}, {-a.de_max, a.de_max}, {-a.da_max, a.da_max}, {-a.dr_max, a.dr_max}};
    info.tfBox = {0.3 * tfRef_, 3.0 * tfRef_};
    info.idxX = XN, info.idxY = YE;

    std::vector<BoundaryRow> boundary;
    for (int i = 0; i < NX; ++i)
        boundary.push_back({"x0." + info.stateNames[i], layout_.idxState(0, i), mission_.x0[i], sx_[i]});
    const int K = layout_.N - 1;
    for (int i = 0; i < 3; ++i)
        boundary.push_back({"xf." + info.stateNames[i], layout_.idxState(K, i), mission_.end[i], sx_[i]});
    boundary.push_back({"xf.psi", layout_.idxState(K, PSI), mission_.headingEnd, sx_[PSI]});

    setup(layout_, std::move(info), std::move(boundary), pack(layout_, guess), std::move(waypoints));
}

void FcProblem::modelJacobian(const Vec& x, const Vec& u, Vec& A, Vec& B) const {
    A.assign(NX * NX, 0.0);
    B.assign(NX * NU, 0.0);
    Vec xp = x, up = u;
    for (int j = 0; j < NX; ++j) {
        const double h = 1e-6 * std::max(std::fabs(x[j]), sx_[j]);
        xp[j] = x[j] + h;
        const Vec fp = rhs(ac_, xp, u, mission_.wind);
        xp[j] = x[j] - h;
        const Vec fm = rhs(ac_, xp, u, mission_.wind);
        xp[j] = x[j];
        for (int i = 0; i < NX; ++i) A[i * NX + j] = (fp[i] - fm[i]) / (2.0 * h);
    }
    for (int j = 0; j < NU; ++j) {
        const double h = 1e-6 * std::max(std::fabs(u[j]), su_[j]);
        up[j] = u[j] + h;
        const Vec fp = rhs(ac_, x, up, mission_.wind);
        up[j] = u[j] - h;
        const Vec fm = rhs(ac_, x, up, mission_.wind);
        up[j] = u[j];
        for (int i = 0; i < NX; ++i) B[i * NU + j] = (fp[i] - fm[i]) / (2.0 * h);
    }
}

Vec FcProblem::defects(const Vec& z) const {
    const Trajectory t = unpack(layout_, z);
    std::vector<Vec> F(layout_.N);
    for (int k = 0; k < layout_.N; ++k) F[k] = rhs(ac_, t.X[k], t.U[k], mission_.wind);
    return trapezoidalDefects(layout_, t.tf, t.X, F, sx_);
}

std::vector<Triplet> FcProblem::defectJacobian(const Vec& z) const {
    const Trajectory t = unpack(layout_, z);
    std::vector<Vec> F(layout_.N), A(layout_.N), B(layout_.N);
    for (int k = 0; k < layout_.N; ++k) {
        F[k] = rhs(ac_, t.X[k], t.U[k], mission_.wind);
        modelJacobian(t.X[k], t.U[k], A[k], B[k]);
    }
    return trapezoidalJacobian(layout_, t.tf, F, A, B, sx_, patternA_, patternB_);
}

Vec FcProblem::pathAt(const Vec& x, const Vec& u) const {
    const Envelope& e = ac_.envelope;
    const Actuators& a = ac_.actuators;
    const AirData air = airData(x, mission_.wind);
    const double alt = -x[ZD], m = x[MASS];
    const AeroCoeffs C = coefficients(ac_, air, x, u);
    const double qS = 0.5 * isa::density(alt) * air.Va * air.Va * ac_.geometry.S;
    const double n = qS * C.CL / (m * isa::g0);
    auto sq = [](double v, double lim) { return v * v / (lim * lim) - 1.0; };

    Vec h;
    h.reserve(16 + mission_.nfz.size());
    h.push_back(1.0 - air.Va / ac_.stallSpeed(alt, m));
    h.push_back(u[THRUST] * air.Va / (ac_.propulsion.eta_p * ac_.maxPower(alt)) - 1.0);
    h.push_back(air.Va / e.V_ne - 1.0);
    h.push_back((air.alpha - e.alpha_max) / std::max(std::fabs(e.alpha_max), 0.05));
    h.push_back((e.alpha_min - air.alpha) / std::max(std::fabs(e.alpha_min), 0.05));
    h.push_back(sq(air.beta, mission_.sideslipMax));
    h.push_back(n / e.n_max - 1.0);
    h.push_back((e.n_min - n) / std::max(1.0, std::fabs(e.n_min)));
    h.push_back((e.h_min - alt) / std::max(1.0, e.h_min));
    h.push_back(sq(x[PHI], e.phi_max));
    h.push_back((x[THETA] - e.theta_max) / std::fabs(e.theta_max));
    h.push_back((e.theta_min - x[THETA]) / std::fabs(e.theta_min));
    h.push_back(sq(x[P], a.p_max));
    h.push_back(sq(x[Q], a.q_max));
    h.push_back(sq(x[R], a.r_max));
    h.push_back(-u[THRUST] / su_[THRUST]);
    for (std::size_t j = 0; j < mission_.nfz.size(); ++j) {
        const env::NoFlyZone& zone = mission_.nfz[j];
        const double scale = std::max(zone.margin, 25.0);
        h.push_back(nfzActive_[j] ? (zone.margin - zone.signedDistance(x[XN], x[YE])) / scale : -1.0);
    }
    return h;
}

namespace {
constexpr int kSmoothStates[] = {PHI, THETA}; // attitude increments penalised by betaState
}

double FcProblem::cost(const Vec& z) const {
    const int N = layout_.N;
    const double a = mission_.alpha;
    const double fuel = mission_.x0[MASS] - z[layout_.idxState(N - 1, MASS)];
    double smooth = 0.0, smoothState = 0.0;
    for (int k = 0; k < N - 1; ++k) {
        for (int j = 0; j < NU; ++j) {
            const double du = (z[layout_.idxControl(k + 1, j)] - z[layout_.idxControl(k, j)]) / su_[j];
            smooth += du * du;
        }
        for (int i : kSmoothStates) {
            const double dx = (z[layout_.idxState(k + 1, i)] - z[layout_.idxState(k, i)]) / sx_[i];
            smoothState += dx * dx;
        }
    }
    return (1.0 - a) * z[layout_.idxTf()] / tfRef_ + a * fuel / fuelRef_ +
           (mission_.betaControl * smooth + mission_.betaState * smoothState) / (N - 1);
}

Vec FcProblem::costGradient(const Vec& z) const {
    const int N = layout_.N;
    const double a = mission_.alpha, w = mission_.betaControl / (N - 1), ws = mission_.betaState / (N - 1);
    Vec g(layout_.numVariables(), 0.0);
    g[layout_.idxTf()] = (1.0 - a) / tfRef_;
    g[layout_.idxState(N - 1, MASS)] = -a / fuelRef_;
    for (int k = 0; k < N - 1; ++k) {
        for (int j = 0; j < NU; ++j) {
            const double d =
                2.0 * w * (z[layout_.idxControl(k + 1, j)] - z[layout_.idxControl(k, j)]) / (su_[j] * su_[j]);
            g[layout_.idxControl(k + 1, j)] += d;
            g[layout_.idxControl(k, j)] -= d;
        }
        for (int i : kSmoothStates) {
            const double d = 2.0 * ws * (z[layout_.idxState(k + 1, i)] - z[layout_.idxState(k, i)]) / (sx_[i] * sx_[i]);
            g[layout_.idxState(k + 1, i)] += d;
            g[layout_.idxState(k, i)] -= d;
        }
    }
    return g;
}

} // namespace ofc::ocp
