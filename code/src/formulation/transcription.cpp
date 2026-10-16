#include "ofc/transcription.hpp"

#include <cmath>
#include <stdexcept>

#include "ofc/atmosphere.hpp"
#include "ofc/trim.hpp"

namespace ofc {

using namespace uf;

namespace {

// Structural sparsity of A = df/dx and B = df/du, found by evaluating the
// analytic Jacobian at a generic point (no symmetric angles, nonzero bank).
struct Pattern {
    std::vector<bool> A, B;
};

Pattern structuralPattern(const Aircraft& ac) {
    const Vec x{100.0, 50.0, -300.0, 1.1 * ac.envelope.V_cruise, 0.05, 0.3, 0.2, ac.mass.m0};
    const Vec u{0.3 * ac.maxThrust(ac.envelope.V_cruise, 300.0), 0.6, 0.01};
    Vec A, B;
    jacobian(ac, x, u, A, B);
    Pattern p;
    p.A.resize(A.size());
    p.B.resize(B.size());
    for (std::size_t i = 0; i < A.size(); ++i) p.A[i] = A[i] != 0.0;
    for (std::size_t i = 0; i < B.size(); ++i) p.B[i] = B[i] != 0.0;
    return p;
}

double speedStart(const Aircraft& ac, const Mission& m) {
    return m.speedStart > 0.0 ? m.speedStart : ac.envelope.V_cruise;
}

} // namespace

UfScales UfScales::fromAircraft(const Aircraft& ac, double distance) {
    UfScales s;
    const double L = std::max(distance, 1.0);
    const double Vref = ac.envelope.V_cruise;
    s.x = {L, L, L, Vref, std::max(ac.envelope.gamma_max, 1e-3), 2.0 * kPi, ac.envelope.phi_max, ac.mass.m0};
    s.u = {ac.maxThrust(Vref, 0.0), ac.clMax(), ac.actuators.p_max};
    s.tf = L / Vref;
    return s;
}

TrapezoidalUF::TrapezoidalUF(const Aircraft& ac, const Mission& mission, int N, UfScales scales)
    : ac_(ac), mission_(mission), N_(N), scales_(std::move(scales)) {
    if (N_ < 2) throw std::invalid_argument("TrapezoidalUF: N must be >= 2");
    if (static_cast<int>(scales_.x.size()) != NX || static_cast<int>(scales_.u.size()) != NU)
        throw std::invalid_argument("TrapezoidalUF: scale vectors have the wrong size");
    energyRef_ = scales_.u[THRUST] * scales_.x[V] * scales_.tf;
    Pattern p = structuralPattern(ac_);
    patternA_ = std::move(p.A);
    patternB_ = std::move(p.B);
}

Vec TrapezoidalUF::pack(double tf, const std::vector<Vec>& X, const std::vector<Vec>& U) const {
    Vec z(numVariables());
    z[idxTf()] = tf;
    for (int k = 0; k < N_; ++k) {
        for (int i = 0; i < NX; ++i) z[idxState(k, i)] = X[k][i];
        for (int j = 0; j < NU; ++j) z[idxControl(k, j)] = U[k][j];
    }
    return z;
}

void TrapezoidalUF::unpack(const Vec& z, double& tf, std::vector<Vec>& X, std::vector<Vec>& U) const {
    tf = z[idxTf()];
    X.assign(N_, Vec(NX));
    U.assign(N_, Vec(NU));
    for (int k = 0; k < N_; ++k) {
        for (int i = 0; i < NX; ++i) X[k][i] = z[idxState(k, i)];
        for (int j = 0; j < NU; ++j) U[k][j] = z[idxControl(k, j)];
    }
}

Vec TrapezoidalUF::defects(const Vec& z) const {
    double tf;
    std::vector<Vec> X, U;
    unpack(z, tf, X, U);
    const double dt = tf / (N_ - 1);

    std::vector<Vec> F(N_);
    for (int k = 0; k < N_; ++k) F[k] = rhs(ac_, X[k], U[k], mission_.wind);

    Vec d(numDefects());
    for (int k = 0; k < N_ - 1; ++k)
        for (int i = 0; i < NX; ++i)
            d[k * NX + i] = (X[k + 1][i] - X[k][i] - 0.5 * dt * (F[k][i] + F[k + 1][i])) / scales_.x[i];
    return d;
}

std::vector<Triplet> TrapezoidalUF::defectJacobian(const Vec& z) const {
    double tf;
    std::vector<Vec> X, U;
    unpack(z, tf, X, U);
    const double dt = tf / (N_ - 1);

    std::vector<Vec> F(N_), A(N_), B(N_);
    for (int k = 0; k < N_; ++k) {
        F[k] = rhs(ac_, X[k], U[k], mission_.wind);
        jacobian(ac_, X[k], U[k], A[k], B[k]);
    }

    std::vector<Triplet> J;
    J.reserve(static_cast<std::size_t>(N_ - 1) * NX * (2 * NX + 2 * NU + 1));
    for (int k = 0; k < N_ - 1; ++k) {
        for (int i = 0; i < NX; ++i) {
            const int row = k * NX + i;
            const double s = 1.0 / scales_.x[i];
            J.push_back({row, idxTf(), -s * (F[k][i] + F[k + 1][i]) / (2.0 * (N_ - 1))});
            for (int j = 0; j < NX; ++j) {
                const bool diag = i == j;
                if (!diag && !patternA_[i * NX + j]) continue;
                const double eye = diag ? 1.0 : 0.0;
                J.push_back({row, idxState(k, j), s * (-eye - 0.5 * dt * A[k][i * NX + j])});
                J.push_back({row, idxState(k + 1, j), s * (eye - 0.5 * dt * A[k + 1][i * NX + j])});
            }
            for (int j = 0; j < NU; ++j) {
                if (!patternB_[i * NU + j]) continue;
                J.push_back({row, idxControl(k, j), -s * 0.5 * dt * B[k][i * NU + j]});
                J.push_back({row, idxControl(k + 1, j), -s * 0.5 * dt * B[k + 1][i * NU + j]});
            }
        }
    }
    return J;
}

Vec TrapezoidalUF::boundaryResiduals(const Vec& z) const {
    const Mission& m = mission_;
    const Vec x0{m.start[0],   m.start[1],     m.start[2], speedStart(ac_, m),
                 m.gammaStart, m.headingStart, 0.0,        ac_.mass.m0};
    Vec r;
    r.reserve(NX + 4);
    for (int i = 0; i < NX; ++i) r.push_back((z[idxState(0, i)] - x0[i]) / scales_.x[i]);
    const int K = N_ - 1;
    for (int i = 0; i < 3; ++i) r.push_back((z[idxState(K, i)] - m.end[i]) / scales_.x[i]);
    r.push_back((z[idxState(K, CHI)] - m.headingEnd) / scales_.x[CHI]);
    return r;
}

const std::vector<std::string>& TrapezoidalUF::pathConstraintNames() {
    static const std::vector<std::string> names{"stall",     "power",    "v_ne",       "load_max",
                                                "load_min",  "altitude", "bank",       "gamma_max",
                                                "gamma_min", "cl_max",   "thrust_min", "bank_rate"};
    return names;
}

Vec TrapezoidalUF::pathConstraints(const Vec& z) const {
    const Envelope& e = ac_.envelope;
    const std::size_t nc = pathConstraintNames().size();
    Vec h;
    h.reserve(nc * N_);
    for (int k = 0; k < N_; ++k) {
        Vec x(NX), u(NU);
        for (int i = 0; i < NX; ++i) x[i] = z[idxState(k, i)];
        for (int j = 0; j < NU; ++j) u[j] = z[idxControl(k, j)];
        const double alt = -x[Z];
        const double n = loadFactor(ac_, x, u);
        h.push_back(1.0 - x[V] / ac_.stallSpeed(alt, x[MASS]));
        h.push_back(u[THRUST] * x[V] / (ac_.propulsion.eta_p * ac_.maxPower(alt)) - 1.0);
        h.push_back(x[V] / e.V_ne - 1.0);
        h.push_back(n / e.n_max - 1.0);
        h.push_back((e.n_min - n) / std::max(1.0, std::fabs(e.n_min)));
        h.push_back((e.h_min - alt) / std::max(1.0, e.h_min));
        h.push_back(x[MU] * x[MU] / (e.phi_max * e.phi_max) - 1.0);
        h.push_back((x[GAMMA] - e.gamma_max) / std::fabs(e.gamma_max));
        h.push_back((e.gamma_min - x[GAMMA]) / std::fabs(e.gamma_min));
        h.push_back(u[CL] / ac_.clMax() - 1.0);
        h.push_back(-u[THRUST] / scales_.u[THRUST]);
        h.push_back(u[BANK_RATE] * u[BANK_RATE] / (ac_.actuators.p_max * ac_.actuators.p_max) - 1.0);
    }
    return h;
}

std::vector<TrapezoidalUF::PathConstraint> TrapezoidalUF::pathSummary(const Vec& z) const {
    const auto& names = pathConstraintNames();
    const Vec h = pathConstraints(z);
    std::vector<PathConstraint> out(names.size());
    for (std::size_t c = 0; c < names.size(); ++c) out[c].name = names[c];
    for (int k = 0; k < N_; ++k)
        for (std::size_t c = 0; c < names.size(); ++c) {
            const double v = h[k * names.size() + c];
            if (v > out[c].worst) {
                out[c].worst = v;
                out[c].node = k;
            }
        }
    return out;
}

double TrapezoidalUF::cost(const Vec& z) const {
    const double a = mission_.alpha, tf = z[idxTf()];
    const double dt = tf / (N_ - 1);
    double energy = 0.0, smooth = 0.0;
    for (int k = 0; k < N_ - 1; ++k) {
        const double Tm = 0.5 * (z[idxControl(k, THRUST)] + z[idxControl(k + 1, THRUST)]);
        const double Vm = 0.5 * (z[idxState(k, V)] + z[idxState(k + 1, V)]);
        energy += Tm * Vm * dt;
        for (int j = 0; j < NU; ++j) {
            const double du = (z[idxControl(k + 1, j)] - z[idxControl(k, j)]) / scales_.u[j];
            smooth += du * du;
        }
    }
    return (1.0 - a) * tf / scales_.tf + a * energy / energyRef_ + mission_.betaControl * smooth / (N_ - 1);
}

Vec TrapezoidalUF::costGradient(const Vec& z) const {
    const double a = mission_.alpha, tf = z[idxTf()];
    const double dt = tf / (N_ - 1);
    const double wReg = mission_.betaControl / (N_ - 1);
    Vec g(numVariables(), 0.0);

    double sumTV = 0.0;
    for (int k = 0; k < N_ - 1; ++k) {
        const double Tm = 0.5 * (z[idxControl(k, THRUST)] + z[idxControl(k + 1, THRUST)]);
        const double Vm = 0.5 * (z[idxState(k, V)] + z[idxState(k + 1, V)]);
        sumTV += Tm * Vm;
        // Each interval contributes to both of its end nodes.
        const double cE = a * dt / energyRef_ * 0.5;
        g[idxControl(k, THRUST)] += cE * Vm;
        g[idxControl(k + 1, THRUST)] += cE * Vm;
        g[idxState(k, V)] += cE * Tm;
        g[idxState(k + 1, V)] += cE * Tm;
        for (int j = 0; j < NU; ++j) {
            const double s = scales_.u[j];
            const double d = 2.0 * wReg * (z[idxControl(k + 1, j)] - z[idxControl(k, j)]) / (s * s);
            g[idxControl(k + 1, j)] += d;
            g[idxControl(k, j)] -= d;
        }
    }
    g[idxTf()] = (1.0 - a) / scales_.tf + a * sumTV / ((N_ - 1) * energyRef_);
    return g;
}

// ── Initial guesses ──────────────────────────────────────────────────────

namespace {

struct Line {
    Vec3 delta;
    double horizontal, length, bearing, gamma;
};

Line lineOf(const Mission& m) {
    Line l;
    l.delta = m.end - m.start;
    l.horizontal = std::hypot(l.delta[0], l.delta[1]);
    l.length = norm(l.delta);
    l.bearing = std::atan2(l.delta[1], l.delta[0]);
    l.gamma = std::atan2(-l.delta[2], l.horizontal);
    return l;
}

} // namespace

Vec coldStartGuess(const TrapezoidalUF& nlp, const Aircraft& ac, const Mission& mission) {
    const int N = nlp.nodes();
    const Line l = lineOf(mission);
    const double V0 = speedStart(ac, mission);
    std::vector<Vec> X(N, Vec(NX, 0.0)), U(N, Vec(NU, 0.0));
    for (int k = 0; k < N; ++k) {
        const double s = static_cast<double>(k) / (N - 1);
        for (int i = 0; i < 3; ++i) X[k][i] = mission.start[i] + s * l.delta[i];
        X[k][V] = V0;
        X[k][CHI] = l.bearing;
        X[k][MASS] = ac.mass.m0;
    }
    return nlp.pack(l.length / V0, X, U);
}

Vec trimmedLineGuess(const TrapezoidalUF& nlp, const Aircraft& ac, const Mission& mission) {
    const int N = nlp.nodes();
    const Line l = lineOf(mission);
    const double V0 = speedStart(ac, mission);
    const double tf = l.length / V0, dt = tf / (N - 1);
    const double cf = fuelFlowCoefficient(ac);

    std::vector<Vec> X(N, Vec(NX, 0.0)), U(N, Vec(NU, 0.0));
    double m = ac.mass.m0;
    for (int k = 0; k < N; ++k) {
        const double s = static_cast<double>(k) / (N - 1);
        for (int i = 0; i < 3; ++i) X[k][i] = mission.start[i] + s * l.delta[i];
        const double alt = -X[k][Z];
        if (k > 0) {
            // Trapezoidal mass update; the trim thrust depends on the new mass,
            // so iterate the implicit relation a few times.
            const double Tprev = U[k - 1][THRUST];
            double mk = m;
            for (int it = 0; it < 5; ++it) {
                const double Tk = trim::pointMass(ac, {V0, alt, l.gamma, 0.0, mk}).u[THRUST];
                mk = m - 0.5 * dt * cf * V0 * (Tprev + Tk);
            }
            m = mk;
        }
        const trim::UfTrim t = trim::pointMass(ac, {V0, alt, l.gamma, 0.0, m});
        X[k][V] = V0;
        X[k][GAMMA] = l.gamma;
        X[k][CHI] = l.bearing;
        X[k][MASS] = m;
        U[k] = t.u;
    }
    return nlp.pack(tf, X, U);
}

} // namespace ofc
