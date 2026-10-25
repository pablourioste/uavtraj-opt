// Numerics, trim and the trapezoidal transcription.

#include <cmath>

#include "check.hpp"
#include "ofc/aircraft.hpp"
#include "ofc/numerics.hpp"
#include "ofc/point_mass.hpp"
#include "ofc/rigid_body.hpp"
#include "ofc/transcription.hpp"
#include "ofc/trim.hpp"

using namespace ofc;

namespace {
const Aircraft& generic() {
    static const Aircraft ac = Aircraft::fromFile("data/aircraft/generic_uav.json");
    return ac;
}

Mission climbingLeg() {
    Mission m;
    m.start = {0.0, 0.0, -300.0};
    m.end = {2500.0, 600.0, -380.0};
    m.headingStart = m.headingEnd = std::atan2(600.0, 2500.0);
    m.gammaStart = std::atan2(80.0, std::hypot(2500.0, 600.0)); // start already on the climb line
    m.speedStart = 25.0;
    m.alpha = 0.7;
    m.betaControl = 0.05;
    m.betaState = 0.05;
    return m;
}
} // namespace

TEST(dense_solver_and_newton) {
    const Vec x = num::solveDense({4, 1, 0, 1, 3, 1, 0, 1, 2}, {1, 2, 3});
    CHECK_NEAR(4 * x[0] + x[1], 1.0, 1e-12);
    CHECK_NEAR(x[0] + 3 * x[1] + x[2], 2.0, 1e-12);
    const auto r = num::newton([](const Vec& v) { return Vec{v[0] * v[0] - 2.0, v[1] - v[0]}; }, {1.0, 0.0});
    CHECK(r.converged);
    CHECK_NEAR(r.x[0], std::sqrt(2.0), 1e-10);
}

TEST(rk4_is_fourth_order) {
    const num::OdeFn f = [](double, const Vec& x) { return Vec{-x[0]}; };
    auto err = [&](double dt) { return std::fabs(num::simulate(f, {1.0}, 2.0, dt).x.back()[0] - std::exp(-2.0)); };
    const double ratio = err(0.1) / err(0.05);
    CHECK(ratio > 14.0 && ratio < 18.0);
}

TEST(uf_trim_is_an_equilibrium) {
    const Aircraft& ac = generic();
    for (double gamma : {0.0, 0.05}) {
        const trim::UfTrim t = trim::pointMass(ac, {25.0, 300.0, gamma, 0.0, 0.0});
        const Vec f = uf::rhs(ac, t.x, t.u);
        CHECK_NEAR(f[uf::V], 0.0, 1e-12);
        CHECK_NEAR(f[uf::GAMMA], 0.0, 1e-12);
        CHECK(t.feasible);
    }
}

TEST(fc_trim_converges_and_matches_uf) {
    const Aircraft& ac = generic();
    const trim::FcTrim t = trim::rigidBody(ac, {25.0, 300.0, 0.0, 0.0, 0.0});
    CHECK(t.converged);
    const Vec f = fc::rhs(ac, t.x, t.u);
    for (int i : {fc::U, fc::V, fc::W, fc::P, fc::Q, fc::R}) CHECK_NEAR(f[i], 0.0, 1e-8);
    CHECK_NEAR(f[fc::ZD], 0.0, 1e-9); // level flight
    CHECK(t.alpha > ac.envelope.alpha_min && t.alpha < ac.envelope.alpha_max);
    CHECK(std::fabs(t.u[fc::ELEVATOR]) < ac.actuators.de_max);
    // Same lift and drag model: the 6-DoF trim thrust is within 1 % of the point-mass one.
    const trim::UfTrim tu = trim::pointMass(ac, {25.0, 300.0, 0.0, 0.0, 0.0});
    CHECK(std::fabs(t.u[fc::THRUST] / tu.u[uf::THRUST] - 1.0) < 0.01);
}

TEST(nlp_layout_and_sparsity) {
    const Aircraft& ac = generic();
    const Mission ms = climbingLeg();
    const TrapezoidalUF nlp(ac, ms, 30, UfScales::fromAircraft(ac, 2600.0));
    CHECK(nlp.numVariables() == 1 + 30 * (uf::NX + uf::NU));
    CHECK(nlp.numDefects() == 29 * uf::NX);
    CHECK(nlp.idxControl(29, uf::NU - 1) == nlp.numVariables() - 1);
    const auto J = nlp.defectJacobian(trimmedLineGuess(nlp, ac, ms));
    const double density = double(J.size()) / (double(nlp.numDefects()) * nlp.numVariables());
    CHECK(density < 0.1); // block-banded, not dense
}

TEST(defect_jacobian_matches_fd) {
    const Aircraft& ac = generic();
    Mission ms = climbingLeg();
    ms.wind = {2.0, -1.0, 0.0};
    const TrapezoidalUF nlp(ac, ms, 7, UfScales::fromAircraft(ac, 2600.0));
    Vec z = trimmedLineGuess(nlp, ac, ms);
    for (int k = 0; k < nlp.nodes(); ++k) { // move away from the symmetric trim point
        z[nlp.idxState(k, uf::MU)] = 0.1 * std::sin(1.0 + k);
        z[nlp.idxControl(k, uf::BANK_RATE)] = 0.05 * std::cos(2.0 * k);
        z[nlp.idxControl(k, uf::CL)] *= 1.0 + 0.05 * std::sin(3.0 * k);
    }
    const Vec Jfd = num::fdJacobian([&](const Vec& v) { return nlp.defects(v); }, z);
    Vec Jan(Jfd.size(), 0.0);
    for (const Triplet& t : nlp.defectJacobian(z)) Jan[t.row * nlp.numVariables() + t.col] += t.value;
    CHECK(num::maxRelativeError(Jan, Jfd) < 1e-6);
}

TEST(cost_gradient_matches_fd) {
    const Aircraft& ac = generic();
    const Mission ms = climbingLeg();
    const TrapezoidalUF nlp(ac, ms, 9, UfScales::fromAircraft(ac, 2600.0));
    Vec z = trimmedLineGuess(nlp, ac, ms);
    for (int k = 0; k < nlp.nodes(); ++k) z[nlp.idxControl(k, uf::THRUST)] *= 1.0 + 0.1 * std::sin(k);
    const Vec gFd = num::fdJacobian([&](const Vec& v) { return Vec{nlp.cost(v)}; }, z);
    CHECK(num::maxRelativeError(nlp.costGradient(z), gFd) < 1e-7);
}

TEST(trimmed_guess_beats_cold_start) {
    const Aircraft& ac = generic();
    const Mission ms = climbingLeg();
    const TrapezoidalUF nlp(ac, ms, 40, UfScales::fromAircraft(ac, 2600.0));
    const double dCold = maxAbs(nlp.defects(coldStartGuess(nlp, ac, ms)));
    const Vec zTrim = trimmedLineGuess(nlp, ac, ms);
    const double dTrim = maxAbs(nlp.defects(zTrim));
    CHECK(dCold > 1e-3);
    CHECK(dTrim < 1e-9);
    CHECK(maxAbs(nlp.boundaryResiduals(zTrim)) < 1e-9);
    for (const auto& pc : nlp.pathSummary(zTrim)) CHECK(pc.worst <= 0.0);
}

TEST(trapezoidal_defect_is_third_order_locally) {
    // Sample a smooth reference trajectory (fine RK4) at the collocation nodes:
    // the trapezoidal defect per interval must shrink like dt^3.
    const Aircraft& ac = generic();
    const trim::UfTrim t = trim::pointMass(ac, {25.0, 300.0, 0.0, 0.0, 0.0});
    auto control = [&](double time) { return Vec{t.u[0], t.u[1], 0.08 * std::sin(0.3 * time)}; };
    const double T = 16.0, h = 1e-3;
    const num::Trajectory ref =
        num::simulate([&](double time, const Vec& x) { return uf::rhs(ac, x, control(time)); }, t.x, T, h);

    Mission ms;
    auto maxDefect = [&](int N) {
        const TrapezoidalUF nlp(ac, ms, N, UfScales::fromAircraft(ac, 400.0));
        std::vector<Vec> X(N), U(N);
        for (int k = 0; k < N; ++k) {
            const double tk = T * k / (N - 1);
            X[k] = ref.x[static_cast<std::size_t>(std::lround(tk / h))];
            U[k] = control(tk);
        }
        return maxAbs(nlp.defects(nlp.pack(T, X, U)));
    };
    const double ratio = maxDefect(17) / maxDefect(33);
    CHECK(ratio > 6.5 && ratio < 9.5);
}
