#include "ofc/trim.hpp"

#include <cmath>

#include "ofc/atmosphere.hpp"
#include "ofc/numerics.hpp"
#include "ofc/point_mass.hpp"
#include "ofc/rigid_body.hpp"

namespace ofc::trim {

namespace {
double massOf(const Aircraft& ac, const FlightCondition& fc) { return fc.mass > 0.0 ? fc.mass : ac.mass.m0; }
} // namespace

UfTrim pointMass(const Aircraft& ac, const FlightCondition& fc) {
    const double m = massOf(ac, fc), g = isa::g0;
    const double qS = 0.5 * isa::density(fc.h) * fc.V * fc.V * ac.geometry.S;

    UfTrim t;
    t.x.assign(uf::NX, 0.0);
    t.x[uf::Z] = -fc.h;
    t.x[uf::V] = fc.V;
    t.x[uf::GAMMA] = fc.gamma;
    t.x[uf::MU] = fc.mu;
    t.x[uf::MASS] = m;

    // gamma_dot = 0: L cos(mu) = m g cos(gamma).  V_dot = 0: T = D + m g sin(gamma).
    const double CL = m * g * std::cos(fc.gamma) / (qS * std::cos(fc.mu));
    const double D = qS * (ac.aero.CD0 + ac.aero.k * CL * CL);
    t.u = {D + m * g * std::sin(fc.gamma), CL, 0.0};
    t.turnRate = qS * CL * std::sin(fc.mu) / (m * fc.V * std::cos(fc.gamma));

    const double n = qS * CL / (m * g);
    t.feasible = CL <= ac.clMax() && t.u[uf::THRUST] <= ac.maxThrust(fc.V, fc.h) && n <= ac.envelope.n_max &&
                 std::fabs(fc.mu) <= ac.envelope.phi_max;
    return t;
}

FcTrim rigidBody(const Aircraft& ac, const FlightCondition& fc) {
    const double m = massOf(ac, fc);

    auto stateFor = [&](double alpha) {
        Vec x(fc::NX, 0.0);
        x[fc::ZD] = -fc.h;
        x[fc::U] = fc.V * std::cos(alpha);
        x[fc::W] = fc.V * std::sin(alpha);
        x[fc::THETA] = alpha + fc.gamma;
        x[fc::MASS] = m;
        return x;
    };

    // Unknowns y = [alpha, delta_e, T]; residuals = normalised (u_dot, w_dot, q_dot).
    const double accRef = isa::g0, qdotRef = 1.0;
    auto residual = [&](const Vec& y) {
        const Vec x = stateFor(y[0]);
        const Vec f = fc::rhs(ac, x, {y[2], y[1], 0.0, 0.0});
        return Vec{f[fc::U] / accRef, f[fc::W] / accRef, f[fc::Q] / qdotRef};
    };

    // Initial guess from the point-mass trim and the linear lift/moment model.
    const UfTrim uf0 = pointMass(ac, {fc.V, fc.h, fc.gamma, 0.0, m});
    const AeroCoefficients& k = ac.aero;
    const double det = k.CLa * k.Cmde - k.CLde * k.Cma;
    const double alpha0 = ((uf0.u[uf::CL] - k.CL0) * k.Cmde + k.CLde * k.Cm0) / det;
    const double de0 = (-k.Cm0 - k.Cma * alpha0) / k.Cmde;

    const num::NewtonResult nr = num::newton(residual, {alpha0, de0, uf0.u[uf::THRUST]}, 1e-11, 60);

    FcTrim t;
    t.alpha = nr.x[0];
    t.x = stateFor(t.alpha);
    t.u = {nr.x[2], nr.x[1], 0.0, 0.0};
    t.residual = nr.residual;
    t.iterations = nr.iterations;
    t.converged = nr.converged;
    return t;
}

} // namespace ofc::trim
