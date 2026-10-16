#pragma once
// Uncoupled formulation (UF): 3-DoF point mass in wind axes.
//
//   x = [x_N, y_E, z_D, V, gamma, chi, mu, m]     (NED position, airspeed,
//   u = [T, CL, p_mu]                               flight-path angle, heading,
//                                                   bank angle, mass)
// The lift coefficient is a direct control (time-scale separation): the inner
// attitude loop is assumed to track the requested lift and bank instantly.
// Wind is a constant NED vector and enters only the position kinematics.

#include "ofc/aircraft.hpp"
#include "ofc/types.hpp"

namespace ofc::uf {

enum State : int { X = 0, Y, Z, V, GAMMA, CHI, MU, MASS, NX };
enum Control : int { THRUST = 0, CL, BANK_RATE, NU };

struct Forces {
    double L = 0.0, D = 0.0, rho = 0.0, qbar = 0.0;
};

Forces aerodynamicForces(const Aircraft& ac, const Vec& x, const Vec& u);

// Right-hand side x_dot = f(x, u).
Vec rhs(const Aircraft& ac, const Vec& x, const Vec& u, const Vec3& wind = {0.0, 0.0, 0.0});

// Analytic Jacobians, row-major: A = df/dx (NX x NX), B = df/du (NX x NU).
void jacobian(const Aircraft& ac, const Vec& x, const Vec& u, Vec& A, Vec& B);

double loadFactor(const Aircraft& ac, const Vec& x, const Vec& u);

// Fuel flow [kg/s] of a constant-power propeller: sfc * (T V / eta_p).
double fuelFlowCoefficient(const Aircraft& ac); // c such that m_dot = -c T V

} // namespace ofc::uf
