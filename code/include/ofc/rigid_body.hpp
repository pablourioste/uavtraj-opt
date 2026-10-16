#pragma once
// Full-control formulation (FC): 6-DoF rigid body in body axes.
//
//   x = [x_N, y_E, z_D, u, v, w, phi, theta, psi, p, q, r, m]
//   u = [T, delta_e, delta_a, delta_r]
// (u, v, w) is the inertial velocity in body axes; the aerodynamic angles are
// computed from the air-relative velocity after removing the NED wind.

#include "ofc/aircraft.hpp"
#include "ofc/types.hpp"

namespace ofc::fc {

enum State : int { XN = 0, YE, ZD, U, V, W, PHI, THETA, PSI, P, Q, R, MASS, NX };
enum Control : int { THRUST = 0, ELEVATOR, AILERON, RUDDER, NU };

struct AirData {
    double Va = 0.0, alpha = 0.0, beta = 0.0;
};

struct AeroCoeffs {
    double CL = 0.0, CD = 0.0, CY = 0.0, Cl = 0.0, Cm = 0.0, Cn = 0.0;
};

// Rotation body -> NED for 3-2-1 Euler angles.
Mat3 bodyToNed(double phi, double theta, double psi);

AirData airData(const Vec& x, const Vec3& windNed);

AeroCoeffs coefficients(const Aircraft& ac, const AirData& air, const Vec& x, const Vec& u);

Vec rhs(const Aircraft& ac, const Vec& x, const Vec& u, const Vec3& windNed = {0.0, 0.0, 0.0});

} // namespace ofc::fc
