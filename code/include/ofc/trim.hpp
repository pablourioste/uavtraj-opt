#pragma once
// Steady-flight trim for both formulations.
//   UF: closed form (lift balances the weight component, thrust balances drag).
//   FC: Newton solve of the longitudinal force and moment balance for
//       (alpha, delta_e, T) in straight, wings-level flight.

#include "ofc/aircraft.hpp"
#include "ofc/types.hpp"

namespace ofc::trim {

struct FlightCondition {
    double V = 0.0;     // airspeed [m/s]
    double h = 0.0;     // altitude [m]
    double gamma = 0.0; // flight-path angle [rad]
    double mu = 0.0;    // bank angle [rad] (UF only; coordinated turn)
    double mass = 0.0;  // [kg]; <= 0 means take-off mass
};

struct UfTrim {
    Vec x, u; // state and control (uf::NX, uf::NU)
    double turnRate = 0.0;
    bool feasible = true; // false if CL, thrust or load factor exceed limits
};
UfTrim pointMass(const Aircraft& ac, const FlightCondition& fc);

struct FcTrim {
    Vec x, u; // state and control (fc::NX, fc::NU)
    double alpha = 0.0;
    double residual = 0.0;
    int iterations = 0;
    bool converged = false;
};
FcTrim rigidBody(const Aircraft& ac, const FlightCondition& fc);

} // namespace ofc::trim
