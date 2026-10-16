#pragma once
// Point performance from the aircraft data: power curves, flight envelope,
// climb and turn performance. All powers are *thrust* powers (T V), so the
// available power already includes the propeller efficiency.

#include "ofc/aircraft.hpp"

namespace ofc::perf {

// Thrust power needed for steady level flight at load factor n [W].
double powerRequired(const Aircraft& ac, double V, double h, double m, double n = 1.0);

// eta_p * P_max * sigma(h) [W].
double powerAvailable(const Aircraft& ac, double h);

// Steady climb rate (P_av - P_req) / W at airspeed V [m/s] (negative = cannot hold altitude).
double rateOfClimb(const Aircraft& ac, double V, double h, double m);

struct SpeedPoint {
    double V = 0.0, value = 0.0;
};

// Maximum rate of climb over [V_stall, V_ne] and the speed where it occurs.
SpeedPoint maxRateOfClimb(const Aircraft& ac, double h, double m);

// Minimum power-required speed over [V_stall, V_ne] (best endurance).
SpeedPoint minPowerSpeed(const Aircraft& ac, double h, double m);

// Highest speed with P_av >= P_req, capped at V_ne; 0 if level flight is impossible.
double maxLevelSpeed(const Aircraft& ac, double h, double m);

// Altitude where the best climb rate drops to rocLimit (0.5 m/s: service ceiling).
double ceiling(const Aircraft& ac, double m, double rocLimit = 0.5);

// Level coordinated turn at bank phi: load factor, radius [m], rate [rad/s].
struct Turn {
    double n = 0.0, radius = 0.0, rate = 0.0, powerRequired = 0.0;
    bool feasible = true; // CL, load factor and power within limits
};
Turn levelTurn(const Aircraft& ac, double V, double h, double m, double phi);

} // namespace ofc::perf
