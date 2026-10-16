#pragma once
// Typical design scenarios flown in closed loop by both models.
//
// A scenario is a straight leg, an optional heading change and an optional
// altitude change, flown at a commanded airspeed until a given along-track
// distance. The same guidance (altitude -> flight-path angle, heading -> bank)
// drives
//   * the point mass (UF) through dynamic inversion of its equations, and
//   * the 6-DoF rigid body (FC) through a simple attitude/speed autopilot.
// The result is a *flyable reference trajectory*, not an optimum: the public
// edition does not include the optimiser. It is the kind of trajectory a warm
// start provides, and it can be evaluated in the NLP (see TrapezoidalUF).

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ofc/aircraft.hpp"
#include "ofc/planner.hpp"
#include "ofc/types.hpp"

namespace ofc::scenario {

struct Scenario {
    std::string id, title;
    double h0 = 300.0;         // initial altitude [m]
    double climb = 0.0;        // altitude change [m]
    double turn = 0.0;         // heading change [rad], + = right
    double turnStart = 1000.0; // along-track distance where the turn starts [m]
    double distance = 3000.0;  // along-track distance to fly [m]
    double V = 0.0;            // commanded airspeed [m/s]; <= 0 means V_cruise
    double bankMax = 0.0;      // bank used in turns [rad]; <= 0 means 3/4 of phi_max
    double gammaMax = 0.0;     // flight-path angle used in climbs [rad]; <= 0 means 1/2 of gamma_max
    double x0 = 0.0, y0 = 0.0; // start position, north / east [m]
    double heading0 = 0.0;     // start heading [rad]

    // Optional planned horizontal path. When set, lateral guidance follows it
    // with the L1 law and the run ends at the end of the path; otherwise the
    // heading-step law (turn / turnStart / distance) is used.
    plan::Path path;

    static Scenario fromJson(const nlohmann::json& j, const Aircraft& ac);
};

enum class Model { PointMass, RigidBody };

struct Run {
    Model model = Model::PointMass;
    Vec t;
    std::vector<Vec> x; // states of the model (uf::NX or fc::NX)
    std::vector<Vec> u; // controls applied on [t_k, t_k+1)
    Vec s;              // along-track distance [m] (progress along the path when one is followed)
    double flightTime = 0.0, fuel = 0.0;
    bool completed = false; // reached the distance within the time budget
};

Run fly(const Aircraft& ac, const Scenario& sc, Model model, double dt = 0.01);

// Common outputs regardless of the model: NED position, airspeed, flight-path
// angle, heading, bank and mass at sample k.
struct Sample {
    double x, y, z, V, gamma, chi, bank, m;
};
Sample sampleOf(const Run& r, std::size_t k);

} // namespace ofc::scenario
