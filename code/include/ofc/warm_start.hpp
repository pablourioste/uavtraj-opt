#pragma once
// Warm start: initial points for the optimal-control problems.
//
//   1. Hybrid A* plans a flyable horizontal path around the active no-fly
//      zones and through the waypoints (plan::planMission).
//   2. The guidance laws fly that path with both models (scenario::fly): the
//      point mass by dynamic inversion, the rigid body with its autopilot.
//      Each run is dynamically consistent, so its collocation defects are
//      small, unlike a straight-line guess.
//   3. The runs are resampled on the collocation nodes (fromRun).
//   4. ufToFc maps a point-mass trajectory (for example the UF optimum) to
//      rigid-body states and controls, to seed the 6-DoF problem.

#include <string>

#include <nlohmann/json.hpp>

#include "ofc/aircraft.hpp"
#include "ofc/collocation.hpp"
#include "ofc/mission.hpp"
#include "ofc/planner.hpp"
#include "ofc/scenario.hpp"

namespace ofc::ws {

struct Config {
    plan::PlannerConfig planner;
    double bankFraction = 0.8; // planning radius R = V^2 / (g tan(bankFraction * bank))
    double safety = 10.0;      // extra clearance added to every zone margin by the planner [m]

    static Config fromJson(const nlohmann::json& j); // keys: cell_m, heading_bins, step_m, bank_fraction, safety_m
};

struct Result {
    bool ok = false;     // path found and flown to the end by the point mass
    std::string message; //
    double radius = 0.0; // planning turn radius [m]
    plan::PlanResult plan;
    scenario::Run uf, fc; // the planned path flown by each model
    double plannerMillis = 0.0, guidanceMillis = 0.0;
};

Result build(const Aircraft& ac, const MissionSpec& m, const Config& cfg);

// States and controls of a flown run on N uniform nodes (time interpolation).
ocp::Trajectory fromRun(const scenario::Run& r, int N);

// Straight line from start to goal at constant speed with zero controls
// (the classic cold start).
ocp::Trajectory coldUf(const Aircraft& ac, const MissionSpec& m, int N);
// Straight line from start to goal, every node at the wings-level trim.
ocp::Trajectory coldFc(const Aircraft& ac, const MissionSpec& m, int N);

// Point-mass trajectory -> rigid-body states and controls:
//   attitude   R_nb = R_nw(mu, gamma, chi) R_wb(alpha), with alpha from C_L
//              and zero sideslip; Euler angles extracted from R_nb
//   rates      p, q, r from the time derivatives of the Euler angles
//   velocity   air-relative (V cos alpha, 0, V sin alpha) plus the wind
//   controls   thrust T / cos(alpha) (it acts along the body axis, not the
//              velocity); alpha and elevator from the lift and pitch-moment
//              balance; aileron and rudder from the roll and yaw balance at
//              zero sideslip
ocp::Trajectory ufToFc(const Aircraft& ac, const ocp::Trajectory& uf, const Vec3& wind = {0.0, 0.0, 0.0});

// Rigid-body state at the start of the mission: wings-level trim at the
// commanded speed and initial altitude, at the start position and heading.
Vec fcStartState(const Aircraft& ac, const MissionSpec& m);

} // namespace ofc::ws
