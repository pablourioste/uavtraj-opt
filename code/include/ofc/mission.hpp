#pragma once
// A mission as read from a case file: start and goal poses, altitude change,
// commanded speed and bank, no-fly zones and waypoints. The same description
// feeds the planner, the guidance laws and both optimal-control problems.
//
// JSON keys (local metres, [north, east]; angles in degrees):
//   id, title, h0_m, climb_m, V_mps, bank_deg, gamma_deg,
//   start [N, E], start_heading_deg, end [N, E] (required), end_heading_deg,
//   nfz [ {id, polygon, h_min_m, h_max_m, margin_m} ],
//   waypoints [ {id, position, tolerance_m} ]

#include <algorithm>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ofc/aircraft.hpp"
#include "ofc/environment.hpp"
#include "ofc/planner.hpp"
#include "ofc/scenario.hpp"
#include "ofc/transcription.hpp"

namespace ofc {

struct MissionSpec {
    std::string id, title;
    scenario::Scenario flight; // start pose, initial altitude, climb, commanded speed, bank and climb angle
    plan::Pose2 goal;          // final position [N, E] and heading
    std::vector<env::NoFlyZone> zones;
    std::vector<env::Waypoint> waypoints;

    double hStart() const { return flight.h0; }
    double hEnd() const { return flight.h0 + flight.climb; }
    double hLow() const { return std::min(hStart(), hEnd()); }
    double hHigh() const { return std::max(hStart(), hEnd()); }

    // Length of the polyline start -> waypoints -> goal [m]: a lower bound of
    // any feasible path, and the mission's characteristic length.
    double referenceLength() const;

    // Point-mass problem data: start and end states, zones, cost weights.
    Mission ufMission(double alpha, double betaControl, double betaState = 0.0) const;

    static MissionSpec fromJson(const nlohmann::json& j, const Aircraft& ac);
};

} // namespace ofc
