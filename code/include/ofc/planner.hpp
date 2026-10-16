#pragma once
// Geometric path planning in the horizontal plane (chapter 11 of the thesis).
//
//   * Dubins paths: shortest forward paths between two poses with a minimum
//     turn radius (six words: LSL, LSR, RSL, RSR, RLR, LRL).
//   * Hybrid A*: graph search over (x, y, psi) whose edges are arcs of radius
//     R_min or straight segments, so every path is flyable. The heuristic is
//     max(Euclidean distance, obstacle-free Dubins length), both admissible,
//     and a Dubins "analytic expansion" to the goal closes the search.
//
// Headings follow the NED convention (psi from north towards east), and a
// positive curvature increases psi.

#include <functional>
#include <vector>

#include "ofc/environment.hpp"

namespace ofc::plan {

struct Pose2 {
    double x = 0.0, y = 0.0, psi = 0.0;
};
using Path = std::vector<Pose2>;

double mod2pi(double a);

// Pose reached after travelling len along a constant-curvature arc.
Pose2 advance(const Pose2& p, double curvature, double len);

struct DubinsPath {
    bool valid = false;
    double length = 0.0; // [m]
    double seg[3] = {};  // segment lengths [m]
    int dir[3] = {};     // curvature sign of each segment (+1, 0, -1)
    Pose2 start;
    double radius = 0.0;
};

DubinsPath dubins(const Pose2& a, const Pose2& b, double radius);                 // shortest of the six words
std::vector<DubinsPath> dubinsAll(const Pose2& a, const Pose2& b, double radius); // every valid word
Path sampleDubins(const DubinsPath& d, double step);

struct PlannerConfig {
    double cell = 20.0;         // grid resolution [m]
    int headingBins = 72;       // heading resolution (5 deg)
    double step = 30.0;         // length of one motion primitive [m]
    double sampleStep = 5.0;    // collision-check and output spacing [m]
    double turnPenalty = 0.05;  // extra cost per unit of normalised curvature
    double switchPenalty = 2.0; // extra cost [m] when the steering changes
    int maxExpansions = 500000;
    int analyticEvery = 5; // try a Dubins shot to the goal every N expansions
};

struct Bounds {
    double xMin = 0.0, xMax = 0.0, yMin = 0.0, yMax = 0.0;
};

using FreeFn = std::function<bool(double x, double y)>;

struct PlanResult {
    Path path;
    bool found = false;
    int expansions = 0;
    double length = 0.0; // [m]
    double millis = 0.0; // wall-clock time
};

PlanResult hybridAStar(const Pose2& start, const Pose2& goal, double radius, const Bounds& bounds, const FreeFn& isFree,
                       const PlannerConfig& cfg = {});

// Plan start -> waypoints -> goal, one Hybrid A* leg per segment. A waypoint's
// heading is the bearing towards the next target. Zones are checked over the
// altitude band [hLow, hHigh] of the flight, with an extra safety distance.
PlanResult planMission(const Pose2& start, const std::vector<env::Waypoint>& waypoints, const Pose2& goal,
                       double radius, const std::vector<env::NoFlyZone>& zones, double hLow, double hHigh,
                       double safety, const PlannerConfig& cfg = {});

} // namespace ofc::plan
