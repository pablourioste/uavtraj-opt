#pragma once
// Mission environment: no-fly zones (NFZ) and waypoints, in local NED metres.
//
// An NFZ is a simple polygon in the horizontal plane (vertices as [north, east]),
// extruded between h_min and h_max. It is active for a flight segment only if
// that altitude band overlaps the segment's altitudes, so a low zone can be
// overflown. Each zone carries a safety margin that the path must keep.

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace ofc::env {

struct Vec2 {
    double x = 0.0, y = 0.0; // north, east [m]
};

struct NoFlyZone {
    std::string id;
    std::vector<Vec2> polygon;     // simple polygon, any orientation
    double hMin = 0.0, hMax = 1e9; // altitude band [m]
    double margin = 0.0;           // required clearance [m]

    bool activeBetween(double h0, double h1) const; // band overlaps [min(h0,h1), max(h0,h1)]
    bool contains(double x, double y) const;
    // Distance to the boundary: positive outside, negative inside [m].
    double signedDistance(double x, double y) const;
    // Signed distance minus the margin: >= 0 means the point is acceptable.
    double clearance(double x, double y) const { return signedDistance(x, y) - margin; }
};

struct Waypoint {
    std::string id;
    Vec2 position;
    double tolerance = 50.0; // the trajectory must pass within this horizontal distance [m]
};

NoFlyZone nfzFromJson(const nlohmann::json& j);
Waypoint waypointFromJson(const nlohmann::json& j);

// Smallest clearance over the zones active at altitude h (+inf if none).
double minClearance(const std::vector<NoFlyZone>& zones, double x, double y, double h);

} // namespace ofc::env
