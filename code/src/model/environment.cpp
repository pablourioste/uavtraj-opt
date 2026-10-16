#include "ofc/environment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ofc::env {

namespace {

double segmentDistance(const Vec2& p, const Vec2& a, const Vec2& b) {
    const double vx = b.x - a.x, vy = b.y - a.y;
    const double len2 = vx * vx + vy * vy;
    double t = len2 > 0.0 ? ((p.x - a.x) * vx + (p.y - a.y) * vy) / len2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    return std::hypot(p.x - (a.x + t * vx), p.y - (a.y + t * vy));
}

Vec2 vec2(const nlohmann::json& a) {
    if (!a.is_array() || a.size() != 2) throw std::invalid_argument("expected [north, east]");
    return {a[0].get<double>(), a[1].get<double>()};
}

} // namespace

bool NoFlyZone::activeBetween(double h0, double h1) const {
    return std::max(h0, h1) >= hMin && std::min(h0, h1) <= hMax;
}

bool NoFlyZone::contains(double x, double y) const {
    // Crossing-number test.
    bool inside = false;
    const std::size_t n = polygon.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const Vec2 &a = polygon[i], &b = polygon[j];
        if ((a.y > y) != (b.y > y) && x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x) inside = !inside;
    }
    return inside;
}

double NoFlyZone::signedDistance(double x, double y) const {
    double d = std::numeric_limits<double>::infinity();
    const std::size_t n = polygon.size();
    for (std::size_t i = 0; i < n; ++i) d = std::min(d, segmentDistance({x, y}, polygon[i], polygon[(i + 1) % n]));
    return contains(x, y) ? -d : d;
}

NoFlyZone nfzFromJson(const nlohmann::json& j) {
    NoFlyZone z;
    z.id = j.value("id", std::string("NFZ"));
    for (const auto& v : j.at("polygon")) z.polygon.push_back(vec2(v));
    if (z.polygon.size() < 3) throw std::invalid_argument("NFZ " + z.id + ": polygon needs at least 3 vertices");
    z.hMin = j.value("h_min_m", 0.0);
    z.hMax = j.value("h_max_m", 1e9);
    z.margin = j.value("margin_m", 0.0);
    if (z.hMax <= z.hMin) throw std::invalid_argument("NFZ " + z.id + ": h_max_m must be > h_min_m");
    return z;
}

Waypoint waypointFromJson(const nlohmann::json& j) {
    Waypoint w;
    w.id = j.value("id", std::string("WP"));
    w.position = vec2(j.at("position"));
    w.tolerance = j.value("tolerance_m", 50.0);
    return w;
}

double minClearance(const std::vector<NoFlyZone>& zones, double x, double y, double h) {
    double c = std::numeric_limits<double>::infinity();
    for (const NoFlyZone& z : zones)
        if (z.activeBetween(h, h)) c = std::min(c, z.clearance(x, y));
    return c;
}

} // namespace ofc::env
