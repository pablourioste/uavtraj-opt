#include "ofc/mission.hpp"

#include <cmath>
#include <stdexcept>

#include "ofc/types.hpp"

namespace ofc {

double MissionSpec::referenceLength() const {
    double L = 0.0, x = flight.x0, y = flight.y0;
    for (const env::Waypoint& w : waypoints) {
        L += std::hypot(w.position.x - x, w.position.y - y);
        x = w.position.x, y = w.position.y;
    }
    L += std::hypot(goal.x - x, goal.y - y);
    return std::hypot(L, hEnd() - hStart());
}

Mission MissionSpec::ufMission(double alpha, double betaControl, double betaState) const {
    Mission m;
    m.start = {flight.x0, flight.y0, -hStart()};
    m.end = {goal.x, goal.y, -hEnd()};
    m.headingStart = flight.heading0;
    m.headingEnd = goal.psi;
    m.speedStart = flight.V;
    m.gammaStart = 0.0;
    m.alpha = alpha;
    m.betaControl = betaControl;
    m.betaState = betaState;
    m.nfz = zones;
    return m;
}

MissionSpec MissionSpec::fromJson(const nlohmann::json& j, const Aircraft& ac) {
    MissionSpec s;
    s.flight = scenario::Scenario::fromJson(j, ac);
    s.id = s.flight.id;
    s.title = s.flight.title;
    if (!j.contains("end")) throw std::invalid_argument("mission " + s.id + ": 'end' [north, east] is required");
    s.goal = {j.at("end").at(0).get<double>(), j.at("end").at(1).get<double>(),
              deg2rad(j.value("end_heading_deg", 0.0))};
    for (const nlohmann::json& z : j.value("nfz", nlohmann::json::array())) s.zones.push_back(env::nfzFromJson(z));
    for (const nlohmann::json& w : j.value("waypoints", nlohmann::json::array()))
        s.waypoints.push_back(env::waypointFromJson(w));
    return s;
}

} // namespace ofc
