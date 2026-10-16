// No-fly zones, Dubins paths, Hybrid A* and path-following missions.

#include <cmath>

#include "check.hpp"
#include "ofc/aircraft.hpp"
#include "ofc/environment.hpp"
#include "ofc/planner.hpp"
#include "ofc/scenario.hpp"
#include "ofc/transcription.hpp"

using namespace ofc;

namespace {
const Aircraft& generic() {
    static const Aircraft ac = Aircraft::fromFile("data/aircraft/generic_uav.json");
    return ac;
}

env::NoFlyZone square(double x0, double y0, double x1, double y1, double margin, double hMax = 1e9) {
    env::NoFlyZone z;
    z.id = "Z";
    z.polygon = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
    z.margin = margin;
    z.hMax = hMax;
    return z;
}

double headingError(double a, double b) { return std::fabs(std::remainder(a - b, 2.0 * kPi)); }
} // namespace

TEST(nfz_signed_distance_and_altitude_band) {
    const env::NoFlyZone z = square(0, 0, 100, 100, 10, 200);
    CHECK_NEAR(z.signedDistance(50, 50), -50.0, 1e-12); // centre: inside
    CHECK_NEAR(z.signedDistance(150, 50), 50.0, 1e-12); // beside an edge
    CHECK_NEAR(z.signedDistance(103, 104), 5.0, 1e-12); // near a corner
    CHECK_NEAR(z.clearance(150, 50), 40.0, 1e-12);
    CHECK(z.activeBetween(150, 400));  // band overlaps
    CHECK(!z.activeBetween(250, 400)); // flying over the zone
}

TEST(dubins_words_reach_the_goal) {
    const double R = 140.0;
    const plan::Pose2 cases[][2] = {{{0, 0, 0}, {1000, 0, 0}},
                                    {{0, 0, 0}, {300, 200, kPi}},
                                    {{100, -50, 1.0}, {-400, 600, -2.0}},
                                    {{0, 0, 0.3}, {50, 40, 2.5}}};
    for (const auto& c : cases) {
        const auto all = plan::dubinsAll(c[0], c[1], R);
        CHECK(!all.empty());
        for (const plan::DubinsPath& d : all) {
            const plan::Path p = plan::sampleDubins(d, 2.0);
            CHECK(std::hypot(p.back().x - c[1].x, p.back().y - c[1].y) < 1e-6);
            CHECK(headingError(p.back().psi, c[1].psi) < 1e-9);
        }
        CHECK(plan::dubins(c[0], c[1], R).length >= std::hypot(c[1].x - c[0].x, c[1].y - c[0].y) - 1e-9);
    }
    CHECK_NEAR(plan::dubins({0, 0, 0}, {1000, 0, 0}, R).length, 1000.0, 1e-9); // straight ahead
}

TEST(hybrid_astar_goes_around_a_wall) {
    const env::NoFlyZone wall = square(600, -400, 800, 400, 30);
    const plan::PlanResult r =
        plan::planMission({0, 0, 0}, {}, {1500, 0, 0}, 140.0, {wall}, 300.0, 300.0, 5.0, plan::PlannerConfig{});
    CHECK(r.found);
    CHECK(r.length > 1500.0);
    for (const plan::Pose2& p : r.path) CHECK(wall.clearance(p.x, p.y) >= 5.0 - 1e-9);
    CHECK(std::hypot(r.path.back().x - 1500.0, r.path.back().y) < 1e-6);
    // A zone below the flight altitude does not block the straight route.
    const env::NoFlyZone low = square(600, -400, 800, 400, 30, 200.0);
    const plan::PlanResult s =
        plan::planMission({0, 0, 0}, {}, {1500, 0, 0}, 140.0, {low}, 300.0, 300.0, 5.0, plan::PlannerConfig{});
    CHECK(s.found);
    CHECK_NEAR(s.length, 1500.0, 1.0);
}

TEST(planned_mission_is_flown_clear_of_the_zones) {
    const Aircraft& ac = generic();
    const env::NoFlyZone zone = square(700, -300, 1100, 300, 30);
    env::Waypoint wp;
    wp.position = {1700, 400};
    scenario::Scenario sc;
    sc.id = "T";
    sc.V = 25.0;
    sc.bankMax = deg2rad(30.0);
    sc.gammaMax = deg2rad(7.5);
    const double R = sc.V * sc.V / (9.80665 * std::tan(0.8 * sc.bankMax));
    const plan::PlanResult pr = plan::planMission({0, 0, 0}, {wp}, {2400, 0, 0}, R, {zone}, 300, 300, 10.0);
    CHECK(pr.found);
    sc.path = pr.path;
    for (scenario::Model m : {scenario::Model::PointMass, scenario::Model::RigidBody}) {
        const scenario::Run r = scenario::fly(ac, sc, m, 0.01);
        CHECK(r.completed);
        double clearance = 1e9, miss = 1e9;
        for (std::size_t k = 0; k < r.x.size(); ++k) {
            const scenario::Sample s = scenario::sampleOf(r, k);
            clearance = std::min(clearance, zone.clearance(s.x, s.y));
            miss = std::min(miss, std::hypot(s.x - wp.position.x, s.y - wp.position.y));
        }
        CHECK(clearance > 0.0);
        CHECK(miss < 10.0);
    }
}

TEST(nlp_nfz_rows) {
    const Aircraft& ac = generic();
    Mission ms;
    ms.start = {0, 0, -300};
    ms.end = {2000, 0, -300};
    ms.speedStart = 25.0;
    ms.nfz = {square(900, -200, 1100, 200, 20), square(900, -200, 1100, 200, 20, 100.0)};
    ms.nfz[1].id = "LOW";
    const TrapezoidalUF nlp(ac, ms, 21, UfScales::fromAircraft(ac, 2000.0));
    const auto names = nlp.pathConstraintNames();
    CHECK(names[names.size() - 2] == "nfz:Z" && names.back() == "nfz:LOW");
    double active = -1e9, low = -1e9;
    for (const auto& pc : nlp.pathSummary(coldStartGuess(nlp, ac, ms))) {
        if (pc.name == "nfz:Z") active = pc.worst;
        if (pc.name == "nfz:LOW") low = pc.worst;
    }
    CHECK(active > 0.0);          // the straight line crosses the zone
    CHECK_NEAR(low, -1.0, 1e-12); // a zone below the flight is inactive
}
