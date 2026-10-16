// Point performance and closed-loop design scenarios.

#include <cmath>

#include "check.hpp"
#include "ofc/aircraft.hpp"
#include "ofc/atmosphere.hpp"
#include "ofc/performance.hpp"
#include "ofc/scenario.hpp"

using namespace ofc;

namespace {
const Aircraft& generic() {
    static const Aircraft ac = Aircraft::fromFile("data/aircraft/generic_uav.json");
    return ac;
}

scenario::Scenario make(double climb, double turnDeg) {
    scenario::Scenario sc;
    sc.id = "test";
    sc.h0 = 300.0;
    sc.climb = climb;
    sc.turn = deg2rad(turnDeg);
    sc.turnStart = 600.0;
    sc.distance = 2000.0;
    sc.V = 25.0;
    sc.bankMax = deg2rad(30.0);
    sc.gammaMax = deg2rad(7.5);
    return sc;
}
} // namespace

TEST(performance_points_are_consistent) {
    const Aircraft& ac = generic();
    const double m = ac.mass.m0;
    // Max level speed: power available equals power required (or V_NE caps it).
    const double Vmax = perf::maxLevelSpeed(ac, 0.0, m);
    CHECK(Vmax > ac.envelope.V_cruise && Vmax <= ac.envelope.V_ne);
    if (Vmax < ac.envelope.V_ne)
        CHECK(std::fabs(perf::powerRequired(ac, Vmax, 0.0, m) / perf::powerAvailable(ac, 0.0) - 1.0) < 0.01);
    // Minimum-power speed lies below the best-L/D speed (classic ratio 3^(-1/4) ~ 0.76).
    const perf::SpeedPoint minP = perf::minPowerSpeed(ac, 0.0, m);
    CHECK_NEAR(minP.V / ac.bestGlideSpeed(0.0, m), std::pow(3.0, -0.25), 0.01);
    // Service ceiling: the best climb rate there is 0.5 m/s.
    const double hc = perf::ceiling(ac, m);
    CHECK_NEAR(perf::maxRateOfClimb(ac, hc, m).value, 0.5, 1e-3);
    // Level turn kinematics.
    const perf::Turn t = perf::levelTurn(ac, 25.0, 300.0, m, deg2rad(30.0));
    CHECK_NEAR(t.radius * t.rate, 25.0, 1e-9);
    CHECK_NEAR(t.n, 1.0 / std::cos(deg2rad(30.0)), 1e-12);
}

TEST(scenario_turn_is_completed_by_both_models) {
    const Aircraft& ac = generic();
    const scenario::Scenario sc = make(0.0, 180.0);
    for (scenario::Model model : {scenario::Model::PointMass, scenario::Model::RigidBody}) {
        const scenario::Run r = scenario::fly(ac, sc, model, 0.01);
        CHECK(r.completed);
        const scenario::Sample end = scenario::sampleOf(r, r.x.size() - 1);
        CHECK(std::fabs(std::remainder(end.chi - kPi, 2.0 * kPi)) < deg2rad(2.0)); // heading reversed
        CHECK(std::fabs(-end.z - sc.h0) < 5.0);                                    // altitude held
        CHECK(r.fuel > 0.0);
    }
}

TEST(scenario_climb_reaches_target_altitude) {
    const Aircraft& ac = generic();
    const scenario::Scenario sc = make(150.0, 0.0);
    const scenario::Run uf = scenario::fly(ac, sc, scenario::Model::PointMass, 0.01);
    const scenario::Run fc = scenario::fly(ac, sc, scenario::Model::RigidBody, 0.01);
    CHECK(std::fabs(-scenario::sampleOf(uf, uf.x.size() - 1).z - 450.0) < 1.0);
    CHECK(std::fabs(-scenario::sampleOf(fc, fc.x.size() - 1).z - 450.0) < 1.0);
    // Climbing costs more fuel than the level leg of the same length.
    const scenario::Run level = scenario::fly(ac, make(0.0, 0.0), scenario::Model::PointMass, 0.01);
    CHECK(uf.fuel > level.fuel);
}
