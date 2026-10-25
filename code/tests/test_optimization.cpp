// Optimal-control problems, warm start, solver and pipeline.

#include <cmath>

#include "check.hpp"
#include "ofc/aircraft.hpp"
#include "ofc/mission.hpp"
#include "ofc/numerics.hpp"
#include "ofc/ocp_fc.hpp"
#include "ofc/ocp_uf.hpp"
#include "ofc/pipeline.hpp"
#include "ofc/point_mass.hpp"
#include "ofc/rigid_body.hpp"
#include "ofc/solver.hpp"
#include "ofc/trim.hpp"
#include "ofc/warm_start.hpp"

using namespace ofc;

namespace {
const Aircraft& generic() {
    static const Aircraft ac = Aircraft::fromFile("data/aircraft/generic_uav.json");
    return ac;
}

// Level leg along north, with an optional zone to the east of the route and a waypoint.
MissionSpec leg(double length, bool zone, bool waypoint) {
    nlohmann::json j = {{"id", "T"}, {"h0_m", 300}, {"V_mps", 25}, {"bank_deg", 30}, {"end", {length, 0.0}}};
    if (zone)
        j["nfz"] = {
            {{"id", "Z"},
             {"polygon", {{0.4 * length, -150}, {0.6 * length, -150}, {0.6 * length, 400}, {0.4 * length, 400}}},
             {"margin_m", 40}}};
    if (waypoint) j["waypoints"] = {{{"id", "W"}, {"position", {0.5 * length, -300}}, {"tolerance_m", 40}}};
    return MissionSpec::fromJson(j, generic());
}

// Trimmed straight line with small wiggles, so that no derivative is evaluated at a symmetric point.
ocp::Trajectory wiggledUfLine(const MissionSpec& m, int N) {
    const Mission ms = m.ufMission(0.5, 0.01);
    const TrapezoidalUF tr(generic(), ms, N, UfScales::fromAircraft(generic(), 2000.0));
    ocp::Trajectory t = ocp::unpack(tr.layout(), trimmedLineGuess(tr, generic(), ms));
    for (int k = 0; k < N; ++k) {
        t.X[k][uf::Y] += 60.0 * std::sin(0.7 * k);
        t.X[k][uf::MU] = 0.1 * std::sin(1.0 + k);
        t.U[k][uf::BANK_RATE] = 0.05 * std::cos(2.0 * k);
    }
    return t;
}
} // namespace

TEST(collocation_pack_and_resample) {
    ocp::Trajectory t{10.0, {{0.0, 1.0}, {2.0, 3.0}, {4.0, 5.0}}, {{1.0}, {2.0}, {3.0}}};
    const ocp::Layout L{3, 2, 1};
    const ocp::Trajectory back = ocp::unpack(L, ocp::pack(L, t));
    CHECK_NEAR(back.tf, 10.0, 0.0);
    CHECK_NEAR(back.X[2][1], 5.0, 0.0);
    CHECK_NEAR(back.U[1][0], 2.0, 0.0);
    const ocp::Trajectory r = ocp::resample(t, 5);
    CHECK(r.nodes() == 5);
    CHECK_NEAR(r.X[1][0], 1.0, 1e-12); // halfway between nodes 0 and 1
    CHECK_NEAR(r.X.back()[1], 5.0, 1e-12);
    CHECK_NEAR(r.U[3][0], 2.5, 1e-12);
}

TEST(waypoints_are_assigned_in_order) {
    ocp::Trajectory t;
    t.tf = 10.0;
    for (int k = 0; k < 11; ++k) t.X.push_back({100.0 * k, 0.0}), t.U.push_back({0.0});
    const std::vector<env::Waypoint> wps{{"A", {320.0, 10.0}, 40.0}, {"B", {280.0, -10.0}, 40.0}};
    const auto nodes = ocp::assignWaypoints(t, wps);
    CHECK(nodes[0].node == 3);
    CHECK(nodes[1].node == 4); // after A, even though node 3 is nearer
}

TEST(uf_problem_jacobian_matches_fd) {
    const MissionSpec m = leg(2000.0, true, true);
    const ocp::Trajectory g = wiggledUfLine(m, 9);
    const ocp::UfProblem p(generic(), m.ufMission(0.5, 0.01), g, ocp::assignWaypoints(g, m.waypoints));
    CHECK(p.numConstraints() == 8 * uf::NX + uf::NX + 4 + 9 * 13 + 1);
    CHECK(nlp::jacobianError(p, p.initialPoint()) < 1e-5);
    const Vec grad = p.costGradient(p.initialPoint());
    CHECK(static_cast<int>(grad.size()) == p.numVariables());
}

TEST(uf_to_fc_projection_is_near_equilibrium) {
    const Aircraft& ac = generic();
    const MissionSpec m = leg(2000.0, false, false);
    const Mission ms = m.ufMission(0.5, 0.0);
    const TrapezoidalUF tr(ac, ms, 6, UfScales::fromAircraft(ac, 2000.0));
    const ocp::Trajectory fcT = ws::ufToFc(ac, ocp::unpack(tr.layout(), trimmedLineGuess(tr, ac, ms)));
    const trim::FcTrim ref = trim::rigidBody(ac, {25.0, 300.0, 0.0, 0.0, 0.0});
    for (int k = 0; k < fcT.nodes(); ++k) {
        const Vec f = fc::rhs(ac, fcT.X[k], fcT.U[k]);
        CHECK(std::fabs(f[fc::U]) < 0.01 && std::fabs(f[fc::W]) < 0.01); // forces balanced [m/s^2]
        CHECK(std::fabs(f[fc::Q]) < 1e-3);                               // pitch moment balanced [rad/s^2]
        CHECK_NEAR(fcT.X[k][fc::THETA], ref.x[fc::THETA], deg2rad(0.05));
        CHECK_NEAR(fcT.U[k][fc::ELEVATOR], ref.u[fc::ELEVATOR], deg2rad(0.05));
    }
}

TEST(fc_problem_jacobian_matches_fd) {
    const Aircraft& ac = generic();
    const MissionSpec m = leg(2000.0, true, true);
    ocp::Trajectory g = ws::ufToFc(ac, wiggledUfLine(m, 6));
    ocp::FcMission fm;
    fm.x0 = ws::fcStartState(ac, m);
    fm.end = {2000.0, 0.0, -300.0};
    fm.alpha = 0.5;
    fm.betaControl = 0.01;
    fm.betaState = 0.02;
    fm.nfz = m.zones;
    fm.hLow = fm.hHigh = 300.0;
    const ocp::FcProblem p(ac, fm, g, ocp::assignWaypoints(g, m.waypoints, fc::XN, fc::YE));
    CHECK(nlp::jacobianError(p, p.initialPoint()) < 1e-4);
    const Vec z = p.initialPoint();
    const Vec gFd = num::fdJacobian([&](const Vec& v) { return Vec{p.cost(v)}; }, z);
    CHECK(num::maxRelativeError(p.costGradient(z), gFd) < 1e-6);
}

TEST(verification_names_the_violated_rows) {
    const MissionSpec m = leg(2000.0, true, false);
    const ocp::Trajectory cold = ws::coldUf(generic(), m, 30); // straight through the zone, zero controls
    const ocp::UfProblem p(generic(), m.ufMission(1.0, 0.0), cold);
    const nlp::Verification v = nlp::verify(p, p.initialPoint());
    CHECK(!v.ok());
    CHECK(v.rows[static_cast<int>(nlp::Kind::Environment)].violations > 0);
    CHECK(v.rows[static_cast<int>(nlp::Kind::Environment)].worstLabel.rfind("nfz:Z", 0) == 0);
    CHECK(v.rows[static_cast<int>(nlp::Kind::Dynamics)].violations > 0); // zero controls do not fly
    CHECK(v.rows[static_cast<int>(nlp::Kind::Boundary)].violations == 0);
}

TEST(solver_reports_when_it_is_not_built) {
    if (nlp::available()) return;
    const MissionSpec m = leg(2000.0, false, false);
    const ocp::UfProblem p(generic(), m.ufMission(1.0, 0.0), ws::coldUf(generic(), m, 10));
    const nlp::Result r = nlp::solve(p);
    CHECK(r.status == nlp::Status::NotAvailable);
    CHECK(r.z == p.initialPoint());
}

TEST(uf_minimum_time_is_faster_and_minimum_fuel_burns_less) {
    if (!nlp::available()) return;
    const Aircraft& ac = generic();
    const MissionSpec m = leg(2500.0, false, false);
    ocp::Trajectory opt[2];
    for (int i = 0; i < 2; ++i) {
        const Mission ms = m.ufMission(i == 0 ? 0.0 : 1.0, 0.01);
        const TrapezoidalUF tr(ac, ms, 30, UfScales::fromAircraft(ac, 2500.0));
        const ocp::UfProblem p(ac, ms, ocp::unpack(tr.layout(), trimmedLineGuess(tr, ac, ms)));
        const nlp::Result r = nlp::solve(p);
        CHECK(r.ok());
        CHECK(nlp::verify(p, r.z).ok());
        opt[i] = p.trajectory(r.z);
    }
    auto fuel = [](const ocp::Trajectory& t) { return t.X.front()[uf::MASS] - t.X.back()[uf::MASS]; };
    CHECK(opt[0].tf < opt[1].tf);
    CHECK(fuel(opt[1]) < fuel(opt[0]));
    CHECK(opt[0].tf < 2500.0 / ac.envelope.V_cruise); // faster than cruising
}

TEST(pipeline_flies_the_optimum_clear_of_the_zone) {
    if (!nlp::available()) return;
    const MissionSpec m = leg(2500.0, true, false);
    pipeline::Config cfg;
    cfg.nodesUf = cfg.nodesFc = 40;
    const pipeline::Result r = pipeline::run(generic(), m, cfg);
    CHECK(r.warmStage.ok);
    CHECK(r.uf.ok);
    CHECK(r.fc.ok);
    CHECK(r.fc.seed == "UF optimum (projected)");
    CHECK(pipeline::clearanceAlong(r.ufOpt, uf::X, uf::Y, m.zones, 300.0, 300.0) >= 0.0);
    CHECK(pipeline::clearanceAlong(r.fcOpt, fc::XN, fc::YE, m.zones, 300.0, 300.0) >= 0.0);
    // Same physics: with coordinated flight both models burn the same fuel.
    auto fuel = [](const ocp::Trajectory& t, int im) { return t.X.front()[im] - t.X.back()[im]; };
    CHECK(std::fabs(fuel(r.fcOpt, fc::MASS) / fuel(r.ufOpt, uf::MASS) - 1.0) < 0.02);
    for (int k = 0; k < r.fcOpt.nodes(); ++k)
        CHECK(std::fabs(fc::airData(r.fcOpt.X[k], {0.0, 0.0, 0.0}).beta) <= deg2rad(cfg.sideslipMax) + 1e-4);
    // The flown warm start is far closer to satisfying the dynamics than a straight line.
    pipeline::Config cold = cfg.withLabel("COLD>UF");
    cold.solver.maxIter = 0;
    CHECK(r.uf.guessDefect < 0.2 * pipeline::run(generic(), m, cold).uf.guessDefect);
}
