#pragma once
// Staged trajectory optimisation of one mission:
//
//   1. warm start   Hybrid A* path flown by both models (ws::build), or a
//                   straight-line cold start
//   2. UF stage     point-mass problem (ocp::UfProblem) solved with IPOPT
//   3. FC stage     rigid-body problem (ocp::FcProblem) solved with IPOPT,
//                   seeded by the projected UF optimum (ws::ufToFc) or
//                   directly by the warm start
//   4. verification every optimum is re-checked against all its constraints
//                   and bounds (nlp::verify), outside the solver
//
// Each stage records its status, iterations, wall-clock time and the quality
// of its initial point, so that pipelines can be compared (UC-09).

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ofc/aircraft.hpp"
#include "ofc/collocation.hpp"
#include "ofc/mission.hpp"
#include "ofc/nlp.hpp"
#include "ofc/ocp_problem.hpp"
#include "ofc/solver.hpp"
#include "ofc/warm_start.hpp"

namespace ofc::pipeline {

enum class Seed { WarmStart, Cold };

struct Config {
    Seed seed = Seed::WarmStart;
    bool uf = true;       // run the point-mass stage
    bool fc = true;       // run the rigid-body stage
    bool fcFromUf = true; // seed FC with the UF optimum when it is available
    int nodesUf = 60, nodesFc = 60;
    double alpha = 1.0;       // 0 = minimum time, 1 = minimum fuel
    double betaControl = 1.0; // smoothness weight on control increments
    double betaState = 1.0;   // smoothness weight on attitude increments
    double sideslipMax = 1.0; // FC sideslip limit [deg] (coordinated flight)
    ws::Config warmStart;
    nlp::Options solver;
    double verifyTol = 1e-4;

    // "WS>UF>FC", "WS>FC", "COLD>UF", ...
    std::string label() const;
    // Same settings with the seed and stages of a label.
    Config withLabel(const std::string& label) const;
    // Keys: nodes_uf, nodes_fc, alpha, beta_control, beta_state, sideslip_max_deg, seed ("warm"|"cold"),
    // stages (e.g. "UF>FC"), fc_from_uf, verify_tol, planner {...}, solver {...}
    static Config fromJson(const nlohmann::json& j);
};

struct Stage {
    bool ran = false;
    bool ok = false;     // solver converged and the verification passed
    std::string status;  // nlp::toString(status), or "ok"/"failed" for the warm start
    std::string message; // solver return code or reason
    std::string seed;    // where the initial point came from
    double millis = 0.0;
    int iterations = 0;
    double objective = 0.0;
    double guessDefect = 0.0;  // largest normalised collocation defect of the initial point
    double penaltyShare = 0.0; // share of the final cost due to the smoothness terms
    nlp::Verification check;   // of the final point
};

struct Result {
    ws::Result warm;
    Stage warmStage, uf, fc;
    ocp::Trajectory ufGuess, ufOpt, fcGuess, fcOpt;
    double totalMillis = 0.0;

    bool ok() const; // every stage that ran succeeded
};

Result run(const Aircraft& ac, const MissionSpec& m, const Config& cfg);

// RK4 simulation of the UF model driven by the optimal controls (linearly
// interpolated between nodes), compared with the nodes [m]:
//   local  each interval restarted from its node: the discretisation error
//          of the collocation, independent of the flight's length
//   drift  one open-loop run over the whole flight: grows with time because
//          heading and speed errors integrate (a tracking loop removes it)
struct SimulationError {
    double local = 0.0, drift = 0.0;
};
SimulationError ufSimulationError(const Aircraft& ac, const ocp::Trajectory& uf);

// Smallest clearance beyond the margin [m] to the zones active in the
// altitude band, along straight segments between nodes (+inf if no zone).
double clearanceAlong(const ocp::Trajectory& t, int idxX, int idxY, const std::vector<env::NoFlyZone>& zones,
                      double hLow, double hHigh);

// Largest horizontal distance [m] from each waypoint to the trajectory.
double waypointMissAlong(const ocp::Trajectory& t, int idxX, int idxY, const std::vector<env::Waypoint>& wps);

} // namespace ofc::pipeline
