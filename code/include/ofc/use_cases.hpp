#pragma once
// Public use cases. Each one reads a JSON case file (see data/cases/),
// prints a report and, from UC-03 on, writes CSV files for plotting.
//
//   UC-01  Aircraft report      derived performance of a generic aircraft
//   UC-02  Trim                 steady flight, point mass (UF) and rigid body (FC)
//   UC-03  Simulation           open-loop RK4 simulation of both models
//   UC-04  Transcription check  NLP size, sparsity, seed quality and
//                               analytic vs finite-difference Jacobians
//   UC-05  Design charts        power curves, flight envelope, trim curves,
//                               turn performance
//   UC-06  Design scenarios     typical missions flown by UF and FC, speed
//                               trade-off and evaluation in the NLP
//   UC-07  Constrained missions no-fly zones and waypoints: Hybrid A* path,
//                               L1 guidance for UF and FC, NFZ rows in the NLP
//   UC-08  Optimal missions     warm start -> UF -> FC optimisation with IPOPT,
//                               verified and exported (needs IPOPT)
//   UC-09  Warm-start benchmark cold start vs warm start, direct FC vs
//                               UF -> FC, on the same missions (needs IPOPT)

#include <string>

#include <nlohmann/json.hpp>

namespace ofc::cases {

int aircraftReport(const nlohmann::json& c);
int trimReport(const nlohmann::json& c);
int simulation(const nlohmann::json& c);
int transcriptionCheck(const nlohmann::json& c);
int designCharts(const nlohmann::json& c);
int designScenarios(const nlohmann::json& c);
int missionScenarios(const nlohmann::json& c);
int optimalMissions(const nlohmann::json& c);
int warmStartBenchmark(const nlohmann::json& c);

// Dispatches on the "use_case" field. Returns the process exit code.
int run(const std::string& casePath);

} // namespace ofc::cases
