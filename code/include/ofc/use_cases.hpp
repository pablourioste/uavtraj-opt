#pragma once
// Public use cases. Each one reads a JSON case file (see data/cases/) and
// prints a report; UC-03 and UC-04 also write CSV files for plotting.
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

#include <string>

#include <nlohmann/json.hpp>

namespace ofc::cases {

int aircraftReport(const nlohmann::json& c);
int trimReport(const nlohmann::json& c);
int simulation(const nlohmann::json& c);
int transcriptionCheck(const nlohmann::json& c);
int designCharts(const nlohmann::json& c);
int designScenarios(const nlohmann::json& c);

// Dispatches on the "use_case" field. Returns the process exit code.
int run(const std::string& casePath);

} // namespace ofc::cases
