#pragma once
// Interior-point solution of an nlp::Problem with IPOPT.
//
// The adapter scales every variable by Problem::variableScale, passes the
// exact gradient and the sparse constraint Jacobian, and approximates the
// Hessian of the Lagrangian with limited-memory BFGS. IPOPT is optional at
// build time (CMake option OFC_IPOPT): without it, solve() returns
// Status::NotAvailable and everything else in the library still works.

#include <string>

#include <nlohmann/json.hpp>

#include "ofc/nlp.hpp"
#include "ofc/types.hpp"

namespace ofc::nlp {

struct Options {
    double tol = 1e-6;           // overall NLP error (scaled)
    double acceptableTol = 1e-4; // "solved to acceptable level"
    double constrViolTol = 1e-6; // absolute constraint violation; rows are normalised
    int maxIter = 1500;          //
    double maxCpuTime = 120.0;   // [s]
    int printLevel = 0;          // IPOPT console output, 0 (silent) to 12
    std::string muStrategy = "adaptive";
    int lbfgsHistory = 6;        // pairs kept by the limited-memory Hessian approximation
    std::string linearSolver;    // empty = IPOPT's default (MUMPS in most builds)
    bool derivativeTest = false; // IPOPT's first-order derivative checker

    static Options fromJson(const nlohmann::json& j); // missing keys keep the defaults
};

enum class Status { Solved, Acceptable, Infeasible, MaxIterations, MaxTime, Failed, NotAvailable };
const char* toString(Status s);

struct Result {
    Status status = Status::NotAvailable;
    std::string message; // IPOPT's return code, or why the solver did not run
    Vec z;               // final point (the initial point if the solver did not run)
    double objective = 0.0;
    int iterations = 0;
    double millis = 0.0;

    bool ok() const { return status == Status::Solved || status == Status::Acceptable; }
};

bool available(); // true when the library was built with IPOPT

Result solve(const Problem& p, const Options& opt = {});

} // namespace ofc::nlp
