#pragma once
// Generic nonlinear program
//
//     min f(z)   subject to   g_l <= g(z) <= g_u,   z_l <= z <= z_u
//
// with a sparse constraint Jacobian whose structure does not change with z.
// Every optimal-control problem (ocp::UfProblem, ocp::FcProblem) implements
// this interface, nlp::solve hands it to IPOPT, and nlp::verify checks a
// solution afterwards, independently of the solver's own report.

#include <string>
#include <vector>

#include "ofc/collocation.hpp"
#include "ofc/types.hpp"

namespace ofc::nlp {

constexpr double kInf = 1e19; // IPOPT's default "no bound"

// What a constraint row represents; used to group the verification report.
enum class Kind { Dynamics, Boundary, Envelope, Environment };
constexpr int kNumKinds = 4;
const char* toString(Kind k);

class Problem {
public:
    virtual ~Problem() = default;

    virtual int numVariables() const = 0;
    virtual int numConstraints() const = 0;

    virtual void variableBounds(Vec& zl, Vec& zu) const = 0;
    virtual void constraintBounds(Vec& gl, Vec& gu) const = 0;
    virtual Vec initialPoint() const = 0;
    // Typical magnitude of each variable. The solver works with z / scale,
    // so that every variable it sees is of order one.
    virtual Vec variableScale() const { return Vec(numVariables(), 1.0); }

    virtual double cost(const Vec& z) const = 0;
    virtual Vec costGradient(const Vec& z) const = 0;
    virtual Vec constraints(const Vec& z) const = 0;
    // Same (row, col) sequence at every call; only the values change.
    virtual std::vector<Triplet> constraintJacobian(const Vec& z) const = 0;

    virtual std::string constraintLabel(int i) const { return "g[" + std::to_string(i) + "]"; }
    virtual Kind constraintKind(int) const { return Kind::Envelope; }
    virtual std::string variableLabel(int j) const { return "z[" + std::to_string(j) + "]"; }
};

// Result of checking a point against every constraint and bound.
struct Verification {
    struct Group {
        int violations = 0;
        double worst = 0.0; // largest violation (0 when satisfied)
        std::string worstLabel;
    };
    double tol = 0.0;
    Group rows[kNumKinds]; // indexed by Kind
    Group bounds;          // variable bounds

    bool ok() const;
    int totalViolations() const;
    std::string summary() const; // one line: "dynamics ok | boundary 2 (xf[X] 3.1e-3) | ..."
};

// A row or bound counts as violated when it is outside its limits by more
// than tol (constraints are normalised, so tol is relative).
Verification verify(const Problem& p, const Vec& z, double tol = 1e-4);

// Largest relative difference between the problem's sparse Jacobian and a
// dense central-difference one (step scaled by variableScale). For tests.
double jacobianError(const Problem& p, const Vec& z);

} // namespace ofc::nlp
