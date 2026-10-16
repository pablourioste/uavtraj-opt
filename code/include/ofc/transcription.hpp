#pragma once
// Trapezoidal direct collocation of the point-mass (UF) optimal-control problem.
//
// Decision vector (all states, then all controls):
//     z = [ t_f | x_0 ... x_{N-1} | u_0 ... u_{N-1} ]        size 1 + N (NX + NU)
//
// Dynamics become N-1 algebraic defects, each normalised by a state scale:
//     Delta_k = ( x_{k+1} - x_k - dt/2 [ f(x_k,u_k) + f(x_{k+1},u_{k+1}) ] ) / s_x,
//     dt = t_f / (N-1).
//
// This class only *evaluates* the NLP functions (cost, constraints and their
// derivatives). It is solver-agnostic: an interior-point solver such as IPOPT
// would call these functions at every iteration. No solver is bundled here.

#include <string>
#include <vector>

#include "ofc/aircraft.hpp"
#include "ofc/point_mass.hpp"
#include "ofc/types.hpp"

namespace ofc {

struct Triplet {
    int row = 0, col = 0;
    double value = 0.0;
};

struct UfScales {
    Vec x; // one reference value per state
    Vec u; // one reference value per control
    double tf = 1.0;
    static UfScales fromAircraft(const Aircraft& ac, double distance);
};

struct Mission {
    Vec3 start{0.0, 0.0, -100.0}; // NED position [m]
    Vec3 end{2000.0, 0.0, -100.0};
    double headingStart = 0.0; // [rad]
    double headingEnd = 0.0;
    double speedStart = 0.0;  // [m/s]; <= 0 means V_cruise
    double gammaStart = 0.0;  // initial flight-path angle [rad]
    Vec3 wind{0.0, 0.0, 0.0}; // constant NED wind [m/s]
    double alpha = 0.0;       // 0 = minimum time, 1 = minimum fuel
    double betaControl = 0.0; // smoothness weight on control increments
};

class TrapezoidalUF {
public:
    TrapezoidalUF(const Aircraft& ac, const Mission& mission, int N, UfScales scales);

    int nodes() const { return N_; }
    int numVariables() const { return 1 + N_ * (uf::NX + uf::NU); }
    int numDefects() const { return (N_ - 1) * uf::NX; }

    int idxTf() const { return 0; }
    int idxState(int k, int i) const { return 1 + k * uf::NX + i; }
    int idxControl(int k, int j) const { return 1 + N_ * uf::NX + k * uf::NU + j; }

    Vec pack(double tf, const std::vector<Vec>& X, const std::vector<Vec>& U) const;
    void unpack(const Vec& z, double& tf, std::vector<Vec>& X, std::vector<Vec>& U) const;

    // ── Equality constraints ────────────────────────────────────────────
    Vec defects(const Vec& z) const;
    std::vector<Triplet> defectJacobian(const Vec& z) const; // analytic, block-banded
    Vec boundaryResiduals(const Vec& z) const;               // initial state, final position + heading

    // ── Inequality (path) constraints, convention h(z) <= 0 ─────────────
    struct PathConstraint {
        std::string name;
        double worst = -1e300; // largest value over all nodes (> 0 = violated)
        int node = -1;
    };
    Vec pathConstraints(const Vec& z) const; // all nodes, all constraints
    std::vector<PathConstraint> pathSummary(const Vec& z) const;
    static const std::vector<std::string>& pathConstraintNames();

    // ── Cost (Bolza: time + fuel + smoothness) ──────────────────────────
    double cost(const Vec& z) const;
    Vec costGradient(const Vec& z) const; // analytic

    const UfScales& scales() const { return scales_; }

private:
    const Aircraft& ac_;
    Mission mission_;
    int N_;
    UfScales scales_;
    double energyRef_;
    std::vector<bool> patternA_, patternB_; // structural nonzeros of df/dx, df/du
};

// ── Initial guesses ────────────────────────────────────────────────────
// Cold start: straight line at constant speed, zero controls (the classic
// "linear interpolation" guess that makes interior-point solvers struggle).
Vec coldStartGuess(const TrapezoidalUF& nlp, const Aircraft& ac, const Mission& mission);
// Trimmed straight line: the same geometry, but every node carries the
// point-mass trim controls and a consistent mass history. A minimal stand-in
// for the full warm-start chain of the thesis (not included here).
Vec trimmedLineGuess(const TrapezoidalUF& nlp, const Aircraft& ac, const Mission& mission);

} // namespace ofc
