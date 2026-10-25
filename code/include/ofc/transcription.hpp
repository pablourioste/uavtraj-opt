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
// This class evaluates the NLP functions (cost, constraints and their
// derivatives). ocp::UfProblem (ocp_uf.hpp) wraps it into a complete
// nonlinear program with bounds, waypoints and a full constraint Jacobian,
// which nlp::solve hands to IPOPT.

#include <algorithm>
#include <string>
#include <vector>

#include "ofc/aircraft.hpp"
#include "ofc/collocation.hpp"
#include "ofc/environment.hpp"
#include "ofc/point_mass.hpp"
#include "ofc/types.hpp"

namespace ofc {

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
    double speedStart = 0.0;         // [m/s]; <= 0 means V_cruise
    double gammaStart = 0.0;         // initial flight-path angle [rad]
    Vec3 wind{0.0, 0.0, 0.0};        // constant NED wind [m/s]
    double alpha = 0.0;              // 0 = minimum time, 1 = minimum fuel
    double betaControl = 0.0;        // smoothness weight on control increments (T, CL, p)
    double betaState = 0.0;          // smoothness weight on attitude increments (gamma, mu)
    std::vector<env::NoFlyZone> nfz; // zones enforced as path constraints

    // Altitude band of the mission (start and end altitudes). A zone takes part
    // in the problem only if its band overlaps this one, the same rule as the
    // planner; a fixed choice keeps every constraint row smooth in z.
    double hLow() const { return std::min(-start[2], -end[2]); }
    double hHigh() const { return std::max(-start[2], -end[2]); }
};

class TrapezoidalUF {
public:
    TrapezoidalUF(const Aircraft& ac, const Mission& mission, int N, UfScales scales);

    int nodes() const { return N_; }
    int numVariables() const { return layout_.numVariables(); }
    int numDefects() const { return layout_.numDefects(); }
    const ocp::Layout& layout() const { return layout_; }
    const Mission& mission() const { return mission_; }

    int idxTf() const { return 0; }
    int idxState(int k, int i) const { return layout_.idxState(k, i); }
    int idxControl(int k, int j) const { return layout_.idxControl(k, j); }

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
    Vec pathConstraints(const Vec& z) const;      // all nodes, all constraints: row c of node k is h[k * nc + c]
    Vec pathAt(const Vec& x, const Vec& u) const; // the nc rows of one node
    std::vector<PathConstraint> pathSummary(const Vec& z) const;
    std::vector<std::string> pathConstraintNames() const; // envelope rows, then one "nfz:<id>" row per zone

    // ── Cost (Bolza: time + fuel + smoothness) ──────────────────────────
    double cost(const Vec& z) const;
    Vec costGradient(const Vec& z) const; // analytic

    const UfScales& scales() const { return scales_; }

private:
    const Aircraft& ac_;
    Mission mission_;
    int N_;
    ocp::Layout layout_;
    UfScales scales_;
    double energyRef_;
    std::vector<bool> patternA_, patternB_; // structural nonzeros of df/dx, df/du
    std::vector<bool> nfzActive_;           // zone overlaps the mission's altitude band
};

// ── Initial guesses ────────────────────────────────────────────────────
// Cold start: straight line at constant speed, zero controls (the classic
// "linear interpolation" guess that makes interior-point solvers struggle).
Vec coldStartGuess(const TrapezoidalUF& nlp, const Aircraft& ac, const Mission& mission);
// Trimmed straight line: the same geometry, but every node carries the
// point-mass trim controls and a consistent mass history. A minimal stand-in
// for the warm-start chain (ws::build in warm_start.hpp).
Vec trimmedLineGuess(const TrapezoidalUF& nlp, const Aircraft& ac, const Mission& mission);

} // namespace ofc
