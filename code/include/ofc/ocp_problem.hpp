#pragma once
// Common shell of a collocated optimal-control problem.
//
// Constraint vector, in this order:
//
//     g(z) = [ defects | boundary | path rows | waypoint rows ]
//             = 0        = 0        <= 0        <= 0
//
//   defects    (N-1) nx rows from the model (trapezoidal collocation)
//   boundary   linear rows (z[col] - target) / scale: fixed initial state,
//              final position and heading
//   path       nc rows per node, h(x_k, u_k) <= 0 (envelope and no-fly zones)
//   waypoints  one row per waypoint, pinned to a node k:
//              ((x_k - x_wp)^2 + (y_k - y_wp)^2) / tol^2 - 1 <= 0
//
// A model (UfProblem, FcProblem) supplies the defects and their Jacobian, the
// per-node path function, the cost, the names, scales and boxes. This class
// assembles everything else, including the full sparse constraint Jacobian:
// the path rows of node k depend only on (x_k, u_k), so their Jacobian is a
// dense nc x (nx + nu) block per node, filled by central differences.

#include <string>
#include <vector>

#include "ofc/collocation.hpp"
#include "ofc/environment.hpp"
#include "ofc/nlp.hpp"

namespace ofc::ocp {

// A waypoint attached to the collocation node that must pass within its tolerance.
struct WaypointNode {
    env::Waypoint wp;
    int node = 0;
};

// Characteristic length [m] and time [s] of a mission. They scale the
// variables, normalise the cost and bound t_f. Taking them from the mission
// rather than from the initial guess means that different seeds solve exactly
// the same problem. Values <= 0 fall back to the guess's length and duration.
struct Reference {
    double length = 0.0, time = 0.0;
};

class CollocationProblem : public nlp::Problem {
public:
    int numVariables() const override { return layout_.numVariables(); }
    int numConstraints() const override { return offWaypoints_ + static_cast<int>(waypoints_.size()); }
    void variableBounds(Vec& zl, Vec& zu) const override;
    void constraintBounds(Vec& gl, Vec& gu) const override;
    Vec initialPoint() const override { return z0_; }
    Vec variableScale() const override;
    Vec constraints(const Vec& z) const override;
    std::vector<Triplet> constraintJacobian(const Vec& z) const override;
    std::string constraintLabel(int i) const override;
    nlp::Kind constraintKind(int i) const override;
    std::string variableLabel(int j) const override;

    const Layout& layout() const { return layout_; }
    Trajectory trajectory(const Vec& z) const { return unpack(layout_, z); }
    const std::vector<WaypointNode>& waypoints() const { return waypoints_; }
    const std::vector<std::string>& pathNames() const { return info_.pathNames; }

    // Largest path-constraint value of each row name over all nodes (> 0 = violated).
    std::vector<double> worstPath(const Vec& z) const;

protected:
    struct Box {
        double lo = -nlp::kInf, hi = nlp::kInf;
    };
    struct BoundaryRow {
        std::string label;
        int col = 0;
        double target = 0.0, scale = 1.0;
    };
    struct ModelInfo {
        std::vector<std::string> stateNames, controlNames, pathNames;
        Vec stateScale, controlScale;
        double tfScale = 1.0;
        std::vector<Box> stateBox, controlBox;
        Box tfBox;
        int idxX = 0, idxY = 1; // horizontal position inside the state vector
    };

    // Called once by the derived constructor, after its model is ready.
    void setup(const Layout& layout, ModelInfo info, std::vector<BoundaryRow> boundary, Vec z0,
               std::vector<WaypointNode> waypoints);

    virtual Vec defects(const Vec& z) const = 0;
    virtual std::vector<Triplet> defectJacobian(const Vec& z) const = 0;
    virtual Vec pathAt(const Vec& x, const Vec& u) const = 0;

private:
    Layout layout_;
    ModelInfo info_;
    std::vector<BoundaryRow> boundary_;
    std::vector<WaypointNode> waypoints_;
    Vec z0_;
    int nc_ = 0;
    int offBoundary_ = 0, offPath_ = 0, offWaypoints_ = 0;
};

// Pins each waypoint to the node of `guess` nearest to it, visiting the
// waypoints in order (each search starts after the previous match).
std::vector<WaypointNode> assignWaypoints(const Trajectory& guess, const std::vector<env::Waypoint>& wps, int idxX = 0,
                                          int idxY = 1);

// Adds the multiple of 2 pi to `angle` that brings it closest to `reference`.
double unwrapNear(double angle, double reference);

} // namespace ofc::ocp
