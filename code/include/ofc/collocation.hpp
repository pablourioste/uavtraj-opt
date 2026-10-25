#pragma once
// Model-independent pieces of trapezoidal direct collocation, shared by the
// point-mass (UF) and rigid-body (FC) problems.
//
// Decision vector (all states, then all controls):
//     z = [ t_f | x_0 ... x_{N-1} | u_0 ... u_{N-1} ]        size 1 + N (nx + nu)
//
// Defect of interval k, normalised by one scale per state:
//     Delta_k = ( x_{k+1} - x_k - dt/2 [ f_k + f_{k+1} ] ) / s_x,     dt = t_f / (N-1)

#include <vector>

#include "ofc/types.hpp"

namespace ofc {

// One nonzero of a sparse matrix (coordinate format).
struct Triplet {
    int row = 0, col = 0;
    double value = 0.0;
};

} // namespace ofc

namespace ofc::ocp {

struct Layout {
    int N = 0, nx = 0, nu = 0;

    int numVariables() const { return 1 + N * (nx + nu); }
    int numDefects() const { return (N - 1) * nx; }
    int idxTf() const { return 0; }
    int idxState(int k, int i) const { return 1 + k * nx + i; }
    int idxControl(int k, int j) const { return 1 + N * nx + k * nu + j; }
};

// States and controls on N uniform nodes over [0, t_f].
struct Trajectory {
    double tf = 0.0;
    std::vector<Vec> X, U;

    int nodes() const { return static_cast<int>(X.size()); }
    double time(int k) const { return tf * k / (nodes() - 1); }
};

Vec pack(const Layout& L, const Trajectory& t);
Trajectory unpack(const Layout& L, const Vec& z);

// Linear interpolation of a trajectory onto N uniform nodes.
Trajectory resample(const Trajectory& t, int N);

// Defects of every interval, given the right-hand side f at every node.
Vec trapezoidalDefects(const Layout& L, double tf, const std::vector<Vec>& X, const std::vector<Vec>& F, const Vec& sx);

// Analytic Jacobian of the defects from the model Jacobians A_k = df/dx and
// B_k = df/du (row-major). Only entries marked in the structural patterns are
// stored; the diagonal and the t_f column are always present, so the sparsity
// structure (order and positions of the triplets) never depends on z.
std::vector<Triplet> trapezoidalJacobian(const Layout& L, double tf, const std::vector<Vec>& F,
                                         const std::vector<Vec>& A, const std::vector<Vec>& B, const Vec& sx,
                                         const std::vector<bool>& patternA, const std::vector<bool>& patternB);

} // namespace ofc::ocp
