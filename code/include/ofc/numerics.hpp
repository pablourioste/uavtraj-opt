#pragma once
// Generic numerical tools: finite-difference Jacobians, RK4 integration and a
// small dense Newton solver used by the trim routines.

#include <functional>

#include "ofc/types.hpp"

namespace ofc::num {

using VecFn = std::function<Vec(const Vec&)>;

// Dense finite-difference Jacobian of f at x, row-major (m x n).
// Central differences by default; forward differences mirror the v1 solver.
enum class FdScheme { Forward, Central };
Vec fdJacobian(const VecFn& f, const Vec& x, FdScheme scheme = FdScheme::Central, double relStep = 1e-6);

// Largest |A - B| over all entries, scaled by max(1, |B|).
double maxRelativeError(const Vec& A, const Vec& B);

// Solve the dense linear system M x = r (row-major M, n x n) by Gaussian
// elimination with partial pivoting. Throws std::runtime_error if singular.
Vec solveDense(Vec M, Vec r);

// Newton iterations on F(x) = 0 with a finite-difference Jacobian and a
// simple backtracking line search on |F|.
struct NewtonResult {
    Vec x;
    double residual = 0.0;
    int iterations = 0;
    bool converged = false;
};
NewtonResult newton(const VecFn& F, Vec x0, double tol = 1e-10, int maxIter = 50);

// Time-varying ODE x_dot = f(t, x).
using OdeFn = std::function<Vec(double, const Vec&)>;
Vec rk4Step(const OdeFn& f, double t, const Vec& x, double dt);

struct Trajectory {
    Vec t;
    std::vector<Vec> x;
};
Trajectory simulate(const OdeFn& f, const Vec& x0, double tEnd, double dt);

} // namespace ofc::num
