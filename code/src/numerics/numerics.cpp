#include "ofc/numerics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ofc::num {

Vec fdJacobian(const VecFn& f, const Vec& x, FdScheme scheme, double relStep) {
    const Vec f0 = f(x);
    const std::size_t m = f0.size(), n = x.size();
    Vec J(m * n, 0.0);
    Vec xp = x;
    for (std::size_t j = 0; j < n; ++j) {
        const double h = relStep * std::max(1.0, std::fabs(x[j]));
        xp[j] = x[j] + h;
        const Vec fp = f(xp);
        if (scheme == FdScheme::Central) {
            xp[j] = x[j] - h;
            const Vec fm = f(xp);
            for (std::size_t i = 0; i < m; ++i) J[i * n + j] = (fp[i] - fm[i]) / (2.0 * h);
        } else {
            for (std::size_t i = 0; i < m; ++i) J[i * n + j] = (fp[i] - f0[i]) / h;
        }
        xp[j] = x[j];
    }
    return J;
}

double maxRelativeError(const Vec& A, const Vec& B) {
    if (A.size() != B.size()) throw std::invalid_argument("maxRelativeError: size mismatch");
    double e = 0.0;
    for (std::size_t i = 0; i < A.size(); ++i) e = std::max(e, std::fabs(A[i] - B[i]) / std::max(1.0, std::fabs(B[i])));
    return e;
}

Vec solveDense(Vec M, Vec r) {
    const std::size_t n = r.size();
    for (std::size_t c = 0; c < n; ++c) {
        std::size_t piv = c;
        for (std::size_t i = c + 1; i < n; ++i)
            if (std::fabs(M[i * n + c]) > std::fabs(M[piv * n + c])) piv = i;
        if (std::fabs(M[piv * n + c]) < 1e-14) throw std::runtime_error("solveDense: singular matrix");
        if (piv != c) {
            for (std::size_t j = 0; j < n; ++j) std::swap(M[c * n + j], M[piv * n + j]);
            std::swap(r[c], r[piv]);
        }
        for (std::size_t i = c + 1; i < n; ++i) {
            const double f = M[i * n + c] / M[c * n + c];
            for (std::size_t j = c; j < n; ++j) M[i * n + j] -= f * M[c * n + j];
            r[i] -= f * r[c];
        }
    }
    Vec x(n);
    for (std::size_t i = n; i-- > 0;) {
        double s = r[i];
        for (std::size_t j = i + 1; j < n; ++j) s -= M[i * n + j] * x[j];
        x[i] = s / M[i * n + i];
    }
    return x;
}

NewtonResult newton(const VecFn& F, Vec x0, double tol, int maxIter) {
    NewtonResult res;
    res.x = std::move(x0);
    Vec r = F(res.x);
    res.residual = maxAbs(r);
    for (res.iterations = 0; res.iterations < maxIter && res.residual > tol; ++res.iterations) {
        const Vec J = fdJacobian(F, res.x);
        Vec rhs(r.size());
        for (std::size_t i = 0; i < r.size(); ++i) rhs[i] = -r[i];
        const Vec dx = solveDense(J, rhs);

        double step = 1.0;
        for (int ls = 0; ls < 30; ++ls, step *= 0.5) {
            Vec xt = res.x;
            for (std::size_t i = 0; i < xt.size(); ++i) xt[i] += step * dx[i];
            const Vec rt = F(xt);
            if (maxAbs(rt) < res.residual || ls == 29) {
                res.x = std::move(xt);
                r = rt;
                break;
            }
        }
        res.residual = maxAbs(r);
    }
    res.converged = res.residual <= tol;
    return res;
}

Vec rk4Step(const OdeFn& f, double t, const Vec& x, double dt) {
    const std::size_t n = x.size();
    auto axpy = [n](const Vec& a, double s, const Vec& b) {
        Vec out(n);
        for (std::size_t i = 0; i < n; ++i) out[i] = a[i] + s * b[i];
        return out;
    };
    const Vec k1 = f(t, x);
    const Vec k2 = f(t + 0.5 * dt, axpy(x, 0.5 * dt, k1));
    const Vec k3 = f(t + 0.5 * dt, axpy(x, 0.5 * dt, k2));
    const Vec k4 = f(t + dt, axpy(x, dt, k3));
    Vec out(n);
    for (std::size_t i = 0; i < n; ++i) out[i] = x[i] + dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    return out;
}

Trajectory simulate(const OdeFn& f, const Vec& x0, double tEnd, double dt) {
    Trajectory tr;
    const int steps = static_cast<int>(std::ceil(tEnd / dt - 1e-9));
    tr.t.reserve(steps + 1);
    tr.x.reserve(steps + 1);
    tr.t.push_back(0.0);
    tr.x.push_back(x0);
    for (int k = 0; k < steps; ++k) {
        const double t = k * dt;
        const double h = std::min(dt, tEnd - t);
        tr.x.push_back(rk4Step(f, t, tr.x.back(), h));
        tr.t.push_back(t + h);
    }
    return tr;
}

} // namespace ofc::num
