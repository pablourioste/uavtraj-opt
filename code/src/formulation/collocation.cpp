#include "ofc/collocation.hpp"

#include <algorithm>
#include <stdexcept>

namespace ofc::ocp {

Vec pack(const Layout& L, const Trajectory& t) {
    if (t.nodes() != L.N) throw std::invalid_argument("pack: trajectory has the wrong number of nodes");
    Vec z(L.numVariables());
    z[L.idxTf()] = t.tf;
    for (int k = 0; k < L.N; ++k) {
        for (int i = 0; i < L.nx; ++i) z[L.idxState(k, i)] = t.X[k][i];
        for (int j = 0; j < L.nu; ++j) z[L.idxControl(k, j)] = t.U[k][j];
    }
    return z;
}

Trajectory unpack(const Layout& L, const Vec& z) {
    Trajectory t;
    t.tf = z[L.idxTf()];
    t.X.assign(L.N, Vec(L.nx));
    t.U.assign(L.N, Vec(L.nu));
    for (int k = 0; k < L.N; ++k) {
        for (int i = 0; i < L.nx; ++i) t.X[k][i] = z[L.idxState(k, i)];
        for (int j = 0; j < L.nu; ++j) t.U[k][j] = z[L.idxControl(k, j)];
    }
    return t;
}

Trajectory resample(const Trajectory& t, int N) {
    if (t.nodes() < 2 || N < 2) throw std::invalid_argument("resample: need at least two nodes");
    Trajectory r;
    r.tf = t.tf;
    r.X.resize(N);
    r.U.resize(N);
    const int M = t.nodes();
    for (int k = 0; k < N; ++k) {
        const double s = static_cast<double>(k) * (M - 1) / (N - 1);
        const int i0 = std::min(static_cast<int>(s), M - 2);
        const double w = s - i0;
        auto lerp = [w](const Vec& a, const Vec& b) {
            Vec v(a.size());
            for (std::size_t i = 0; i < a.size(); ++i) v[i] = (1.0 - w) * a[i] + w * b[i];
            return v;
        };
        r.X[k] = lerp(t.X[i0], t.X[i0 + 1]);
        r.U[k] = lerp(t.U[i0], t.U[i0 + 1]);
    }
    return r;
}

Vec trapezoidalDefects(const Layout& L, double tf, const std::vector<Vec>& X, const std::vector<Vec>& F,
                       const Vec& sx) {
    const double dt = tf / (L.N - 1);
    Vec d(L.numDefects());
    for (int k = 0; k < L.N - 1; ++k)
        for (int i = 0; i < L.nx; ++i)
            d[k * L.nx + i] = (X[k + 1][i] - X[k][i] - 0.5 * dt * (F[k][i] + F[k + 1][i])) / sx[i];
    return d;
}

std::vector<Triplet> trapezoidalJacobian(const Layout& L, double tf, const std::vector<Vec>& F,
                                         const std::vector<Vec>& A, const std::vector<Vec>& B, const Vec& sx,
                                         const std::vector<bool>& patternA, const std::vector<bool>& patternB) {
    const int nx = L.nx, nu = L.nu;
    const double dt = tf / (L.N - 1);
    std::vector<Triplet> J;
    J.reserve(static_cast<std::size_t>(L.N - 1) * nx * (2 * nx + 2 * nu + 1));
    for (int k = 0; k < L.N - 1; ++k) {
        for (int i = 0; i < nx; ++i) {
            const int row = k * nx + i;
            const double s = 1.0 / sx[i];
            J.push_back({row, L.idxTf(), -s * (F[k][i] + F[k + 1][i]) / (2.0 * (L.N - 1))});
            for (int j = 0; j < nx; ++j) {
                const bool diag = i == j;
                if (!diag && !patternA[i * nx + j]) continue;
                const double eye = diag ? 1.0 : 0.0;
                J.push_back({row, L.idxState(k, j), s * (-eye - 0.5 * dt * A[k][i * nx + j])});
                J.push_back({row, L.idxState(k + 1, j), s * (eye - 0.5 * dt * A[k + 1][i * nx + j])});
            }
            for (int j = 0; j < nu; ++j) {
                if (!patternB[i * nu + j]) continue;
                J.push_back({row, L.idxControl(k, j), -s * 0.5 * dt * B[k][i * nu + j]});
                J.push_back({row, L.idxControl(k + 1, j), -s * 0.5 * dt * B[k + 1][i * nu + j]});
            }
        }
    }
    return J;
}

} // namespace ofc::ocp
