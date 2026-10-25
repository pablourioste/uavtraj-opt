#include "ofc/nlp.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace ofc::nlp {

const char* toString(Kind k) {
    switch (k) {
    case Kind::Dynamics:
        return "dynamics";
    case Kind::Boundary:
        return "boundary";
    case Kind::Envelope:
        return "envelope";
    case Kind::Environment:
        return "environment";
    }
    return "?";
}

bool Verification::ok() const { return totalViolations() == 0; }

int Verification::totalViolations() const {
    int n = bounds.violations;
    for (const Group& g : rows) n += g.violations;
    return n;
}

std::string Verification::summary() const {
    std::string s;
    auto add = [&s](const char* name, const Group& g) {
        if (!s.empty()) s += " | ";
        s += name;
        if (g.violations == 0) {
            s += " ok";
            return;
        }
        char buf[160];
        std::snprintf(buf, sizeof buf, " %d (%s %.1e)", g.violations, g.worstLabel.c_str(), g.worst);
        s += buf;
    };
    for (int k = 0; k < kNumKinds; ++k) add(toString(static_cast<Kind>(k)), rows[k]);
    add("bounds", bounds);
    return s;
}

Verification verify(const Problem& p, const Vec& z, double tol) {
    Verification v;
    v.tol = tol;
    auto record = [tol](Verification::Group& g, double excess, const std::string& label) {
        if (excess <= tol) return;
        ++g.violations;
        if (excess > g.worst) g.worst = excess, g.worstLabel = label;
    };

    Vec gl, gu, zl, zu;
    p.constraintBounds(gl, gu);
    p.variableBounds(zl, zu);
    const Vec g = p.constraints(z);
    for (int i = 0; i < p.numConstraints(); ++i) {
        const double excess = std::max(gl[i] - g[i], g[i] - gu[i]);
        record(v.rows[static_cast<int>(p.constraintKind(i))], std::isfinite(g[i]) ? excess : 1e300,
               p.constraintLabel(i));
    }
    // Bounds are compared relative to the variable's scale.
    const Vec s = p.variableScale();
    for (int j = 0; j < p.numVariables(); ++j)
        record(v.bounds, std::max(zl[j] - z[j], z[j] - zu[j]) / s[j], p.variableLabel(j));
    return v;
}

double jacobianError(const Problem& p, const Vec& z) {
    const int n = p.numVariables(), m = p.numConstraints();
    Vec dense(static_cast<std::size_t>(m) * n, 0.0);
    for (const Triplet& t : p.constraintJacobian(z)) dense[static_cast<std::size_t>(t.row) * n + t.col] += t.value;

    const Vec s = p.variableScale();
    double worst = 0.0;
    Vec zp = z;
    for (int j = 0; j < n; ++j) {
        const double h = 1e-6 * std::max(std::fabs(z[j]), s[j]);
        zp[j] = z[j] + h;
        const Vec gp = p.constraints(zp);
        zp[j] = z[j] - h;
        const Vec gm = p.constraints(zp);
        zp[j] = z[j];
        for (int i = 0; i < m; ++i) {
            const double fd = (gp[i] - gm[i]) / (2.0 * h);
            const double an = dense[static_cast<std::size_t>(i) * n + j];
            // Absolute error in units of the variable's scale, relative to the entry.
            const double err = std::fabs(an - fd) * s[j] / std::max(1.0, std::fabs(fd) * s[j]);
            worst = std::max(worst, err);
        }
    }
    return worst;
}

} // namespace ofc::nlp
