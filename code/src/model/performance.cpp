#include "ofc/performance.hpp"

#include <algorithm>
#include <cmath>

#include "ofc/atmosphere.hpp"

namespace ofc::perf {

namespace {
constexpr int kSamples = 400;

template <class F> SpeedPoint scanMax(const Aircraft& ac, double h, double m, F&& f) {
    const double V0 = ac.stallSpeed(h, m), V1 = ac.envelope.V_ne;
    SpeedPoint best{V0, -1e300};
    for (int i = 0; i <= kSamples; ++i) {
        const double V = V0 + (V1 - V0) * i / kSamples;
        const double v = f(V);
        if (v > best.value) best = {V, v};
    }
    return best;
}
} // namespace

double powerRequired(const Aircraft& ac, double V, double h, double m, double n) {
    const double qS = 0.5 * isa::density(h) * V * V * ac.geometry.S;
    const double CL = n * m * isa::g0 / qS;
    return qS * (ac.aero.CD0 + ac.aero.k * CL * CL) * V;
}

double powerAvailable(const Aircraft& ac, double h) { return ac.propulsion.eta_p * ac.maxPower(h); }

double rateOfClimb(const Aircraft& ac, double V, double h, double m) {
    return (powerAvailable(ac, h) - powerRequired(ac, V, h, m)) / (m * isa::g0);
}

SpeedPoint maxRateOfClimb(const Aircraft& ac, double h, double m) {
    return scanMax(ac, h, m, [&](double V) { return rateOfClimb(ac, V, h, m); });
}

SpeedPoint minPowerSpeed(const Aircraft& ac, double h, double m) {
    SpeedPoint p = scanMax(ac, h, m, [&](double V) { return -powerRequired(ac, V, h, m); });
    p.value = -p.value;
    return p;
}

double maxLevelSpeed(const Aircraft& ac, double h, double m) {
    const double V0 = ac.stallSpeed(h, m), V1 = ac.envelope.V_ne;
    for (int i = kSamples; i >= 0; --i) {
        const double V = V0 + (V1 - V0) * i / kSamples;
        if (rateOfClimb(ac, V, h, m) >= 0.0) return V;
    }
    return 0.0;
}

double ceiling(const Aircraft& ac, double m, double rocLimit) {
    double lo = 0.0, hi = isa::hMax;
    if (maxRateOfClimb(ac, lo, m).value < rocLimit) return 0.0;
    if (maxRateOfClimb(ac, hi, m).value >= rocLimit) return hi;
    for (int it = 0; it < 60; ++it) {
        const double mid = 0.5 * (lo + hi);
        (maxRateOfClimb(ac, mid, m).value >= rocLimit ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
}

Turn levelTurn(const Aircraft& ac, double V, double h, double m, double phi) {
    Turn t;
    t.n = 1.0 / std::cos(phi);
    t.radius = V * V / (isa::g0 * std::tan(phi));
    t.rate = isa::g0 * std::tan(phi) / V;
    t.powerRequired = powerRequired(ac, V, h, m, t.n);
    const double CL = t.n * m * isa::g0 / (0.5 * isa::density(h) * V * V * ac.geometry.S);
    t.feasible = CL <= ac.clMax() && t.n <= ac.envelope.n_max && t.powerRequired <= powerAvailable(ac, h);
    return t;
}

} // namespace ofc::perf
