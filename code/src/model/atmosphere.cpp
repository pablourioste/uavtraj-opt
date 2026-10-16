#include "ofc/atmosphere.hpp"

#include <algorithm>
#include <cmath>

namespace ofc::isa {

namespace {
constexpr double kExponent = g0 / (R * lapse); // ~5.256

double clampAltitude(double h) { return std::clamp(h, hMin, hMax); }
} // namespace

double temperature(double h) { return T0 - lapse * clampAltitude(h); }

double pressure(double h) { return p0 * std::pow(temperature(h) / T0, kExponent); }

double seaLevelDensity() { return p0 / (R * T0); }

double density(double h) { return pressure(h) / (R * temperature(h)); }

double densityRatio(double h) { return density(h) / seaLevelDensity(); }

double densityGradient(double h) {
    if (h <= hMin || h >= hMax) return 0.0;
    // rho = rho0 * theta^(n-1), theta = T/T0  =>  d rho/dh = rho * (n-1) * (-lapse / T)
    return density(h) * (kExponent - 1.0) * (-lapse / temperature(h));
}

} // namespace ofc::isa
