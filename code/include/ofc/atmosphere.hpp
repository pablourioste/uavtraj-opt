#pragma once
// International Standard Atmosphere, troposphere only (0 - 11 km).
// Altitudes outside the band are clamped, so density stays smooth and positive.

namespace ofc::isa {

constexpr double g0 = 9.80665;   // gravity [m/s^2]
constexpr double R = 287.05287;  // gas constant of dry air [J/(kg K)]
constexpr double T0 = 288.15;    // sea-level temperature [K]
constexpr double p0 = 101325.0;  // sea-level pressure [Pa]
constexpr double lapse = 0.0065; // temperature lapse rate [K/m]
constexpr double hMin = -500.0;  // validity band [m]
constexpr double hMax = 11000.0;

double temperature(double h);     // [K]
double pressure(double h);        // [Pa]
double density(double h);         // [kg/m^3]
double densityRatio(double h);    // rho / rho0 [-]
double densityGradient(double h); // d rho / d h [kg/m^4], zero outside the band
double seaLevelDensity();         // p0 / (R T0) = 1.225 kg/m^3

} // namespace ofc::isa
