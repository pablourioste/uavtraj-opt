#include "ofc/aircraft.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

#include "ofc/atmosphere.hpp"
#include "ofc/types.hpp"

namespace ofc {

namespace {

using nlohmann::json;

// Required field: a clear error message beats a bare json::out_of_range.
double req(const json& j, const char* group, const char* key) {
    if (!j.contains(group)) throw std::invalid_argument(std::string("aircraft: missing group '") + group + "'");
    const json& g = j.at(group);
    if (!g.contains(key) || !g.at(key).is_number())
        throw std::invalid_argument(std::string("aircraft: missing number '") + group + "." + key + "'");
    return g.at(key).get<double>();
}

double opt(const json& j, const char* group, const char* key, double fallback) {
    if (!j.contains(group) || !j.at(group).contains(key)) return fallback;
    return j.at(group).at(key).get<double>();
}

double reqDeg(const json& j, const char* group, const char* key) { return deg2rad(req(j, group, key)); }

void requirePositive(double v, const char* what) {
    if (!(v > 0.0)) throw std::invalid_argument(std::string("aircraft: '") + what + "' must be > 0");
}

} // namespace

Aircraft Aircraft::fromJson(const json& j) {
    Aircraft ac;
    ac.name = j.value("name", std::string("unnamed"));
    ac.source = j.value("source", std::string());

    ac.mass.m0 = req(j, "mass", "m0_kg");
    ac.mass.m_empty = req(j, "mass", "m_empty_kg");
    ac.mass.fuel_max = req(j, "mass", "fuel_max_kg");

    ac.geometry.S = req(j, "geometry", "S_m2");
    ac.geometry.b = req(j, "geometry", "b_m");
    ac.geometry.c = req(j, "geometry", "c_mac_m");

    ac.inertia.Ixx = req(j, "inertia", "Ixx");
    ac.inertia.Iyy = req(j, "inertia", "Iyy");
    ac.inertia.Izz = req(j, "inertia", "Izz");
    ac.inertia.Ixz = opt(j, "inertia", "Ixz", 0.0);

    ac.propulsion.P_max = req(j, "propulsion", "P_max_W");
    ac.propulsion.eta_p = req(j, "propulsion", "eta_p");
    ac.propulsion.sfc = req(j, "propulsion", "sfc_kg_per_kWh");
    ac.propulsion.thrust_z_offset = opt(j, "propulsion", "thrust_z_offset_m", 0.0);

    Envelope& e = ac.envelope;
    e.V_ne = req(j, "envelope", "V_ne");
    e.V_cruise = req(j, "envelope", "V_cruise");
    e.h_min = req(j, "envelope", "h_min_m");
    e.n_min = req(j, "envelope", "n_min");
    e.n_max = req(j, "envelope", "n_max");
    e.phi_max = reqDeg(j, "envelope", "phi_max_deg");
    e.theta_min = reqDeg(j, "envelope", "theta_min_deg");
    e.theta_max = reqDeg(j, "envelope", "theta_max_deg");
    e.gamma_min = reqDeg(j, "envelope", "gamma_min_deg");
    e.gamma_max = reqDeg(j, "envelope", "gamma_max_deg");
    e.alpha_min = reqDeg(j, "envelope", "alpha_min_deg");
    e.alpha_max = reqDeg(j, "envelope", "alpha_max_deg");

    Actuators& a = ac.actuators;
    a.de_max = reqDeg(j, "actuators", "de_max_deg");
    a.da_max = reqDeg(j, "actuators", "da_max_deg");
    a.dr_max = reqDeg(j, "actuators", "dr_max_deg");
    a.p_max = reqDeg(j, "actuators", "p_max_dps");
    a.q_max = reqDeg(j, "actuators", "q_max_dps");
    a.r_max = reqDeg(j, "actuators", "r_max_dps");

    if (!j.contains("aero")) throw std::invalid_argument("aircraft: missing group 'aero'");
    const json& aero = j.at("aero");
    AeroCoefficients& c = ac.aero;
    c.CL0 = req(aero, "lift", "CL0");
    c.CLa = req(aero, "lift", "CLa");
    c.CLq = opt(aero, "lift", "CLq", 0.0);
    c.CLde = req(aero, "lift", "CLde");

    c.CD0 = req(aero, "drag", "CD0");
    if (aero.at("drag").contains("k")) {
        c.k = req(aero, "drag", "k");
    } else {
        // k from the Oswald factor when not given explicitly.
        const double oswald = req(aero, "drag", "oswald_e");
        c.k = 1.0 / (kPi * oswald * ac.geometry.aspectRatio());
    }

    c.CY0 = opt(aero, "side", "CY0", 0.0);
    c.CYb = req(aero, "side", "CYb");
    c.CYp = opt(aero, "side", "CYp", 0.0);
    c.CYr = opt(aero, "side", "CYr", 0.0);
    c.CYda = opt(aero, "side", "CYda", 0.0);
    c.CYdr = req(aero, "side", "CYdr");

    c.Cl0 = opt(aero, "roll", "Cl0", 0.0);
    c.Clb = req(aero, "roll", "Clb");
    c.Clp = req(aero, "roll", "Clp");
    c.Clr = req(aero, "roll", "Clr");
    c.Clda = req(aero, "roll", "Clda");
    c.Cldr = req(aero, "roll", "Cldr");

    c.Cm0 = req(aero, "pitch", "Cm0");
    c.Cma = req(aero, "pitch", "Cma");
    c.Cmq = req(aero, "pitch", "Cmq");
    c.Cmde = req(aero, "pitch", "Cmde");

    c.Cn0 = opt(aero, "yaw", "Cn0", 0.0);
    c.Cnb = req(aero, "yaw", "Cnb");
    c.Cnp = req(aero, "yaw", "Cnp");
    c.Cnr = req(aero, "yaw", "Cnr");
    c.Cnda = req(aero, "yaw", "Cnda");
    c.Cndr = req(aero, "yaw", "Cndr");

    ac.validate();
    return ac;
}

Aircraft Aircraft::fromFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open aircraft file: " + path);
    return fromJson(nlohmann::json::parse(in));
}

double Aircraft::clMax() const { return aero.CL0 + aero.CLa * envelope.alpha_max; }

double Aircraft::stallSpeed(double h, double m) const {
    return std::sqrt(2.0 * m * isa::g0 / (isa::density(h) * geometry.S * clMax()));
}

double Aircraft::maxPower(double h) const {
    // Gagg-Ferrar lapse for a naturally aspirated piston engine: P/P0 = 1.132 sigma - 0.132.
    return propulsion.P_max * std::max(0.0, 1.132 * isa::densityRatio(h) - 0.132);
}

double Aircraft::maxThrust(double V, double h) const {
    // Constant-power propeller; below stall speed the thrust is capped to keep it finite.
    const double Vref = std::max(V, stallSpeed(h, mass.m0));
    return propulsion.eta_p * maxPower(h) / Vref;
}

double Aircraft::minTurnRadius(double V, double phi) const { return V * V / (isa::g0 * std::tan(phi)); }

double Aircraft::bestGlideCL() const { return std::sqrt(aero.CD0 / aero.k); }

double Aircraft::bestGlideSpeed(double h, double m) const {
    return std::sqrt(2.0 * m * isa::g0 / (isa::density(h) * geometry.S * bestGlideCL()));
}

double Aircraft::maxLiftToDrag() const { return 0.5 / std::sqrt(aero.CD0 * aero.k); }

void Aircraft::validate() const {
    requirePositive(mass.m0, "mass.m0_kg");
    requirePositive(mass.m_empty, "mass.m_empty_kg");
    if (mass.m_empty + mass.fuel_max > mass.m0 + 1e-9)
        throw std::invalid_argument("aircraft: m_empty + fuel_max exceeds m0");
    requirePositive(geometry.S, "geometry.S_m2");
    requirePositive(geometry.b, "geometry.b_m");
    requirePositive(geometry.c, "geometry.c_mac_m");
    requirePositive(inertia.Ixx, "inertia.Ixx");
    requirePositive(inertia.Iyy, "inertia.Iyy");
    requirePositive(inertia.Izz, "inertia.Izz");
    if (inertia.Ixx * inertia.Izz - inertia.Ixz * inertia.Ixz <= 0.0)
        throw std::invalid_argument("aircraft: inertia tensor is not positive definite");
    requirePositive(propulsion.P_max, "propulsion.P_max_W");
    if (propulsion.eta_p <= 0.0 || propulsion.eta_p > 1.0)
        throw std::invalid_argument("aircraft: propulsion.eta_p must be in (0, 1]");
    if (envelope.alpha_min >= envelope.alpha_max)
        throw std::invalid_argument("aircraft: alpha_min_deg must be < alpha_max_deg");
    if (envelope.n_min >= 1.0 || envelope.n_max <= 1.0)
        throw std::invalid_argument("aircraft: load-factor limits must bracket 1 g");
    requirePositive(aero.CLa, "aero.lift.CLa");
    requirePositive(aero.k, "aero.drag.k");
    if (clMax() <= 0.0) throw std::invalid_argument("aircraft: CL at alpha_max must be > 0");
    if (aero.Cma >= 0.0) throw std::invalid_argument("aircraft: Cma >= 0 (statically unstable in pitch)");
    if (stallSpeed(0.0, mass.m0) >= envelope.V_ne) throw std::invalid_argument("aircraft: stall speed exceeds V_ne");
}

} // namespace ofc
