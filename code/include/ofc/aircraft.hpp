#pragma once
// Generic fixed-wing aircraft: design parameters grouped by discipline.
// Values are read from a JSON file (see data/aircraft/README.md). Angles are
// given in degrees in the file and stored in radians here.

#include <string>

#include <nlohmann/json.hpp>

namespace ofc {

struct MassData {
    double m0 = 0.0;       // take-off mass [kg]
    double m_empty = 0.0;  // empty mass [kg]
    double fuel_max = 0.0; // usable fuel [kg]
};

struct Geometry {
    double S = 0.0; // wing reference area [m^2]
    double b = 0.0; // span [m]
    double c = 0.0; // mean aerodynamic chord [m]
    double aspectRatio() const { return b * b / S; }
};

struct Inertia {
    double Ixx = 0.0, Iyy = 0.0, Izz = 0.0, Ixz = 0.0; // body axes [kg m^2]
};

struct Propulsion {
    double P_max = 0.0;           // sea-level shaft power [W]
    double eta_p = 0.0;           // propeller efficiency [-]
    double sfc = 0.0;             // specific fuel consumption [kg/kWh]
    double thrust_z_offset = 0.0; // thrust line below (+) / above (-) the CG [m]
};

struct Envelope {
    double V_ne = 0.0, V_cruise = 0.0; // [m/s]
    double h_min = 0.0;                // [m]
    double n_min = 0.0, n_max = 0.0;   // load factor [g]
    double phi_max = 0.0;              // [rad]
    double theta_min = 0.0, theta_max = 0.0;
    double gamma_min = 0.0, gamma_max = 0.0;
    double alpha_min = 0.0, alpha_max = 0.0;
};

struct Actuators {
    double de_max = 0.0, da_max = 0.0, dr_max = 0.0; // deflection limits [rad]
    double p_max = 0.0, q_max = 0.0, r_max = 0.0;    // body-rate limits [rad/s]
};

// Linear stability-derivative model. Rate derivatives use the usual
// non-dimensional rates p b/(2V), q c/(2V), r b/(2V).
struct AeroCoefficients {
    double CL0 = 0.0, CLa = 0.0, CLq = 0.0, CLde = 0.0;
    double CD0 = 0.0, k = 0.0; // parabolic polar CD = CD0 + k CL^2
    double CY0 = 0.0, CYb = 0.0, CYp = 0.0, CYr = 0.0, CYda = 0.0, CYdr = 0.0;
    double Cl0 = 0.0, Clb = 0.0, Clp = 0.0, Clr = 0.0, Clda = 0.0, Cldr = 0.0;
    double Cm0 = 0.0, Cma = 0.0, Cmq = 0.0, Cmde = 0.0;
    double Cn0 = 0.0, Cnb = 0.0, Cnp = 0.0, Cnr = 0.0, Cnda = 0.0, Cndr = 0.0;
};

class Aircraft {
public:
    static Aircraft fromJson(const nlohmann::json& j);
    static Aircraft fromFile(const std::string& path);

    std::string name;
    std::string source;
    MassData mass;
    Geometry geometry;
    Inertia inertia;
    Propulsion propulsion;
    Envelope envelope;
    Actuators actuators;
    AeroCoefficients aero;

    // ── Derived performance (never read from the file, always computed) ──
    double clMax() const;                             // CL at alpha_max, de = 0
    double stallSpeed(double h, double m) const;      // 1 g, wings level
    double maxPower(double h) const;                  // P_max (1.132 sigma - 0.132), Gagg-Ferrar
    double maxThrust(double V, double h) const;       // eta_p P(h) / max(V, V_stall)
    double minTurnRadius(double V, double phi) const; // coordinated turn
    double bestGlideCL() const;                       // sqrt(CD0 / k)
    double bestGlideSpeed(double h, double m) const;  // speed at max L/D
    double maxLiftToDrag() const;

    // Throws std::invalid_argument when the data are physically inconsistent.
    void validate() const;
};

} // namespace ofc
