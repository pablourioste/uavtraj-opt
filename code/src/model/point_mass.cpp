#include "ofc/point_mass.hpp"

#include <cmath>

#include "ofc/atmosphere.hpp"

namespace ofc::uf {

double fuelFlowCoefficient(const Aircraft& ac) {
    // sfc [kg/kWh] * P [W] / 3.6e6 = [kg/s],  P = T V / eta_p
    return ac.propulsion.sfc / (3.6e6 * ac.propulsion.eta_p);
}

Forces aerodynamicForces(const Aircraft& ac, const Vec& x, const Vec& u) {
    Forces f;
    f.rho = isa::density(-x[Z]);
    f.qbar = 0.5 * f.rho * x[V] * x[V];
    const double CD = ac.aero.CD0 + ac.aero.k * u[CL] * u[CL];
    f.L = f.qbar * ac.geometry.S * u[CL];
    f.D = f.qbar * ac.geometry.S * CD;
    return f;
}

Vec rhs(const Aircraft& ac, const Vec& x, const Vec& u, const Vec3& wind) {
    const Forces F = aerodynamicForces(ac, x, u);
    const double g = isa::g0;
    const double Va = x[V], m = x[MASS];
    const double cg = std::cos(x[GAMMA]), sg = std::sin(x[GAMMA]);
    const double cc = std::cos(x[CHI]), sc = std::sin(x[CHI]);
    const double cm = std::cos(x[MU]), sm = std::sin(x[MU]);

    Vec f(NX);
    f[X] = Va * cg * cc + wind[0];
    f[Y] = Va * cg * sc + wind[1];
    f[Z] = -Va * sg + wind[2];
    f[V] = (u[THRUST] - F.D) / m - g * sg;
    f[GAMMA] = (F.L * cm) / (m * Va) - g * cg / Va;
    f[CHI] = (F.L * sm) / (m * Va * cg);
    f[MU] = u[BANK_RATE];
    f[MASS] = -fuelFlowCoefficient(ac) * u[THRUST] * Va;
    return f;
}

void jacobian(const Aircraft& ac, const Vec& x, const Vec& u, Vec& A, Vec& B) {
    A.assign(NX * NX, 0.0);
    B.assign(NX * NU, 0.0);
    auto a = [&A](int i, int j) -> double& { return A[i * NX + j]; };
    auto b = [&B](int i, int j) -> double& { return B[i * NU + j]; };

    const Forces F = aerodynamicForces(ac, x, u);
    const double g = isa::g0, S = ac.geometry.S;
    const double Va = x[V], m = x[MASS], CL_ = u[CL], T = u[THRUST];
    const double cg = std::cos(x[GAMMA]), sg = std::sin(x[GAMMA]);
    const double cc = std::cos(x[CHI]), sc = std::sin(x[CHI]);
    const double cm = std::cos(x[MU]), sm = std::sin(x[MU]);

    // Partial derivatives of lift and drag. Altitude h = -z, so d rho/dz = -d rho/dh.
    const double CD = ac.aero.CD0 + ac.aero.k * CL_ * CL_;
    const double rho_z = -isa::densityGradient(-x[Z]);
    const double L_z = 0.5 * rho_z * Va * Va * S * CL_;
    const double D_z = 0.5 * rho_z * Va * Va * S * CD;
    const double L_V = 2.0 * F.L / Va;
    const double D_V = 2.0 * F.D / Va;
    const double L_CL = F.qbar * S;
    const double D_CL = F.qbar * S * 2.0 * ac.aero.k * CL_;

    // Position kinematics.
    a(X, V) = cg * cc;
    a(X, GAMMA) = -Va * sg * cc;
    a(X, CHI) = -Va * cg * sc;
    a(Y, V) = cg * sc;
    a(Y, GAMMA) = -Va * sg * sc;
    a(Y, CHI) = Va * cg * cc;
    a(Z, V) = -sg;
    a(Z, GAMMA) = -Va * cg;

    // Speed.
    a(V, Z) = -D_z / m;
    a(V, V) = -D_V / m;
    a(V, GAMMA) = -g * cg;
    a(V, MASS) = -(T - F.D) / (m * m);
    b(V, THRUST) = 1.0 / m;
    b(V, CL) = -D_CL / m;

    // Flight-path angle: gamma_dot = L cos(mu)/(m V) - g cos(gamma)/V.
    a(GAMMA, Z) = L_z * cm / (m * Va);
    a(GAMMA, V) = L_V * cm / (m * Va) - F.L * cm / (m * Va * Va) + g * cg / (Va * Va);
    a(GAMMA, GAMMA) = g * sg / Va;
    a(GAMMA, MU) = -F.L * sm / (m * Va);
    a(GAMMA, MASS) = -F.L * cm / (m * m * Va);
    b(GAMMA, CL) = L_CL * cm / (m * Va);

    // Heading: chi_dot = L sin(mu)/(m V cos(gamma)).
    const double den = m * Va * cg;
    a(CHI, Z) = L_z * sm / den;
    a(CHI, V) = L_V * sm / den - F.L * sm / (den * Va);
    a(CHI, GAMMA) = F.L * sm * sg / (m * Va * cg * cg);
    a(CHI, MU) = F.L * cm / den;
    a(CHI, MASS) = -F.L * sm / (den * m);
    b(CHI, CL) = L_CL * sm / den;

    // Bank angle and mass.
    b(MU, BANK_RATE) = 1.0;
    const double cf = fuelFlowCoefficient(ac);
    a(MASS, V) = -cf * T;
    b(MASS, THRUST) = -cf * Va;
}

double loadFactor(const Aircraft& ac, const Vec& x, const Vec& u) {
    return aerodynamicForces(ac, x, u).L / (x[MASS] * isa::g0);
}

} // namespace ofc::uf
