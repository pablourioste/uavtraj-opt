#include "ofc/rigid_body.hpp"

#include <algorithm>
#include <cmath>

#include "ofc/atmosphere.hpp"
#include "ofc/point_mass.hpp"

namespace ofc::fc {

Mat3 bodyToNed(double phi, double theta, double psi) {
    const double cf = std::cos(phi), sf = std::sin(phi);
    const double ct = std::cos(theta), st = std::sin(theta);
    const double cp = std::cos(psi), sp = std::sin(psi);
    return {{{ct * cp, sf * st * cp - cf * sp, cf * st * cp + sf * sp},
             {ct * sp, sf * st * sp + cf * cp, cf * st * sp - sf * cp},
             {-st, sf * ct, cf * ct}}};
}

AirData airData(const Vec& x, const Vec3& windNed) {
    const Mat3 Rbn = transpose(bodyToNed(x[PHI], x[THETA], x[PSI]));
    const Vec3 vRel = Vec3{x[U], x[V], x[W]} - mul(Rbn, windNed);
    AirData a;
    a.Va = norm(vRel);
    a.alpha = std::atan2(vRel[2], vRel[0]);
    a.beta = std::asin(std::clamp(vRel[1] / a.Va, -1.0, 1.0));
    return a;
}

AeroCoeffs coefficients(const Aircraft& ac, const AirData& air, const Vec& x, const Vec& u) {
    const AeroCoefficients& k = ac.aero;
    const double b = ac.geometry.b, c = ac.geometry.c;
    const double ph = x[P] * b / (2.0 * air.Va);
    const double qh = x[Q] * c / (2.0 * air.Va);
    const double rh = x[R] * b / (2.0 * air.Va);
    const double de = u[ELEVATOR], da = u[AILERON], dr = u[RUDDER];

    AeroCoeffs C;
    C.CL = k.CL0 + k.CLa * air.alpha + k.CLq * qh + k.CLde * de;
    C.CD = k.CD0 + k.k * C.CL * C.CL;
    C.CY = k.CY0 + k.CYb * air.beta + k.CYp * ph + k.CYr * rh + k.CYda * da + k.CYdr * dr;
    C.Cl = k.Cl0 + k.Clb * air.beta + k.Clp * ph + k.Clr * rh + k.Clda * da + k.Cldr * dr;
    C.Cm = k.Cm0 + k.Cma * air.alpha + k.Cmq * qh + k.Cmde * de;
    C.Cn = k.Cn0 + k.Cnb * air.beta + k.Cnp * ph + k.Cnr * rh + k.Cnda * da + k.Cndr * dr;
    return C;
}

Vec rhs(const Aircraft& ac, const Vec& x, const Vec& u, const Vec3& windNed) {
    const double g = isa::g0, m = x[MASS];
    const AirData air = airData(x, windNed);
    const AeroCoeffs C = coefficients(ac, air, x, u);
    const double qS = 0.5 * isa::density(-x[ZD]) * air.Va * air.Va * ac.geometry.S;

    // Wind-axis forces [-D, Y, -L] rotated to body axes.
    const double D = qS * C.CD, Y = qS * C.CY, L = qS * C.CL;
    const double ca = std::cos(air.alpha), sa = std::sin(air.alpha);
    const double cb = std::cos(air.beta), sb = std::sin(air.beta);
    const Vec3 Faero{-D * ca * cb - Y * ca * sb + L * sa, -D * sb + Y * cb, -D * sa * cb - Y * sa * sb - L * ca};

    const double cf = std::cos(x[PHI]), sf = std::sin(x[PHI]);
    const double ct = std::cos(x[THETA]), st = std::sin(x[THETA]);
    const Vec3 Fgrav{-m * g * st, m * g * sf * ct, m * g * cf * ct};
    const Vec3 Fthrust{u[THRUST], 0.0, 0.0};
    const Vec3 F = Faero + Fgrav + Fthrust;

    // Moments: aerodynamic (body axes) + thrust line offset r x T = (0, zT T, 0).
    const Vec3 M{qS * ac.geometry.b * C.Cl, qS * ac.geometry.c * C.Cm + ac.propulsion.thrust_z_offset * u[THRUST],
                 qS * ac.geometry.b * C.Cn};

    const Vec3 vb{x[U], x[V], x[W]};
    const Vec3 w{x[P], x[Q], x[R]};
    const Inertia& I = ac.inertia;
    const Vec3 Iw{I.Ixx * w[0] - I.Ixz * w[2], I.Iyy * w[1], -I.Ixz * w[0] + I.Izz * w[2]};
    const Vec3 H = M - cross(w, Iw);
    const double Gamma = I.Ixx * I.Izz - I.Ixz * I.Ixz;

    const Vec3 vdot = (1.0 / m) * F - cross(w, vb);
    const Vec3 pos = mul(bodyToNed(x[PHI], x[THETA], x[PSI]), vb);

    Vec f(NX);
    f[XN] = pos[0];
    f[YE] = pos[1];
    f[ZD] = pos[2];
    f[U] = vdot[0];
    f[V] = vdot[1];
    f[W] = vdot[2];
    // Euler-angle kinematics (singular at theta = +-90 deg, outside the envelope).
    f[PHI] = w[0] + std::tan(x[THETA]) * (w[1] * sf + w[2] * cf);
    f[THETA] = w[1] * cf - w[2] * sf;
    f[PSI] = (w[1] * sf + w[2] * cf) / ct;
    f[P] = (I.Izz * H[0] + I.Ixz * H[2]) / Gamma;
    f[Q] = H[1] / I.Iyy;
    f[R] = (I.Ixz * H[0] + I.Ixx * H[2]) / Gamma;
    f[MASS] = -uf::fuelFlowCoefficient(ac) * u[THRUST] * air.Va;
    return f;
}

} // namespace ofc::fc
