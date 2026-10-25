#pragma once
// Rigid-body (FC) optimal-control problem: trapezoidal collocation of the
// 6-DoF model with 13 states and 4 controls (thrust and three surfaces).
//
//   defects   model Jacobians A_k, B_k by central differences of fc::rhs,
//             assembled exactly as in the UF problem (block-banded)
//   boundary  x_0 fixed (the whole state); x_{N-1}: position and yaw
//   path      airspeed (stall, V_NE), power, incidence, sideslip, load
//             factor, altitude, bank, pitch, body rates, no-fly zones
//   boxes     actuator deflections at their real limits; states within
//             sanity ranges (pitch kept away from the Euler singularity)
//   cost      (1 - alpha) t_f / t_ref + alpha (m_0 - m_{N-1}) / m_ref
//             + [beta_c sum ((u_{k+1} - u_k) / s_u)^2
//                + beta_s sum ((x_{k+1} - x_k) / s_x)^2 over phi, theta] / (N-1)

#include <vector>

#include "ofc/aircraft.hpp"
#include "ofc/environment.hpp"
#include "ofc/ocp_problem.hpp"
#include "ofc/types.hpp"

namespace ofc::ocp {

struct FcMission {
    Vec x0;                        // initial state (fc::NX), fixed
    Vec3 end{2000.0, 0.0, -100.0}; // final NED position [m]
    double headingEnd = 0.0;       // final yaw [rad], matched modulo 2 pi
    Vec3 wind{0.0, 0.0, 0.0};      // constant NED wind [m/s]
    double alpha = 0.0;            // 0 = minimum time, 1 = minimum fuel
    double betaControl = 0.0;      // smoothness weight on control increments (T, de, da, dr)
    double betaState = 0.0;        // smoothness weight on attitude increments (phi, theta)
    // Sideslip limit (coordinated flight). The linear model's polar has no
    // sideslip drag, so without this limit the optimum flies sideways and
    // uses the side force as free lift.
    double sideslipMax = 1.0 * kPi / 180.0;
    std::vector<env::NoFlyZone> nfz;
    double hLow = 0.0, hHigh = 0.0; // altitude band; zones outside it are ignored
};

class FcProblem : public CollocationProblem {
public:
    FcProblem(const Aircraft& ac, const FcMission& mission, const Trajectory& guess,
              std::vector<WaypointNode> waypoints = {}, Reference ref = {});

    double cost(const Vec& z) const override;
    Vec costGradient(const Vec& z) const override;

    // Collocation defects alone (normalised), e.g. to measure a guess.
    Vec dynamicsDefects(const Vec& z) const { return defects(z); }

protected:
    Vec defects(const Vec& z) const override;
    std::vector<Triplet> defectJacobian(const Vec& z) const override;
    Vec pathAt(const Vec& x, const Vec& u) const override;

private:
    void modelJacobian(const Vec& x, const Vec& u, Vec& A, Vec& B) const;

    const Aircraft& ac_;
    FcMission mission_;
    Layout layout_;
    Vec sx_, su_;
    double tfRef_ = 1.0, fuelRef_ = 1.0;
    std::vector<bool> patternA_, patternB_, nfzActive_;
};

} // namespace ofc::ocp
