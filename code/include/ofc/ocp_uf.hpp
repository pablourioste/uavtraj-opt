#pragma once
// Point-mass (UF) optimal-control problem: TrapezoidalUF (defects, analytic
// defect Jacobian, envelope and no-fly-zone rows, Bolza cost) completed with
// variable boxes, boundary conditions and waypoints into an nlp::Problem.
//
//   boundary  x_0 fixed (position, speed, flight-path angle, heading, bank 0,
//             mass m0); x_{N-1}: position and heading
//   boxes     physical sanity ranges that keep every function defined; the
//             real limits are the path rows
//   t_f       free, within [0.3, 3] times the reference time

#include <vector>

#include "ofc/aircraft.hpp"
#include "ofc/ocp_problem.hpp"
#include "ofc/transcription.hpp"

namespace ofc::ocp {

class UfProblem : public CollocationProblem {
public:
    // `guess` fixes N and the initial point. The final heading is taken modulo
    // 2 pi, closest to the guess's final heading (headings are unwrapped).
    UfProblem(const Aircraft& ac, const Mission& mission, const Trajectory& guess,
              std::vector<WaypointNode> waypoints = {}, Reference ref = {});

    double cost(const Vec& z) const override { return tr_.cost(z); }
    Vec costGradient(const Vec& z) const override { return tr_.costGradient(z); }

    const TrapezoidalUF& transcription() const { return tr_; }

protected:
    Vec defects(const Vec& z) const override { return tr_.defects(z); }
    std::vector<Triplet> defectJacobian(const Vec& z) const override { return tr_.defectJacobian(z); }
    Vec pathAt(const Vec& x, const Vec& u) const override { return tr_.pathAt(x, u); }

private:
    TrapezoidalUF tr_;
};

// Scales for a mission of the given path length and flight time.
UfScales ufScales(const Aircraft& ac, double pathLength, double flightTime);

} // namespace ofc::ocp
