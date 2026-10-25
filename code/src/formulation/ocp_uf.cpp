#include "ofc/ocp_uf.hpp"

#include <algorithm>
#include <cmath>

#include "ofc/point_mass.hpp"

namespace ofc::ocp {

using namespace uf;

namespace {

double pathLengthOf(const Trajectory& t) {
    double L = 0.0;
    for (int k = 1; k < t.nodes(); ++k) {
        const double dx = t.X[k][X] - t.X[k - 1][X], dy = t.X[k][Y] - t.X[k - 1][Y], dz = t.X[k][Z] - t.X[k - 1][Z];
        L += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return L;
}

Mission withUnwrappedHeading(Mission m, const Trajectory& guess) {
    m.headingEnd = unwrapNear(m.headingEnd, guess.X.back()[CHI]);
    return m;
}

Reference resolved(Reference ref, const Trajectory& guess) {
    if (ref.length <= 0.0) ref.length = pathLengthOf(guess);
    if (ref.time <= 0.0) ref.time = guess.tf;
    return ref;
}

} // namespace

UfScales ufScales(const Aircraft& ac, double pathLength, double flightTime) {
    UfScales s = UfScales::fromAircraft(ac, pathLength);
    s.tf = std::max(flightTime, 1.0);
    return s;
}

UfProblem::UfProblem(const Aircraft& ac, const Mission& mission, const Trajectory& guess,
                     std::vector<WaypointNode> waypoints, Reference ref)
    : tr_(ac, withUnwrappedHeading(mission, guess), guess.nodes(),
          ufScales(ac, resolved(ref, guess).length, resolved(ref, guess).time)) {
    const Mission& m = tr_.mission();
    const UfScales& s = tr_.scales();
    const Envelope& e = ac.envelope;

    ModelInfo info;
    info.stateNames = {"x", "y", "z", "V", "gamma", "chi", "mu", "m"};
    info.controlNames = {"T", "CL", "p"};
    info.pathNames = tr_.pathConstraintNames();
    info.stateScale = s.x;
    info.controlScale = s.u;
    info.tfScale = s.tf;
    info.stateBox.resize(NX);
    info.stateBox[V] = {0.3 * e.V_cruise, 1.3 * e.V_ne};
    info.stateBox[GAMMA] = {-kPi / 4.0, kPi / 4.0};
    info.stateBox[MU] = {-deg2rad(80.0), deg2rad(80.0)};
    info.stateBox[MASS] = {ac.mass.m_empty, 1.001 * ac.mass.m0};
    info.controlBox = {
        {0.0, ac.maxThrust(0.0, 0.0)}, {-0.5, 1.5 * ac.clMax()}, {-2.0 * ac.actuators.p_max, 2.0 * ac.actuators.p_max}};
    info.tfBox = {0.3 * s.tf, 3.0 * s.tf};
    info.idxX = X, info.idxY = Y;

    const Vec x0{m.start[0],   m.start[1],     m.start[2], m.speedStart > 0.0 ? m.speedStart : e.V_cruise,
                 m.gammaStart, m.headingStart, 0.0,        ac.mass.m0};
    const Layout& L = tr_.layout();
    std::vector<BoundaryRow> boundary;
    for (int i = 0; i < NX; ++i) boundary.push_back({"x0." + info.stateNames[i], L.idxState(0, i), x0[i], s.x[i]});
    const int K = L.N - 1;
    for (int i = 0; i < 3; ++i) boundary.push_back({"xf." + info.stateNames[i], L.idxState(K, i), m.end[i], s.x[i]});
    boundary.push_back({"xf.chi", L.idxState(K, CHI), m.headingEnd, s.x[CHI]});

    setup(L, std::move(info), std::move(boundary), pack(L, guess), std::move(waypoints));
}

} // namespace ofc::ocp
