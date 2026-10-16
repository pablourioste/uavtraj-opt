// Atmosphere, aircraft data and dynamic models.

#include <cmath>
#include <fstream>

#include "check.hpp"
#include "ofc/aircraft.hpp"
#include "ofc/atmosphere.hpp"
#include "ofc/numerics.hpp"
#include "ofc/point_mass.hpp"
#include "ofc/rigid_body.hpp"

using namespace ofc;

namespace {
const Aircraft& generic() {
    static const Aircraft ac = Aircraft::fromFile("data/aircraft/generic_uav.json");
    return ac;
}
} // namespace

TEST(isa_reference_values) {
    CHECK_NEAR(isa::density(0.0), 1.2250, 1e-4);
    CHECK_NEAR(isa::density(1000.0), 1.1117, 1e-3);
    CHECK_NEAR(isa::density(5000.0), 0.7364, 1e-3);
    CHECK_NEAR(isa::temperature(11000.0), 216.65, 1e-9);
}

TEST(isa_density_gradient_matches_fd) {
    for (double h : {0.0, 800.0, 3000.0, 9000.0}) {
        const double d = 1e-2;
        const double fd = (isa::density(h + d) - isa::density(h - d)) / (2 * d);
        CHECK_NEAR(isa::densityGradient(h), fd, 1e-9);
    }
}

TEST(aircraft_loads_and_derives) {
    const Aircraft& ac = generic();
    CHECK(ac.name == "generic_uav");
    CHECK_NEAR(ac.geometry.aspectRatio(), 2.8956 * 2.8956 / 0.55, 1e-12);
    // At the stall speed, CL_max lift balances the weight.
    const double Vs = ac.stallSpeed(0.0, ac.mass.m0);
    const double L = 0.5 * isa::density(0.0) * Vs * Vs * ac.geometry.S * ac.clMax();
    CHECK_NEAR(L, ac.mass.m0 * isa::g0, 1e-9);
    CHECK(Vs < ac.envelope.V_cruise);
    CHECK(ac.maxLiftToDrag() > 5.0);
}

TEST(aircraft_rejects_bad_data) {
    nlohmann::json j = nlohmann::json::parse(R"({"name":"x"})");
    CHECK_THROWS(Aircraft::fromJson(j));

    std::ifstream in("data/aircraft/generic_uav.json");
    nlohmann::json good = nlohmann::json::parse(in);
    nlohmann::json heavy = good;
    heavy["mass"]["fuel_max_kg"] = 50.0; // empty + fuel > take-off mass
    CHECK_THROWS(Aircraft::fromJson(heavy));
    nlohmann::json unstable = good;
    unstable["aero"]["pitch"]["Cma"] = 0.5; // statically unstable
    CHECK_THROWS(Aircraft::fromJson(unstable));
}

TEST(uf_analytic_jacobian_matches_central_fd) {
    const Aircraft& ac = generic();
    const std::vector<Vec> xs{{0, 0, -300, 25, 0, 0, 0, 11},
                              {120, -40, -850, 31, 0.12, 1.1, 0.35, 10.2},
                              {-60, 300, -60, 18, -0.2, -2.5, -0.6, 9.1}};
    const std::vector<Vec> us{{9.0, 0.5, 0.0}, {22.0, 0.9, 0.2}, {3.0, 0.3, -0.4}};
    for (std::size_t c = 0; c < xs.size(); ++c) {
        Vec A, B;
        uf::jacobian(ac, xs[c], us[c], A, B);
        const Vec Afd = num::fdJacobian([&](const Vec& x) { return uf::rhs(ac, x, us[c]); }, xs[c]);
        const Vec Bfd = num::fdJacobian([&](const Vec& u) { return uf::rhs(ac, xs[c], u); }, us[c]);
        CHECK(num::maxRelativeError(A, Afd) < 1e-7);
        CHECK(num::maxRelativeError(B, Bfd) < 1e-7);
    }
}

TEST(uf_coordinated_turn_rate) {
    // With L cos(mu) = m g (level turn), chi_dot must equal g tan(mu) / V.
    const Aircraft& ac = generic();
    const double V = 25.0, mu = 0.5, h = 300.0, m = ac.mass.m0;
    const double qS = 0.5 * isa::density(h) * V * V * ac.geometry.S;
    const Vec x{0, 0, -h, V, 0, 0, mu, m};
    const Vec u{10.0, m * isa::g0 / (qS * std::cos(mu)), 0.0};
    const Vec f = uf::rhs(ac, x, u);
    CHECK_NEAR(f[uf::GAMMA], 0.0, 1e-12);
    CHECK_NEAR(f[uf::CHI], isa::g0 * std::tan(mu) / V, 1e-12);
}

TEST(rotation_is_orthonormal) {
    const Mat3 R = fc::bodyToNed(0.3, -0.2, 2.1);
    const Mat3 Rt = transpose(R);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double s = 0;
            for (int k = 0; k < 3; ++k) s += R[i][k] * Rt[k][j];
            CHECK_NEAR(s, i == j ? 1.0 : 0.0, 1e-14);
        }
}

TEST(fc_wind_only_changes_air_data) {
    // A steady wind leaves the aircraft unchanged if the inertial velocity
    // carries it along: same air-relative velocity -> same accelerations.
    const Aircraft& ac = generic();
    const Vec3 wind{3.0, -2.0, 0.5};
    Vec x(fc::NX, 0.0);
    x[fc::ZD] = -300;
    x[fc::U] = 25;
    x[fc::W] = 1.5;
    x[fc::THETA] = 0.06;
    x[fc::MASS] = ac.mass.m0;
    Vec xw = x;
    const Mat3 Rbn = transpose(fc::bodyToNed(x[fc::PHI], x[fc::THETA], x[fc::PSI]));
    const Vec3 wb = mul(Rbn, wind);
    xw[fc::U] += wb[0];
    xw[fc::V] += wb[1];
    xw[fc::W] += wb[2];
    const Vec u{10, -0.1, 0, 0};
    const Vec f0 = fc::rhs(ac, x, u), f1 = fc::rhs(ac, xw, u, wind);
    for (int i : {fc::U, fc::V, fc::W, fc::P, fc::Q, fc::R}) CHECK_NEAR(f0[i], f1[i], 1e-12);
    CHECK_NEAR(f1[fc::XN] - f0[fc::XN], wind[0], 1e-12);
}
