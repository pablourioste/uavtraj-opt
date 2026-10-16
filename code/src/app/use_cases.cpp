#include "ofc/use_cases.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

#include "ofc/aircraft.hpp"
#include "ofc/atmosphere.hpp"
#include "ofc/io.hpp"
#include "ofc/numerics.hpp"
#include "ofc/performance.hpp"
#include "ofc/planner.hpp"
#include "ofc/point_mass.hpp"
#include "ofc/rigid_body.hpp"
#include "ofc/scenario.hpp"
#include "ofc/transcription.hpp"
#include "ofc/trim.hpp"

namespace ofc::cases {

using nlohmann::json;

namespace {

Aircraft loadAircraft(const json& c) { return Aircraft::fromFile(c.at("aircraft").get<std::string>()); }

trim::FlightCondition flightFrom(const Aircraft& ac, const json& c) {
    const json f = c.value("flight", json::object());
    trim::FlightCondition fc;
    fc.V = f.value("V_mps", ac.envelope.V_cruise);
    fc.h = f.value("h_m", 300.0);
    fc.gamma = deg2rad(f.value("gamma_deg", 0.0));
    fc.mu = deg2rad(f.value("mu_deg", 0.0));
    fc.mass = f.value("mass_kg", 0.0);
    return fc;
}

template <class F> double millis(F&& fn, int repeats) {
    const auto t0 = std::chrono::steady_clock::now();
    for (int r = 0; r < repeats; ++r) fn();
    const auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(t1 - t0).count() / repeats;
}

void header(const char* title) { std::printf("\n== %s ==\n", title); }

} // namespace

// ── UC-01 ─────────────────────────────────────────────────────────────────
int aircraftReport(const json& c) {
    const Aircraft ac = loadAircraft(c);
    header("UC-01  Aircraft report");
    std::printf("name     : %s\nsource   : %s\n", ac.name.c_str(), ac.source.c_str());
    std::printf("mass     : m0 %.2f kg, empty %.2f kg, fuel %.2f kg\n", ac.mass.m0, ac.mass.m_empty, ac.mass.fuel_max);
    std::printf("geometry : S %.3f m2, b %.3f m, c %.3f m, AR %.2f\n", ac.geometry.S, ac.geometry.b, ac.geometry.c,
                ac.geometry.aspectRatio());
    std::printf("aero     : CL_max %.3f, k %.4f, (L/D)max %.2f at CL %.3f\n", ac.clMax(), ac.aero.k, ac.maxLiftToDrag(),
                ac.bestGlideCL());

    std::printf("\n%8s %10s %10s %10s %10s %12s\n", "h [m]", "rho", "V_stall", "V_(L/D)", "P_max [W]", "T_max@Vc [N]");
    const std::vector<double> hs = c.value("altitudes_m", std::vector<double>{0.0, 500.0, 1000.0, 2000.0});
    for (double h : hs)
        std::printf("%8.0f %10.4f %10.2f %10.2f %10.0f %12.1f\n", h, isa::density(h), ac.stallSpeed(h, ac.mass.m0),
                    ac.bestGlideSpeed(h, ac.mass.m0), ac.maxPower(h), ac.maxThrust(ac.envelope.V_cruise, h));

    std::printf("\nmin turn radius at V_cruise, phi_max: %.1f m\n",
                ac.minTurnRadius(ac.envelope.V_cruise, ac.envelope.phi_max));
    return 0;
}

// ── UC-02 ─────────────────────────────────────────────────────────────────
int trimReport(const json& c) {
    const Aircraft ac = loadAircraft(c);
    const trim::FlightCondition fc = flightFrom(ac, c);
    header("UC-02  Trim");
    std::printf("condition: V %.2f m/s, h %.0f m, gamma %.2f deg, mu %.2f deg\n", fc.V, fc.h, rad2deg(fc.gamma),
                rad2deg(fc.mu));

    const trim::UfTrim t1 = trim::pointMass(ac, fc);
    std::printf("\n[UF] T %.2f N, CL %.4f, n %.3f g, turn rate %.2f deg/s, within limits: %s\n", t1.u[uf::THRUST],
                t1.u[uf::CL], uf::loadFactor(ac, t1.x, t1.u), rad2deg(t1.turnRate), t1.feasible ? "yes" : "NO");
    std::printf("     |f(x,u)| on V, gamma: %.2e, %.2e\n", std::fabs(uf::rhs(ac, t1.x, t1.u)[uf::V]),
                std::fabs(uf::rhs(ac, t1.x, t1.u)[uf::GAMMA] - 0.0));

    if (fc.mu != 0.0) {
        std::printf("\n[FC] skipped: rigid-body trim is implemented for wings-level flight only.\n");
        return 0;
    }
    const trim::FcTrim t2 = trim::rigidBody(ac, fc);
    const Vec f = fc::rhs(ac, t2.x, t2.u);
    std::printf("\n[FC] alpha %.3f deg, theta %.3f deg, delta_e %.3f deg, T %.2f N\n", rad2deg(t2.alpha),
                rad2deg(t2.x[fc::THETA]), rad2deg(t2.u[fc::ELEVATOR]), t2.u[fc::THRUST]);
    std::printf("     Newton: %d iterations, residual %.2e, converged: %s\n", t2.iterations, t2.residual,
                t2.converged ? "yes" : "NO");
    std::printf("     |u_dot| %.2e, |w_dot| %.2e, |q_dot| %.2e\n", std::fabs(f[fc::U]), std::fabs(f[fc::W]),
                std::fabs(f[fc::Q]));
    std::printf("     de within limits: %s, alpha within limits: %s\n",
                std::fabs(t2.u[fc::ELEVATOR]) <= ac.actuators.de_max ? "yes" : "NO",
                t2.alpha >= ac.envelope.alpha_min && t2.alpha <= ac.envelope.alpha_max ? "yes" : "NO");
    return t2.converged ? 0 : 2;
}

// ── UC-03 ─────────────────────────────────────────────────────────────────
int simulation(const json& c) {
    const Aircraft ac = loadAircraft(c);
    const trim::FlightCondition fc = flightFrom(ac, c);
    const double tEnd = c.value("duration_s", 60.0), dt = c.value("dt_s", 0.01);
    const std::string out = c.value("output", std::string("output/uc03"));
    header("UC-03  Open-loop simulation (RK4)");

    // UF: level trim, then roll to a commanded bank with a proportional law on
    // the bank rate while lift and thrust follow the quasi-steady turn schedule.
    const json ufc = c.value("uf", json::object());
    const double muCmd = deg2rad(ufc.value("bank_cmd_deg", 30.0));
    const double kBank = ufc.value("bank_gain", 1.0);
    const trim::UfTrim t1 = trim::pointMass(ac, {fc.V, fc.h, 0.0, 0.0, fc.mass});
    const double CLlevel = t1.u[uf::CL];

    auto ufControls = [&](const Vec& x) {
        const double p = std::clamp(kBank * (muCmd - x[uf::MU]), -ac.actuators.p_max, ac.actuators.p_max);
        const double CL = std::min(CLlevel / std::cos(x[uf::MU]), ac.clMax());
        const double qS = 0.5 * isa::density(-x[uf::Z]) * x[uf::V] * x[uf::V] * ac.geometry.S;
        const double T = std::min(qS * (ac.aero.CD0 + ac.aero.k * CL * CL), ac.maxThrust(x[uf::V], -x[uf::Z]));
        return Vec{T, CL, p};
    };
    const num::Trajectory a =
        num::simulate([&](double, const Vec& x) { return uf::rhs(ac, x, ufControls(x)); }, t1.x, tEnd, dt);
    std::vector<Vec> rowsA;
    for (std::size_t k = 0; k < a.t.size(); ++k) {
        Vec r{a.t[k]};
        r.insert(r.end(), a.x[k].begin(), a.x[k].end());
        const Vec u = ufControls(a.x[k]);
        r.insert(r.end(), u.begin(), u.end());
        rowsA.push_back(r);
    }
    io::writeCsv(out + "/uf.csv", {"t", "x", "y", "z", "V", "gamma", "chi", "mu", "m", "T", "CL", "p"}, rowsA);
    const Vec& xa = a.x.back();
    std::printf("[UF] bank %.1f deg after %.0f s, heading change %.1f deg, altitude change %.2f m, fuel %.3f kg\n",
                rad2deg(xa[uf::MU]), tEnd, rad2deg(xa[uf::CHI] - t1.x[uf::CHI]), -(xa[uf::Z] - t1.x[uf::Z]),
                t1.x[uf::MASS] - xa[uf::MASS]);
    std::printf("     expected steady turn radius %.1f m\n", fc.V * fc.V / (isa::g0 * std::tan(muCmd)));

    // FC: wings-level trim held open loop, with an optional elevator doublet.
    const json fcc = c.value("fc", json::object());
    const double amp = deg2rad(fcc.value("elevator_doublet_deg", 0.0));
    const double t0 = fcc.value("doublet_start_s", 2.0), w = fcc.value("doublet_width_s", 1.0);
    const trim::FcTrim t2 = trim::rigidBody(ac, {fc.V, fc.h, 0.0, 0.0, fc.mass});
    if (!t2.converged) throw std::runtime_error("UC-03: rigid-body trim did not converge");

    auto fcControls = [&](double t) {
        Vec u = t2.u;
        if (t >= t0 && t < t0 + w)
            u[fc::ELEVATOR] += amp;
        else if (t >= t0 + w && t < t0 + 2.0 * w)
            u[fc::ELEVATOR] -= amp;
        return u;
    };
    const num::Trajectory b =
        num::simulate([&](double t, const Vec& x) { return fc::rhs(ac, x, fcControls(t)); }, t2.x, tEnd, dt);
    std::vector<Vec> rowsB;
    for (std::size_t k = 0; k < b.t.size(); ++k) {
        Vec r{b.t[k]};
        r.insert(r.end(), b.x[k].begin(), b.x[k].end());
        const fc::AirData air = fc::airData(b.x[k], {0.0, 0.0, 0.0});
        r.push_back(air.Va);
        r.push_back(air.alpha);
        const Vec u = fcControls(b.t[k]);
        r.insert(r.end(), u.begin(), u.end());
        rowsB.push_back(r);
    }
    io::writeCsv(out + "/fc.csv", {"t", "x", "y", "z", "u",  "v",     "w", "phi", "theta", "psi",
                                   "p", "q", "r", "m", "Va", "alpha", "T", "de",  "da",    "dr"},
                 rowsB);
    const Vec& xb = b.x.back();
    double hMin = 1e300, hMax = -1e300;
    for (const Vec& x : b.x) {
        hMin = std::min(hMin, -x[fc::ZD]);
        hMax = std::max(hMax, -x[fc::ZD]);
    }
    std::printf("[FC] doublet %.1f deg: altitude band [%.1f, %.1f] m, final theta %.2f deg, final Va %.2f m/s\n",
                rad2deg(amp), hMin, hMax, rad2deg(xb[fc::THETA]), fc::airData(xb, {0.0, 0.0, 0.0}).Va);
    std::printf("CSV written to %s/uf.csv and %s/fc.csv\n", out.c_str(), out.c_str());
    return 0;
}

// ── UC-04 ─────────────────────────────────────────────────────────────────
int transcriptionCheck(const json& c) {
    const Aircraft ac = loadAircraft(c);
    const json mj = c.value("mission", json::object());
    Mission ms;
    ms.start = io::vec3(mj, "start_ned_m", ms.start);
    ms.end = io::vec3(mj, "end_ned_m", ms.end);
    ms.wind = io::vec3(mj, "wind_ned_mps", ms.wind);
    ms.speedStart = mj.value("speed_mps", 0.0);
    ms.gammaStart = deg2rad(mj.value("gamma_start_deg", 0.0));
    ms.alpha = mj.value("alpha", 1.0);
    ms.betaControl = mj.value("beta_control", 0.0);
    const Vec3 d = ms.end - ms.start;
    ms.headingStart = ms.headingEnd = std::atan2(d[1], d[0]);
    if (mj.contains("heading_end_deg")) ms.headingEnd = deg2rad(mj.at("heading_end_deg").get<double>());
    const std::string out = c.value("output", std::string("output/uc04"));
    const std::vector<int> Ns = c.value("N", std::vector<int>{25, 50, 100, 200});

    header("UC-04  Transcription check (point mass, trapezoidal collocation)");
    std::printf("mission: %.0f m straight line, alpha = %.2f, beta_control = %.3f\n", norm(d), ms.alpha,
                ms.betaControl);
    std::printf("\n%5s %7s %7s %8s %8s | %10s %10s | %9s %9s %8s | %9s\n", "N", "vars", "defects", "nnz", "dens%",
                "|D|cold", "|D|trim", "J_an[ms]", "J_fd[ms]", "speedup", "jac err");

    std::vector<Vec> rows;
    for (int N : Ns) {
        const TrapezoidalUF nlp(ac, ms, N, UfScales::fromAircraft(ac, norm(d)));
        const Vec zCold = coldStartGuess(nlp, ac, ms);
        const Vec zTrim = trimmedLineGuess(nlp, ac, ms);

        const std::vector<Triplet> J = nlp.defectJacobian(zTrim);
        const double density = 100.0 * J.size() / (double(nlp.numDefects()) * nlp.numVariables());

        // Analytic sparse Jacobian vs forward differences (what the v1 solver did).
        const int reps = N <= 50 ? 20 : 3;
        const double tAn = millis([&] { (void)nlp.defectJacobian(zTrim); }, reps);
        const auto defectsFn = [&](const Vec& z) { return nlp.defects(z); };
        const double tFd = millis([&] { (void)num::fdJacobian(defectsFn, zTrim, num::FdScheme::Forward); }, 1);

        // Accuracy check against central differences (densified analytic Jacobian).
        const Vec Jfd = num::fdJacobian(defectsFn, zTrim, num::FdScheme::Central);
        Vec Jan(Jfd.size(), 0.0);
        for (const Triplet& t : J) Jan[t.row * nlp.numVariables() + t.col] += t.value;
        const double err = num::maxRelativeError(Jan, Jfd);

        const double dCold = maxAbs(nlp.defects(zCold)), dTrim = maxAbs(nlp.defects(zTrim));
        std::printf("%5d %7d %7d %8zu %8.2f | %10.2e %10.2e | %9.3f %9.2f %7.0fx | %9.1e\n", N, nlp.numVariables(),
                    nlp.numDefects(), J.size(), density, dCold, dTrim, tAn, tFd, tFd / tAn, err);
        rows.push_back({double(N), double(nlp.numVariables()), double(nlp.numDefects()), double(J.size()), density,
                        dCold, dTrim, tAn, tFd, err});

        if (N == Ns.front()) {
            std::vector<Vec> spy;
            for (const Triplet& t : J) spy.push_back({double(t.row), double(t.col)});
            io::writeCsv(out + "/sparsity_N" + std::to_string(N) + ".csv", {"row", "col"}, spy);
        }
    }
    io::writeCsv(out + "/transcription_check.csv",
                 {"N", "variables", "defects", "nnz", "density_pct", "defect_cold", "defect_trim", "jac_analytic_ms",
                  "jac_fd_ms", "jac_rel_error"},
                 rows);

    // Constraint, boundary and cost evaluation at the trimmed guess (smallest N).
    const TrapezoidalUF nlp(ac, ms, Ns.front(), UfScales::fromAircraft(ac, norm(d)));
    const Vec z = trimmedLineGuess(nlp, ac, ms);
    std::printf("\nTrimmed guess, N = %d: cost %.5f, |boundary residual| %.2e\n", Ns.front(), nlp.cost(z),
                maxAbs(nlp.boundaryResiduals(z)));
    const Vec gAn = nlp.costGradient(z);
    const Vec gFd = num::fdJacobian([&](const Vec& v) { return Vec{nlp.cost(v)}; }, z);
    std::printf("cost gradient, analytic vs central FD: max rel. error %.1e\n", num::maxRelativeError(gAn, gFd));
    std::printf("\n%-11s %12s %6s   (h <= 0 is satisfied)\n", "constraint", "worst h", "node");
    for (const auto& pc : nlp.pathSummary(z))
        std::printf("%-11s %12.4f %6d %s\n", pc.name.c_str(), pc.worst, pc.node, pc.worst > 0.0 ? "VIOLATED" : "");
    std::printf("\nCSV written to %s/\n", out.c_str());
    return 0;
}

// ── UC-05 ─────────────────────────────────────────────────────────────────
int designCharts(const json& c) {
    const Aircraft ac = loadAircraft(c);
    const double m = c.value("mass_kg", ac.mass.m0);
    const std::string out = c.value("output", std::string("output/uc05"));
    const std::vector<double> hs = c.value("altitudes_m", std::vector<double>{0.0, 1000.0, 2000.0, 4000.0});
    header("UC-05  Design charts");

    // Power curves at the selected altitudes.
    std::vector<Vec> power;
    for (double h : hs) {
        const double Vs = ac.stallSpeed(h, m);
        for (int i = 0; i <= 120; ++i) {
            const double V = Vs + (ac.envelope.V_ne - Vs) * i / 120.0;
            power.push_back(
                {h, V, perf::powerRequired(ac, V, h, m), perf::powerAvailable(ac, h), perf::rateOfClimb(ac, V, h, m)});
        }
    }
    io::writeCsv(out + "/power_curves.csv", {"h", "V", "P_req", "P_av", "roc"}, power);

    // Flight envelope over altitude.
    const double ceiling = perf::ceiling(ac, m), absCeiling = perf::ceiling(ac, m, 1e-3);
    std::printf("mass %.2f kg, service ceiling (0.5 m/s) %.0f m, absolute ceiling %.0f m\n\n", m, ceiling, absCeiling);
    std::printf("%7s %9s %9s %9s %9s %9s %10s\n", "h [m]", "V_stall", "V_minP", "V_(L/D)", "V_max", "ROC_max",
                "@V [m/s]");
    std::vector<Vec> env;
    for (double h = 0.0; h <= std::min(absCeiling, isa::hMax); h += 250.0) {
        const perf::SpeedPoint roc = perf::maxRateOfClimb(ac, h, m);
        const perf::SpeedPoint minP = perf::minPowerSpeed(ac, h, m);
        const double Vmax = perf::maxLevelSpeed(ac, h, m);
        env.push_back(
            {h, ac.stallSpeed(h, m), minP.V, ac.bestGlideSpeed(h, m), Vmax, roc.value, roc.V, ac.envelope.V_ne});
        if (std::fmod(h, 1000.0) == 0.0)
            std::printf("%7.0f %9.2f %9.2f %9.2f %9.2f %9.2f %10.2f\n", h, ac.stallSpeed(h, m), minP.V,
                        ac.bestGlideSpeed(h, m), Vmax, roc.value, roc.V);
    }
    io::writeCsv(out + "/envelope.csv",
                 {"h", "V_stall", "V_min_power", "V_best_LD", "V_max", "roc_max", "V_roc_max", "V_ne"}, env);

    // 6-DoF trim curves: elevator-to-trim and incidence versus speed.
    const double hTrim = c.value("trim_altitude_m", 300.0);
    std::vector<Vec> trimRows;
    const double Vs = ac.stallSpeed(hTrim, m);
    for (int i = 0; i <= 40; ++i) {
        const double V = 1.05 * Vs + (ac.envelope.V_ne - 1.05 * Vs) * i / 40.0;
        const trim::FcTrim t = trim::rigidBody(ac, {V, hTrim, 0.0, 0.0, m});
        const trim::UfTrim tu = trim::pointMass(ac, {V, hTrim, 0.0, 0.0, m});
        trimRows.push_back({V, rad2deg(t.alpha), rad2deg(t.u[fc::ELEVATOR]), t.u[fc::THRUST], tu.u[uf::THRUST],
                            tu.u[uf::CL], t.converged ? 1.0 : 0.0});
    }
    io::writeCsv(out + "/trim_curves.csv", {"V", "alpha_deg", "de_deg", "T_fc", "T_uf", "CL", "converged"}, trimRows);
    std::printf("\ntrim at %.0f m: delta_e from %.2f deg (%.1f m/s) to %.2f deg (%.1f m/s)\n", hTrim,
                trimRows.front()[2], trimRows.front()[0], trimRows.back()[2], trimRows.back()[0]);

    // Turn performance: radius and feasibility versus bank, at several speeds.
    const std::vector<double> Vt = c.value("turn_speeds_mps", std::vector<double>{18.0, 25.0, 32.0});
    std::vector<Vec> turn;
    for (double V : Vt)
        for (int i = 1; i <= 60; ++i) {
            const double phi = deg2rad(60.0 * i / 60.0);
            const perf::Turn t = perf::levelTurn(ac, V, hTrim, m, phi);
            turn.push_back({V, rad2deg(phi), t.n, t.radius, rad2deg(t.rate), t.powerRequired,
                            perf::powerAvailable(ac, hTrim), t.feasible ? 1.0 : 0.0});
        }
    io::writeCsv(out + "/turn.csv", {"V", "phi_deg", "n", "radius", "rate_dps", "P_req", "P_av", "feasible"}, turn);
    std::printf("CSV written to %s/\n", out.c_str());
    return 0;
}

// ── UC-06 ─────────────────────────────────────────────────────────────────
namespace {

using scenario::Model;
using scenario::Run;

std::vector<Vec> runRows(const Run& r) {
    std::vector<Vec> rows;
    for (std::size_t k = 0; k < r.t.size(); k += 10) { // 0.1 s output
        const scenario::Sample s = scenario::sampleOf(r, k);
        Vec row{r.t[k], r.s[k], s.x, s.y, -s.z, s.V, rad2deg(s.gamma), rad2deg(s.chi), rad2deg(s.bank), s.m};
        row.insert(row.end(), r.u[k].begin(), r.u[k].end());
        rows.push_back(row);
    }
    return rows;
}

// Largest 3D distance between the UF and FC paths at equal along-track distance.
double pathDeviation(const Run& a, const Run& b) {
    double worst = 0.0;
    std::size_t j = 0;
    for (std::size_t k = 0; k < b.s.size(); k += 10) {
        while (j + 1 < a.s.size() && a.s[j + 1] <= b.s[k]) ++j;
        const scenario::Sample p = scenario::sampleOf(a, j), q = scenario::sampleOf(b, k);
        worst = std::max(worst,
                         std::sqrt((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y) + (p.z - q.z) * (p.z - q.z)));
    }
    return worst;
}

// Resample the UF run on N uniform nodes and evaluate it in the NLP.
struct NlpCheck {
    double defect = 0.0, boundary = 0.0, costTime = 0.0, costFuel = 0.0;
    std::string tightest;
    double tightestValue = -1e300;
    double nfzRun = -1e300;  // worst NFZ row of the flown trajectory (<= 0 satisfied)
    double nfzCold = -1e300; // worst NFZ row of the straight-line cold start
};

NlpCheck evaluateInNlp(const Aircraft& ac, const Run& r, int N, const std::vector<env::NoFlyZone>& nfz = {}) {
    const double tf = r.flightTime;
    std::vector<Vec> X(N), U(N);
    for (int k = 0; k < N; ++k) {
        const double tk = tf * k / (N - 1);
        const auto it = std::upper_bound(r.t.begin(), r.t.end(), tk);
        const std::size_t i1 = std::min<std::size_t>(it - r.t.begin(), r.t.size() - 1), i0 = i1 ? i1 - 1 : 0;
        const double w = i1 == i0 ? 0.0 : (tk - r.t[i0]) / (r.t[i1] - r.t[i0]);
        X[k].resize(uf::NX);
        for (int i = 0; i < uf::NX; ++i) X[k][i] = (1.0 - w) * r.x[i0][i] + w * r.x[i1][i];
        U[k] = r.u[i0];
    }
    Mission ms;
    ms.start = {X.front()[uf::X], X.front()[uf::Y], X.front()[uf::Z]};
    ms.end = {X.back()[uf::X], X.back()[uf::Y], X.back()[uf::Z]};
    ms.headingStart = X.front()[uf::CHI];
    ms.headingEnd = X.back()[uf::CHI];
    ms.speedStart = X.front()[uf::V];
    ms.betaControl = 0.01;
    ms.nfz = nfz;

    NlpCheck chk;
    const double dist = r.s.back();
    ms.alpha = 0.0;
    const TrapezoidalUF nlpTime(ac, ms, N, UfScales::fromAircraft(ac, dist));
    ms.alpha = 1.0;
    const TrapezoidalUF nlp(ac, ms, N, UfScales::fromAircraft(ac, dist));
    const Vec z = nlp.pack(tf, X, U);
    chk.defect = maxAbs(nlp.defects(z));
    chk.boundary = maxAbs(nlp.boundaryResiduals(z));
    chk.costTime = nlpTime.cost(z);
    chk.costFuel = nlp.cost(z);
    for (const auto& pc : nlp.pathSummary(z)) {
        if (pc.name != "thrust_min" && pc.worst > chk.tightestValue) { // a sign bound, not a performance limit
            chk.tightestValue = pc.worst;
            chk.tightest = pc.name;
        }
        if (pc.name.rfind("nfz:", 0) == 0) chk.nfzRun = std::max(chk.nfzRun, pc.worst);
    }
    for (const auto& pc : nlp.pathSummary(coldStartGuess(nlp, ac, ms)))
        if (pc.name.rfind("nfz:", 0) == 0) chk.nfzCold = std::max(chk.nfzCold, pc.worst);
    return chk;
}

// Smallest NFZ clearance along a run [m] (negative = inside the margin), and
// the largest horizontal miss distance over the waypoints [m].
double runClearance(const Run& r, const std::vector<env::NoFlyZone>& zones) {
    double c = 1e300;
    for (std::size_t k = 0; k < r.x.size(); k += 5) {
        const scenario::Sample s = scenario::sampleOf(r, k);
        c = std::min(c, env::minClearance(zones, s.x, s.y, -s.z));
    }
    return c;
}

double waypointMiss(const Run& r, const std::vector<env::Waypoint>& wps) {
    double worst = 0.0;
    for (const env::Waypoint& w : wps) {
        double best = 1e300;
        for (std::size_t k = 0; k < r.x.size(); ++k) {
            const scenario::Sample s = scenario::sampleOf(r, k);
            best = std::min(best, std::hypot(s.x - w.position.x, s.y - w.position.y));
        }
        worst = std::max(worst, best);
    }
    return worst;
}

} // namespace

int designScenarios(const json& c) {
    const Aircraft ac = loadAircraft(c);
    const std::string out = c.value("output", std::string("output/uc06"));
    const int N = c.value("nodes", 60), Nfine = 4 * N;
    const std::vector<double> sweep = c.value("speed_sweep_mps", std::vector<double>{});
    header("UC-06  Design scenarios (closed-loop UF and FC, evaluated in the NLP)");

    std::printf("%-6s %-24s | %6s %6s %6s | %7s %7s | %6s %6s | %8s %8s | %-9s %6s | %6s\n", "id", "scenario", "t_UF",
                "t_FC", "dev[m]", "fuelUF", "fuelFC", "dh_end", "bank", "|D|N", "|D|4N", "tightest", "h", "V*fuel");
    std::vector<Vec> summary;
    int idx = 0;
    for (const json& sj : c.at("scenarios")) {
        const scenario::Scenario sc = scenario::Scenario::fromJson(sj, ac);
        const Run runUf = scenario::fly(ac, sc, Model::PointMass);
        const Run runFc = scenario::fly(ac, sc, Model::RigidBody);
        const std::vector<std::string> base{"t", "s", "x", "y", "h", "V", "gamma_deg", "chi_deg", "bank_deg", "m"};
        auto hdr = [&](std::vector<std::string> extra) {
            std::vector<std::string> h = base;
            h.insert(h.end(), extra.begin(), extra.end());
            return h;
        };
        io::writeCsv(out + "/" + sc.id + "_uf.csv", hdr({"T", "CL", "p"}), runRows(runUf));
        io::writeCsv(out + "/" + sc.id + "_fc.csv", hdr({"T", "de", "da", "dr"}), runRows(runFc));

        double maxBank = 0.0;
        for (std::size_t k = 0; k < runFc.x.size(); ++k)
            maxBank = std::max(maxBank, std::fabs(scenario::sampleOf(runFc, k).bank));
        const double dhEnd = -scenario::sampleOf(runFc, runFc.x.size() - 1).z - (sc.h0 + sc.climb);
        const double dev = pathDeviation(runUf, runFc);
        const NlpCheck chk = evaluateInNlp(ac, runUf, N);
        const NlpCheck fine = evaluateInNlp(ac, runUf, Nfine);

        // Speed trade-off (point mass): flight time against fuel.
        std::vector<Vec> trade;
        double bestV = 0.0, bestFuel = 1e300;
        for (double V : sweep) {
            scenario::Scenario s2 = sc;
            s2.V = V;
            if (V < 1.1 * ac.stallSpeed(sc.h0, ac.mass.m0)) continue;
            const Run r = scenario::fly(ac, s2, Model::PointMass, 0.02);
            trade.push_back({V, r.flightTime, r.fuel, r.completed ? 1.0 : 0.0});
            if (r.completed && r.fuel < bestFuel) {
                bestFuel = r.fuel;
                bestV = V;
            }
        }
        if (!trade.empty())
            io::writeCsv(out + "/" + sc.id + "_speed_trade.csv", {"V", "time", "fuel", "completed"}, trade);

        std::printf("%-6s %-24s | %6.1f %6.1f %6.1f | %7.4f %7.4f | %6.2f %6.1f | %8.1e %8.1e | %-9s %6.3f | %6.0f%s\n",
                    sc.id.c_str(), sc.title.substr(0, 24).c_str(), runUf.flightTime, runFc.flightTime, dev, runUf.fuel,
                    runFc.fuel, dhEnd, rad2deg(maxBank), chk.defect, fine.defect, chk.tightest.c_str(),
                    chk.tightestValue, bestV, runUf.completed && runFc.completed ? "" : "  (incomplete)");
        summary.push_back({double(idx), runUf.flightTime, runFc.flightTime, dev, runUf.fuel, runFc.fuel, dhEnd,
                           rad2deg(maxBank), chk.defect, fine.defect, chk.boundary, chk.costTime, chk.costFuel, bestV});
        ++idx;
    }
    io::writeCsv(out + "/summary.csv",
                 {"index", "time_uf", "time_fc", "deviation_m", "fuel_uf", "fuel_fc", "dh_end_fc", "max_bank_fc_deg",
                  "defect_N", "defect_4N", "boundary_residual", "cost_min_time", "cost_min_fuel", "V_min_fuel"},
                 summary);
    std::printf("\ncolumns: t = flight time [s]; dev = max UF-FC distance at equal along-track position [m];\n"
                "fuel [kg]; dh_end = FC altitude error at the end [m]; bank = max FC bank [deg];\n"
                "|D| = max normalised collocation defect of the flown UF trajectory resampled on N = %d\n"
                "and 4N = %d nodes (mesh refinement); tightest = path constraint closest\n"
                "to its limit (h <= 0 satisfied); V*fuel = speed of the sweep with the least fuel [m/s].\n",
                N, Nfine);
    std::printf("CSV written to %s/\n", out.c_str());
    return 0;
}

// ── UC-07 ─────────────────────────────────────────────────────────────────
int missionScenarios(const json& c) {
    const Aircraft ac = loadAircraft(c);
    const std::string out = c.value("output", std::string("output/uc07"));
    const int N = c.value("nodes", 80);
    const json pj = c.value("planner", json::object());
    plan::PlannerConfig cfg;
    cfg.cell = pj.value("cell_m", cfg.cell);
    cfg.headingBins = pj.value("heading_bins", cfg.headingBins);
    cfg.step = pj.value("step_m", cfg.step);
    const double bankFraction = pj.value("bank_fraction", 0.8);
    const double safety = pj.value("safety_m", 10.0);
    header("UC-07  Missions with no-fly zones and waypoints (Hybrid A* + L1 guidance, UF and FC)");

    std::printf("%-7s %-30s | %6s %6s %6s | %6s %6s | %6s %6s | %6s %6s | %8s %8s | %7s %7s\n", "id", "mission",
                "planMs", "R[m]", "path", "t_UF", "t_FC", "clrUF", "clrFC", "wpUF", "wpFC", "|D|N", "|D|4N", "nfzRun",
                "nfzCold");
    std::vector<Vec> summary;
    int status = 0, idx = 0;
    for (const json& mj : c.at("missions")) {
        scenario::Scenario sc = scenario::Scenario::fromJson(mj, ac);
        std::vector<env::NoFlyZone> zones;
        for (const json& z : mj.value("nfz", json::array())) zones.push_back(env::nfzFromJson(z));
        std::vector<env::Waypoint> wps;
        for (const json& w : mj.value("waypoints", json::array())) wps.push_back(env::waypointFromJson(w));
        const plan::Pose2 start{sc.x0, sc.y0, sc.heading0};
        const plan::Pose2 goal{mj.at("end").at(0).get<double>(), mj.at("end").at(1).get<double>(),
                               deg2rad(mj.value("end_heading_deg", 0.0))};
        const double radius = sc.V * sc.V / (isa::g0 * std::tan(bankFraction * sc.bankMax));
        const double hLow = std::min(sc.h0, sc.h0 + sc.climb), hHigh = std::max(sc.h0, sc.h0 + sc.climb);

        const plan::PlanResult pr = plan::planMission(start, wps, goal, radius, zones, hLow, hHigh, safety, cfg);
        if (!pr.found) {
            std::printf("%-7s %-30s | planner found no path after %d expansions\n", sc.id.c_str(),
                        sc.title.substr(0, 30).c_str(), pr.expansions);
            status = 2;
            continue;
        }
        sc.path = pr.path;
        const Run runUf = scenario::fly(ac, sc, Model::PointMass);
        const Run runFc = scenario::fly(ac, sc, Model::RigidBody);

        // Geometry and trajectories for plotting.
        std::vector<Vec> pathRows, zoneRows, wpRows;
        for (const plan::Pose2& p : pr.path) pathRows.push_back({p.x, p.y, rad2deg(p.psi)});
        for (std::size_t zi = 0; zi < zones.size(); ++zi)
            for (const env::Vec2& v : zones[zi].polygon)
                zoneRows.push_back({double(zi), v.x, v.y, zones[zi].hMin, std::min(zones[zi].hMax, 1e6),
                                    zones[zi].margin, zones[zi].activeBetween(hLow, hHigh) ? 1.0 : 0.0});
        for (const env::Waypoint& w : wps) wpRows.push_back({w.position.x, w.position.y, w.tolerance});
        wpRows.insert(wpRows.begin(), {start.x, start.y, -1.0});
        wpRows.push_back({goal.x, goal.y, -2.0});
        io::writeCsv(out + "/" + sc.id + "_path.csv", {"x", "y", "psi_deg"}, pathRows);
        io::writeCsv(out + "/" + sc.id + "_nfz.csv", {"zone", "x", "y", "h_min", "h_max", "margin", "active"},
                     zoneRows);
        io::writeCsv(out + "/" + sc.id + "_points.csv", {"x", "y", "tolerance"}, wpRows);
        const std::vector<std::string> base{"t", "s", "x", "y", "h", "V", "gamma_deg", "chi_deg", "bank_deg", "m"};
        auto hdr = [&](std::vector<std::string> extra) {
            std::vector<std::string> h = base;
            h.insert(h.end(), extra.begin(), extra.end());
            return h;
        };
        io::writeCsv(out + "/" + sc.id + "_uf.csv", hdr({"T", "CL", "p"}), runRows(runUf));
        io::writeCsv(out + "/" + sc.id + "_fc.csv", hdr({"T", "de", "da", "dr"}), runRows(runFc));

        const double clrUf = runClearance(runUf, zones), clrFc = runClearance(runFc, zones);
        const double wpUf = waypointMiss(runUf, wps), wpFc = waypointMiss(runFc, wps);
        const NlpCheck chk = evaluateInNlp(ac, runUf, N, zones);
        const NlpCheck fine = evaluateInNlp(ac, runUf, 4 * N, zones);
        double wpTol = 1e300;
        for (const env::Waypoint& w : wps) wpTol = std::min(wpTol, w.tolerance);
        const bool ok = runUf.completed && runFc.completed && clrUf >= 0.0 && clrFc >= 0.0 &&
                        (wps.empty() || std::max(wpUf, wpFc) <= wpTol);
        if (!ok) status = 2;

        auto fmtClr = [](double v) { return v > 1e299 ? std::string("n/a") : std::to_string(int(std::lround(v))); };
        auto fmtRow = [](double v) {
            char buf[16];
            std::snprintf(buf, sizeof buf, "%.2f", v);
            return v < -1e299 ? std::string("n/a") : std::string(buf);
        };
        std::printf("%-7s %-30s | %6.0f %6.0f %6.0f | %6.1f %6.1f | %6s %6s | %6.1f %6.1f | %8.1e %8.1e | %7s %7s%s\n",
                    sc.id.c_str(), sc.title.substr(0, 30).c_str(), pr.millis, radius, pr.length, runUf.flightTime,
                    runFc.flightTime, fmtClr(clrUf).c_str(), fmtClr(clrFc).c_str(), wpUf, wpFc, chk.defect, fine.defect,
                    fmtRow(chk.nfzRun).c_str(), fmtRow(chk.nfzCold).c_str(), ok ? "" : "  (CHECK)");
        // Missions without active zones have no clearance or NFZ rows: NaN in the CSV.
        auto orNan = [](double v) { return std::fabs(v) > 1e299 ? std::numeric_limits<double>::quiet_NaN() : v; };
        summary.push_back({double(idx++), pr.millis, double(pr.expansions), radius, pr.length, runUf.flightTime,
                           runFc.flightTime, runUf.fuel, runFc.fuel, orNan(clrUf), orNan(clrFc), wpUf, wpFc, chk.defect,
                           fine.defect, orNan(chk.nfzRun), orNan(chk.nfzCold)});
    }
    io::writeCsv(out + "/summary.csv",
                 {"index", "planner_ms", "expansions", "radius_m", "path_m", "time_uf", "time_fc", "fuel_uf", "fuel_fc",
                  "clearance_uf_m", "clearance_fc_m", "wp_miss_uf_m", "wp_miss_fc_m", "defect_N", "defect_4N",
                  "nfz_run", "nfz_cold"},
                 summary);
    std::printf("\ncolumns: planMs = Hybrid A* time; R = planning turn radius; path = planned length [m];\n"
                "t = flight time [s]; clr = smallest distance to an active NFZ beyond its margin [m] (>= 0 ok);\n"
                "wp = largest waypoint miss distance [m]; |D| = collocation defect of the flown UF trajectory\n"
                "on N = %d and 4N nodes; nfzRun / nfzCold = worst NFZ row of the NLP (<= 0 satisfied) for the\n"
                "flown trajectory and for a straight-line cold start between the same end points.\n",
                N);
    std::printf("CSV written to %s/\n", out.c_str());
    return status;
}

int run(const std::string& casePath) {
    const json c = io::readJson(casePath);
    const std::string uc = c.value("use_case", std::string());
    if (uc == "UC-01") return aircraftReport(c);
    if (uc == "UC-02") return trimReport(c);
    if (uc == "UC-03") return simulation(c);
    if (uc == "UC-04") return transcriptionCheck(c);
    if (uc == "UC-05") return designCharts(c);
    if (uc == "UC-06") return designScenarios(c);
    if (uc == "UC-07") return missionScenarios(c);
    throw std::invalid_argument("unknown use_case '" + uc + "' in " + casePath + " (expected UC-01 ... UC-07)");
}

} // namespace ofc::cases
