# 11. Using and extending the library

Everything the `ofc` program does is available as a C++ library, `ofc_core`. This chapter shows how to link it, how to run the pipeline or a single optimisation stage from your own code, how to hand IPOPT a problem of your own, and how to add new constraints.

## 11.1 The library

`ofc_core` is a static library built from every source in `code/src/` except `main.cpp`. Its public headers are in `code/include/ofc/`, one per module; [Architecture](../code/architecture.md) shows how they depend on each other.

| Namespace | Header | Content |
|---|---|---|
| `ofc` | `aircraft.hpp`, `mission.hpp`, `types.hpp` | `Aircraft` (loading, validation, derived performance), `MissionSpec`, vectors and rotations |
| `ofc::isa` | `atmosphere.hpp` | Standard atmosphere |
| `ofc::uf`, `ofc::fc` | `point_mass.hpp`, `rigid_body.hpp` | The two models: `rhs`, `uf::jacobian`, air data, coefficients |
| `ofc::trim`, `ofc::num`, `ofc::perf` | `trim.hpp`, `numerics.hpp`, `performance.hpp` | Trim, RK4 and Newton, point performance |
| `ofc::env`, `ofc::plan` | `environment.hpp`, `planner.hpp` | No-fly zones and waypoints, Dubins paths and Hybrid A\* |
| `ofc::scenario`, `ofc::ws` | `scenario.hpp`, `warm_start.hpp` | Guidance laws, warm start, cold starts, UF → FC projection |
| `ofc::ocp` | `collocation.hpp`, `ocp_problem.hpp`, `ocp_uf.hpp`, `ocp_fc.hpp` | Decision-vector layout, collocation, the two optimal-control problems |
| `ofc::nlp` | `nlp.hpp`, `solver.hpp` | The `Problem` interface, `verify`, `solve` (IPOPT) and its options |
| `ofc::pipeline` | `pipeline.hpp` | `run`: the staged optimisation of one mission, and the post-checks |
| `ofc::io` | `io.hpp` | JSON input, CSV and JSON output |

## 11.2 Linking it from your project

The simplest way is to add the `code/` folder as a CMake subdirectory:

```cmake
cmake_minimum_required(VERSION 3.14)
project(my_app LANGUAGES CXX)

set(OFC_BUILD_TESTS OFF CACHE BOOL "" FORCE)        # the library's tests are not needed here
add_subdirectory(path/to/uavtraj-opt/code ofc)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE ofc_core)       # headers, C++17 and IPOPT come with it
```

IPOPT is found exactly as in chapter 1, and `OFC_IPOPT=ON` can be set before `add_subdirectory` to make it mandatory. On Windows with the prebuilt IPOPT, the IPOPT DLLs must be next to your executable or on `PATH`; for example:

```cmake
file(GLOB IPOPT_DLLS "$ENV{IPOPT_DIR}/bin/*.dll")
add_custom_command(TARGET my_app POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different ${IPOPT_DLLS} $<TARGET_FILE_DIR:my_app>)
```

The examples below load `data/aircraft/generic_uav.json` and `data/cases/tutorial_first_mission.json` with relative paths, so run them from `code/`. Both were compiled and run against the repository as described here; their output is shown after each listing.

## 11.3 Running the pipeline from C++

```cpp
#include <cstdio>

#include "ofc/aircraft.hpp"
#include "ofc/io.hpp"
#include "ofc/mission.hpp"
#include "ofc/pipeline.hpp"

using namespace ofc;

int main() {
    const Aircraft ac = Aircraft::fromFile("data/aircraft/generic_uav.json");
    const nlohmann::json c = io::readJson("data/cases/tutorial_first_mission.json");
    const MissionSpec m = MissionSpec::fromJson(c.at("missions").at(1), ac); // T-02

    pipeline::Config cfg = pipeline::Config::fromJson(c.at("pipeline"));
    cfg.alpha = 0.0; // minimum time instead of minimum fuel

    const pipeline::Result r = pipeline::run(ac, m, cfg);
    std::printf("UF %s in %d iterations: t_f = %.1f s\n", r.uf.status.c_str(), r.uf.iterations, r.ufOpt.tf);
    std::printf("FC %s in %d iterations: t_f = %.1f s\n", r.fc.status.c_str(), r.fc.iterations, r.fcOpt.tf);
    std::printf("FC verification: %s\n", r.fc.check.summary().c_str());
    return r.ok() ? 0 : 1;
}
```

```text
UF solved in 36 iterations: t_f = 90.4 s
FC acceptable in 78 iterations: t_f = 90.0 s
FC verification: dynamics ok | boundary ok | envelope ok | environment ok | bounds ok
```

`pipeline::Config` holds every setting of the `pipeline` block (section 3.4) as a C++ member: `seed`, `uf`, `fc`, `fcFromUf`, `nodesUf`, `nodesFc`, `alpha`, `betaControl`, `betaState`, `sideslipMax`, `warmStart`, `solver`, `verifyTol`. A default-constructed `Config` is the default pipeline (`WS>UF>FC`, minimum fuel, 60 nodes). `Config::withLabel("WS>UF")` returns a copy with another seed and stage sequence.

`pipeline::Result` contains the warm start (`warm`), the initial points and optima of both stages (`ufGuess`, `ufOpt`, `fcGuess`, `fcOpt`, each an `ocp::Trajectory` with `tf`, `X` and `U`), and the stage records (`warmStage`, `uf`, `fc`) with status, iterations, time, objective, seed and verification. `pipeline::clearanceAlong`, `waypointMissAlong` and `ufSimulationError` compute the post-checks of section 6.8.

## 11.4 Solving one stage by hand

To control each step yourself, build the problem from an initial point and call the solver directly:

```cpp
#include <cstdio>

#include "ofc/aircraft.hpp"
#include "ofc/io.hpp"
#include "ofc/mission.hpp"
#include "ofc/ocp_uf.hpp"
#include "ofc/point_mass.hpp"
#include "ofc/solver.hpp"
#include "ofc/warm_start.hpp"

using namespace ofc;

int main() {
    const Aircraft ac = Aircraft::fromFile("data/aircraft/generic_uav.json");
    const nlohmann::json c = io::readJson("data/cases/tutorial_first_mission.json");
    const MissionSpec m = MissionSpec::fromJson(c.at("missions").at(1), ac); // T-02

    // 1. Initial point: the warm start, resampled on 40 nodes.
    const ws::Result warm = ws::build(ac, m, ws::Config{});
    if (!warm.ok) return 1;
    const ocp::Trajectory guess = ws::fromRun(warm.uf, 40);

    // 2. Problem: minimum fuel, standard smoothing, references from the mission.
    const Mission ms = m.ufMission(/*alpha=*/1.0, /*betaControl=*/1.0, /*betaState=*/1.0);
    const ocp::Reference ref{m.referenceLength(), m.referenceLength() / m.flight.V};
    const auto wps = ocp::assignWaypoints(guess, m.waypoints, uf::X, uf::Y);
    const ocp::UfProblem problem(ac, ms, guess, wps, ref);

    // 3. Solve and verify.
    nlp::Options opt;
    opt.printLevel = 0;
    const nlp::Result r = nlp::solve(problem, opt);
    const nlp::Verification v = nlp::verify(problem, r.z);
    const ocp::Trajectory t = problem.trajectory(r.z);
    std::printf("%s (%s), %d iterations, t_f = %.1f s, fuel = %.2f g\n", nlp::toString(r.status), r.message.c_str(),
                r.iterations, t.tf, 1e3 * (t.X.front()[uf::MASS] - t.X.back()[uf::MASS]));
    std::printf("%s\n", v.summary().c_str());
    return r.ok() && v.ok() ? 0 : 1;
}
```

```text
solved (Solve_Succeeded), 63 iterations, t_f = 157.3 s, fuel = 6.44 g
dynamics ok | boundary ok | envelope ok | environment ok | bounds ok
```

Two differences from `pipeline::run` are worth knowing:

- The zone margins are used as given. The pipeline inflates them so that the segments between nodes keep the margin (section 7.5); without it, only the nodes are guaranteed to. Inflate `ms.nfz[j].margin` yourself if you need the same guarantee.
- Any `ocp::Trajectory` with the right number of states and controls can be the initial point: `ws::coldUf` for a straight line, a trajectory from a previous solve (resampled with `ocp::resample`), or your own.

`ocp::FcProblem` is used in the same way, with an `ocp::FcMission` (start state from `ws::fcStartState`, goal, heading, weights, sideslip limit, zones and altitude band) and a 6-DoF initial point, for example `ws::ufToFc(ac, ocp::resample(ufOptimum, N))`.

## 11.5 Your own problem with IPOPT

`nlp::solve` accepts any class derived from `nlp::Problem` ([`nlp.hpp`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/include/ofc/nlp.hpp)):

| Method | Must return |
|---|---|
| `numVariables()`, `numConstraints()` | Problem size |
| `variableBounds(zl, zu)`, `constraintBounds(gl, gu)` | Bounds; use `±nlp::kInf` for "no bound" |
| `initialPoint()` | The starting point |
| `variableScale()` (optional) | Typical magnitude of each variable; default 1 |
| `cost(z)`, `costGradient(z)` | Objective and its exact gradient |
| `constraints(z)` | Constraint values |
| `constraintJacobian(z)` | Nonzero entries as `{row, col, value}` triplets, **with the same rows and columns in the same order at every call** |
| `constraintLabel(i)`, `constraintKind(i)`, `variableLabel(j)` (optional) | Names and groups used by `nlp::verify` |

A minimal example, the Rosenbrock function constrained to a disc:

```cpp
#include <cstdio>

#include "ofc/nlp.hpp"
#include "ofc/solver.hpp"

using namespace ofc;

// min (1 - x)^2 + 100 (y - x^2)^2   subject to   x^2 + y^2 <= 1.5
class Rosenbrock : public nlp::Problem {
public:
    int numVariables() const override { return 2; }
    int numConstraints() const override { return 1; }
    void variableBounds(Vec& zl, Vec& zu) const override { zl = {-nlp::kInf, -nlp::kInf}, zu = {nlp::kInf, nlp::kInf}; }
    void constraintBounds(Vec& gl, Vec& gu) const override { gl = {-nlp::kInf}, gu = {1.5}; }
    Vec initialPoint() const override { return {-1.0, 1.0}; }
    double cost(const Vec& z) const override {
        return (1 - z[0]) * (1 - z[0]) + 100 * (z[1] - z[0] * z[0]) * (z[1] - z[0] * z[0]);
    }
    Vec costGradient(const Vec& z) const override {
        const double r = z[1] - z[0] * z[0];
        return {-2 * (1 - z[0]) - 400 * z[0] * r, 200 * r};
    }
    Vec constraints(const Vec& z) const override { return {z[0] * z[0] + z[1] * z[1]}; }
    std::vector<Triplet> constraintJacobian(const Vec& z) const override { return {{0, 0, 2 * z[0]}, {0, 1, 2 * z[1]}}; }
};

int main() {
    const Rosenbrock p;
    const nlp::Result r = nlp::solve(p);
    std::printf("%s: x = %.4f, y = %.4f\n", nlp::toString(r.status), r.z[0], r.z[1]);
    return r.ok() ? 0 : 1;
}
```

```text
solved: x = 0.9072, y = 0.8228
```

The adapter of section 8.3 then applies the scaling, the limited-memory Hessian and the status mapping to your problem as it does to the trajectory problems.

## 11.6 Adding constraints

### A new kind of environment constraint

Zones and waypoints of the existing kinds need no code: list them in the case file. A new kind (a circular zone, a ceiling, terrain clearance, a speed limit inside an area) touches six places:

1. **Model.** Add a struct, a `...FromJson` parser and a smooth, distance-like function to [`environment.hpp`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/include/ofc/environment.hpp) and `environment.cpp`.
2. **Mission data.** Add a vector of it to `MissionSpec` ([`mission.hpp`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/include/ofc/mission.hpp)) and parse it in `MissionSpec::fromJson`.
3. **NLP rows.** Add a member to `Mission` ([`transcription.hpp`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/include/ofc/transcription.hpp)) and `FcMission` ([`ocp_fc.hpp`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/include/ofc/ocp_fc.hpp)). In **both** models, append one name to the path-row names and one value to `pathAt`, in the same order (`TrapezoidalUF::pathConstraintNames` and `TrapezoidalUF::pathAt`; the `FcProblem` constructor and `FcProblem::pathAt`). Write the row as $h\le0$ and normalise it by a characteristic size. The Jacobian picks the new row up automatically (central differences per node). `nlp::verify` reports it in the `environment` group if its name starts with `nfz:`; extend `CollocationProblem::constraintKind` for other prefixes.
4. **Planner.** If the constraint restricts the horizontal path, extend the free-space test in `plan::planMission` ([`planner.cpp`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/src/planning/planner.cpp)), or the warm start will cross it.
5. **Pipeline.** Pass the new data from `MissionSpec` to both problems in `pipeline::run`.
6. **Tests.** Check the sign convention, a point inside and one outside, and a complete pipeline run that respects the constraint ([`tests/test_optimization.cpp`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/tests/test_optimization.cpp)).

The row must be smooth in the state: avoid switching it on and off with altitude or time inside `pathAt` (section 7.5 explains why zones are activated once per mission).

### A new boundary condition

Equality conditions at the start or the end (a final airspeed, a final flight-path angle) are boundary rows, added in the constructors of `UfProblem` ([`ocp_uf.cpp`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/src/formulation/ocp_uf.cpp)) and `FcProblem`. Each row is a label, a column of the decision vector, a target and a scale. For example, to impose a final airspeed in the UF problem, after the existing final-heading row:

```cpp
// Final airspeed (m.speedEnd: a new member of Mission, filled from the case file).
if (m.speedEnd > 0.0) boundary.push_back({"xf.V", L.idxState(K, V), m.speedEnd, s.x[V]});
```

`CollocationProblem` adapts the constraint count, the Jacobian (one constant entry per boundary row) and the labels by itself. Add the corresponding row to `FcProblem` with the forward body velocity `u` if the FC stage must respect it too.

## 11.7 Tests and formatting

The unit tests in `code/tests/` use no external framework: each file registers checks that `tests/main.cpp` runs. Add a test for every new model term, constraint or derivative, and compare any analytic derivative with `nlp::jacobianError` or a central difference. Run them with `ctest --test-dir build --output-on-failure -C Release`, and format the sources with clang-format 18 before committing (section 1.7).

[← 10. Troubleshooting](10-troubleshooting.md) · [Back to the user guide](index.md)
