# 9. Example catalogue

The repository ships ten case files in [`code/data/cases/`](https://github.com/pablourioste/uavtraj-opt/tree/main/code/data/cases). This chapter describes each one: what it demonstrates, which inputs matter, what to look at in the output, and where its results are discussed. The examples are listed in the order in which they build on each other, from the aircraft to the optimiser.

All examples use the generic UAV of [`data/aircraft/generic_uav.json`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/data/aircraft/generic_uav.json): 11 kg, 2.9 m span, 800 W, cruise speed 25 m/s. Its mass, geometry, inertia and stability derivatives follow the Aerosonde-class model of Beard and McLain (2012); its drag polar, propulsion and envelope limits are illustrative values.

## Overview

| Example | Purpose | IPOPT | Input reference | Results |
|---|---|:---:|---|---|
| UC-01 | Check an aircraft file and its derived performance | | [UC-01](../code/use-cases.md#uc-01-aircraft-report) | [UC-01](../code/examples.md#uc-01-aircraft-report) |
| UC-02 | Trim both models in steady flight | | [UC-02](../code/use-cases.md#uc-02-trim) | [UC-02](../code/examples.md#uc-02-trim) |
| UC-03 | Simulate both models open loop | | [UC-03](../code/use-cases.md#uc-03-open-loop-simulation) | [UC-03](../code/examples.md#uc-03-open-loop-simulation) |
| UC-04 | Inspect the collocated NLP: size, sparsity, derivatives | | [UC-04](../code/use-cases.md#uc-04-transcription-check) | [UC-04](../code/examples.md#uc-04-the-nlp-an-optimiser-sees) |
| UC-05 | Draw the design charts of the aircraft | | [UC-05](../code/use-cases.md#uc-05-design-charts) | [UC-05](../code/examples.md#uc-05-design-charts) |
| UC-06 | Fly four reference manoeuvres with guidance laws | | [UC-06](../code/use-cases.md#uc-06-typical-design-scenarios) | [UC-06](../code/examples.md#uc-06-typical-design-scenarios) |
| UC-07 | Plan and fly missions around no-fly zones (the warm starts) | | [UC-07](../code/use-cases.md#uc-07-missions-with-no-fly-zones-and-waypoints) | [UC-07](../code/examples.md#uc-07-missions-with-no-fly-zones-and-waypoints) |
| UC-08 | Optimise the same missions: warm start → UF → FC | ✓ | [Chapter 3](03-inputs.md) | [UC-08](../code/examples.md#uc-08-optimal-missions) |
| UC-09 | Compare pipelines on identical problems | ✓ | [Section 3.5](03-inputs.md#35-benchmark-case-files-uc-09) | [UC-09](../code/examples.md#uc-09-warm-start-benchmark) |
| Tutorial | Three missions of increasing difficulty | ✓ | [Chapter 3](03-inputs.md) | [Chapter 5](05-tutorial.md) |

## UC-01: aircraft report

**File.** [`uc01_aircraft_report.json`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/data/cases/uc01_aircraft_report.json). **Key input:** `altitudes_m`.

Prints the aircraft data and the quantities derived from it: maximum lift coefficient, induced-drag factor, maximum lift-to-drag ratio, and for each altitude the density, stall speed, best lift-to-drag speed, available power and thrust at cruise speed.

**What to look at.** This is the first case to run with a new aircraft. The stall speed and the best lift-to-drag speed bracket the speeds a minimum-fuel optimum will fly (20.8 m/s at sea level for the generic UAV, which is where the UC-08 and tutorial optima settle). The minimum turn radius at cruise speed and maximum bank (76 m) tells you how tight the missions can be.

## UC-02: trim

**File.** [`uc02_trim.json`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/data/cases/uc02_trim.json). **Key input:** `flight` (speed, altitude, flight-path angle, bank).

Trims the point-mass model in closed form and the rigid-body model by Newton iterations on the force and moment balance.

**What to look at.** Both models need the same thrust (9.27 N against 9.25 N at 25 m/s), because they share the lift and drag description. The 6-DoF trim (incidence 3.2°, elevator -8.1°) is the initial state of every FC stage (section 7.6). If the 6-DoF trim fails to converge for your aircraft, the FC stage cannot start.

## UC-03: open-loop simulation

**File.** [`uc03_simulation.json`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/data/cases/uc03_simulation.json). **Key inputs:** `duration_s`, `dt_s`, `uf.bank_cmd_deg`, `fc.elevator_doublet_deg`.

Integrates both models with RK4: the point mass rolls into a 30° coordinated turn; the rigid body starts from trim and receives an elevator doublet.

**What to look at.** The point-mass turn rate matches $g\tan\phi/V$ at constant altitude. The 6-DoF response shows the two longitudinal modes: the short period (a fast incidence oscillation that dies out in about 1 s) and the phugoid (a slow exchange of speed and altitude with a period of about 12 s). A new aircraft whose doublet response diverges is unstable, and its 6-DoF optimisation will be hard.

## UC-04: transcription check

**File.** [`uc04_transcription.json`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/data/cases/uc04_transcription.json). **Key inputs:** `mission` (a straight 3.1 km leg with a 100 m climb), `N` (list of node counts).

Builds the point-mass collocation problem for several node counts and reports its size, its sparsity, the defects of a cold start and of a trimmed straight line, and the time to compute the defect Jacobian analytically and by finite differences.

**What to look at.** This example makes chapter 7 concrete. The number of variables is $1+11N$; only 0.44 % of the Jacobian entries are nonzero at $N=200$; the analytic Jacobian agrees with finite differences to about $10^{-10}$ and is 57 to 487 times faster, because finite differences cost one full evaluation per variable. A cold start violates the dynamics by order one, a trimmed line not at all: the reason for the warm start (section 6.1).

## UC-05: design charts

**File.** [`uc05_design_charts.json`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/data/cases/uc05_design_charts.json). **Key inputs:** `altitudes_m`, `trim_altitude_m`, `turn_speeds_mps`.

Computes power curves, the flight envelope with ceilings, 6-DoF trim curves against speed, and level-turn performance.

**What to look at.** The envelope (stall speed to maximum level speed, 15.1 to 36.9 m/s at sea level) bounds what the optimiser can do; the minimum-time tutorial optimum flies at 39 m/s only because it dives. The trim curves show that elevator authority, not stall, sets the minimum trimmed speed of this aircraft. Run this case for every new aircraft: unrealistic coefficients show up here as unreachable trim or odd power curves.

## UC-06: typical design scenarios

**File.** [`uc06_design_scenarios.json`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/data/cases/uc06_design_scenarios.json). **Key inputs:** `scenarios` (straight or 180° turn, level or +200 m climb), `speed_sweep_mps`, `nodes`.

Flies four manoeuvres with the guidance laws of the warm start, with both models, and sweeps the commanded speed to find the least-fuel speed.

**What to look at.** The two models agree to within 0.2–67 m and burn the same fuel. Climbs are power-limited. The least-fuel speed (22 m/s) sits next to the best lift-to-drag speed, which anticipates the minimum-fuel optima of UC-08. The defects of the flown trajectories on 60 and 240 nodes show that the mesh must resolve the fastest manoeuvre (section 6.10).

## UC-07: missions with no-fly zones and waypoints

**File.** [`uc07_nfz_missions.json`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/data/cases/uc07_nfz_missions.json). **Key inputs:** `missions` (same format as UC-08), `planner`, `nodes`.

Plans each mission with Hybrid A\* and flies the path with both models. These are exactly the warm starts of UC-08.

| Mission | Content |
|---|---|
| M-01 | One zone on the direct route |
| M-02 | Slalom between two zones |
| M-03 | Two waypoints, no zone |
| M-04 | One zone, one waypoint, a 150 m climb and a final heading change; a low zone on the direct route is overflown |

**What to look at.** The flown trajectories keep 11–16 m outside every margin and pass within about 1 m of each waypoint. The column `nfzCold` shows how far a straight-line cold start would intrude into the zones (200–394 m inside): the situation the warm start avoids. Use this case to check a new mission's geometry before optimising it: if the planner finds no path here, UC-08 will start from a straight line.

## UC-08: optimal missions

**File.** [`uc08_optimal_missions.json`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/data/cases/uc08_optimal_missions.json). **Key inputs:** the whole `pipeline` block and `missions` (chapter 3).

The four UC-07 missions, optimised for minimum fuel with the full pipeline: warm start, point-mass optimum on 60 nodes, projection, 6-DoF optimum on 60 nodes, verification.

**What to look at.** Every stage converges and passes the verification; the optimal paths keep 9–12 m beyond every margin between nodes; the two models agree to 0.5 % in time and 0.2 % in fuel. The optima slow to the best lift-to-drag speed, pass the waypoints on their tolerance circles, and shorten the planner's path by hugging the zone corners. Mission M-04 is the richest: it combines a zone, a waypoint, a climb and an overflown zone. Chapter 4 explains every output of this case, and [Worked examples: UC-08](../code/examples.md#uc-08-optimal-missions) discusses the results with figures.

**To adapt it.** Copy it under a new name, replace the missions with yours, and keep the pipeline block (chapter 5 shows the procedure).

## UC-09: warm-start benchmark

**File.** [`uc09_warmstart_benchmark.json`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/data/cases/uc09_warmstart_benchmark.json). **Key inputs:** `pipelines`, `repeats`, and the same `pipeline` and `missions` as UC-08.

Solves each mission with four pipelines: `COLD>UF`, `WS>UF`, `WS>FC` and `WS>UF>FC`. All pipelines solve identical problems, so their iterations, times and optima can be compared directly.

**What to look at.** With this small aircraft, the cold start still converges, but needs up to twice the iterations and in M-04 reaches a worse local optimum (+3.9 % cost). Seeding the FC stage with the UF optimum or with the FC autopilot run leads to the same 6-DoF optimum. This is the public counterpart of the thesis benchmark ([14. Warm-start benchmark](../14-results-warmstart.md)), where the cold start failed in 3 of 8 scenarios.

**To adapt it.** Put your own missions in it to measure, on your problem, whether the warm start and the UF stage pay off.

## Tutorial: first mission

**File.** [`tutorial_first_mission.json`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/data/cases/tutorial_first_mission.json). **Key inputs:** three missions T-01 (empty airspace), T-02 (one zone), T-03 (zone, waypoint and 100 m climb).

The worked example of chapter 5, written with every pipeline setting explicit so that it can serve as a template. It is not run by `run.sh`; run it by hand and plot it with `python utils/plot_results.py --optimal output/tutorial`.

## Choosing a starting point for your own case

| You want to | Start from |
|---|---|
| Check a new aircraft | UC-01, then UC-05, then UC-02 |
| Optimise one or more missions | The tutorial file or UC-08 |
| Check that a mission's geometry is feasible before optimising | UC-07 with your missions |
| Study minimum time against minimum fuel | The tutorial file, varying `alpha` (section 5.7) |
| Measure the value of the warm start on your missions | UC-09 |
| Study discretisation and derivatives | UC-04 |

[← 8. Using IPOPT](08-ipopt.md) · [Next: 10. Troubleshooting →](10-troubleshooting.md)
