# Use cases: input reference

Every use case is a JSON file in `code/data/cases/` with a `use_case` field and an `aircraft` path. Run them from `code/`:

```bash
./build/ofc data/cases/uc01_aircraft_report.json [more cases ...]
```

Paths are relative to the working directory. The program exits with a non-zero code if a case fails (for example, a trim that does not converge). Results and their interpretation are on the [Worked examples](examples.md) page.

## UC-01 — Aircraft report

```json
{ "use_case": "UC-01", "aircraft": "data/aircraft/generic_uav.json", "altitudes_m": [0, 1000, 2000] }
```

The report prints the input data and the **derived** performance: CL_max, k, (L/D)max, and a table over altitude with ρ, V_stall, the speed for best L/D, available power, and the thrust available at V_cruise. Use it to sanity-check a new aircraft file before anything else.

## UC-02 — Trim

```json
{ "use_case": "UC-02", "aircraft": "...", "flight": { "V_mps": 25, "h_m": 300, "gamma_deg": 0, "mu_deg": 0 } }
```

- **UF.** Closed form: $L\cos\mu=mg\cos\gamma$ and $T=D+mg\sin\gamma$. The report also checks the trim against C_L,max, available thrust, n_max and the bank limit.
- **FC.** Newton iterations on (α, δ_e, T) that zero u̇, ẇ and q̇ in straight, wings-level flight. The starting point comes from the UF trim and the linear lift and moment model.

## UC-03 — Open-loop simulation

```json
{ "use_case": "UC-03", "aircraft": "...", "flight": { "V_mps": 25, "h_m": 300 },
  "duration_s": 60, "dt_s": 0.01,
  "uf": { "bank_cmd_deg": 30, "bank_gain": 1.0 },
  "fc": { "elevator_doublet_deg": 2, "doublet_start_s": 2, "doublet_width_s": 0.5 },
  "output": "output/uc03" }
```

- **UF.** Rolls into a coordinated turn following the quasi-steady schedule ($C_L=C_{L,level}/\cos\mu$, T = D).
- **FC.** Starts from trim with the controls frozen, plus an elevator doublet.

Outputs: `uf.csv` and `fc.csv`.

## UC-04 — Transcription check

```json
{ "use_case": "UC-04", "aircraft": "...",
  "mission": { "start_ned_m": [0, 0, -300], "end_ned_m": [3000, 800, -400], "speed_mps": 25,
               "gamma_start_deg": 0, "heading_end_deg": 15, "wind_ned_mps": [0, 0, 0],
               "alpha": 1.0, "beta_control": 0.01 },
  "N": [25, 50, 100, 200], "output": "output/uc04" }
```

`gamma_start_deg` and `heading_end_deg` are optional. For each N, the report gives:

- the NLP size and sparsity;
- the maximum normalised defect of a cold start and of a trimmed straight line;
- the time per defect Jacobian, analytic against forward differences, and the relative error between them.

After the table it gives the cost, the boundary residual, a check of the cost gradient, and the worst value of every path constraint. Outputs: `transcription_check.csv` and `sparsity_N<smallest>.csv`.

## UC-05 — Design charts

```json
{ "use_case": "UC-05", "aircraft": "...", "altitudes_m": [0, 1000, 2000, 4000],
  "trim_altitude_m": 300, "turn_speeds_mps": [18, 25, 32], "mass_kg": 11.0, "output": "output/uc05" }
```

| Output | Content |
|---|---|
| `power_curves.csv` | thrust power required and available against airspeed, at each altitude |
| `envelope.csv` | V_stall, min-power speed, best-L/D speed, max level speed and max climb rate, every 250 m up to the absolute ceiling |
| `trim_curves.csv` | 6-DoF trim (α, δ_e, T) and UF trim thrust against airspeed |
| `turn.csv` | level-turn load factor, radius, rate and power against bank, at each speed |

`mass_kg` is optional and defaults to m₀. The report prints the service ceiling (where the climb rate drops to 0.5 m/s), the absolute ceiling, and the envelope table.

## UC-06 — Typical design scenarios

```json
{ "use_case": "UC-06", "aircraft": "...", "nodes": 60,
  "speed_sweep_mps": [18, 20, 22, 24, 26, 28, 30, 32, 34, 36],
  "scenarios": [
    { "id": "SC-03", "title": "Climb +200 m and 180 deg turn",
      "h0_m": 300, "climb_m": 200, "turn_deg": 180, "turn_start_m": 800, "distance_m": 2500,
      "V_mps": 25, "bank_deg": 30, "gamma_deg": 7.5 } ],
  "output": "output/uc06" }
```

`V_mps`, `bank_deg` and `gamma_deg` are optional. They default to V_cruise, ¾ φ_max and ½ γ_max.

Each scenario is flown by both models with the same guidance (see [Formulation in code](formulation.md#design-scenario-guidance-srcscenariosscenariocpp)). For each one, the report gives:

- flight time and fuel for UF and FC;
- the largest UF–FC distance at the same along-track position;
- the FC altitude error at the end, and the maximum FC bank;
- the collocation defect of the flown UF trajectory on N and 4N nodes;
- the path constraint closest to its limit;
- the speed that uses the least fuel in the sweep.

Outputs per scenario: `<id>_uf.csv`, `<id>_fc.csv` and `<id>_speed_trade.csv`, plus a `summary.csv` for all of them.

## UC-07 — Missions with no-fly zones and waypoints

```json
{ "use_case": "UC-07", "aircraft": "...", "nodes": 80,
  "planner": { "cell_m": 20, "heading_bins": 72, "step_m": 30, "bank_fraction": 0.8, "safety_m": 10 },
  "missions": [
    { "id": "M-04", "title": "NFZ, waypoint, climb; low NFZ overflown",
      "h0_m": 300, "climb_m": 150, "V_mps": 25, "bank_deg": 30,
      "start": [0, 0], "start_heading_deg": 0, "end": [3300, 1500], "end_heading_deg": 90,
      "nfz": [ { "id": "A", "polygon": [[1500, 300], [2200, 300], [2200, 1150], [1500, 1150]],
                 "h_min_m": 0, "h_max_m": 1500, "margin_m": 40 } ],
      "waypoints": [ { "id": "WP1", "position": [2700, 900], "tolerance_m": 40 } ] } ],
  "output": "output/uc07" }
```

- **Coordinates.** Positions are `[north, east]` in metres, and headings are in degrees from north towards east.
- **Polygons.** Each `polygon` is a simple polygon, in any orientation.
- **Altitude band.** A zone whose band does not overlap the flight's altitudes (`h0_m` to `h0_m + climb_m`) is ignored, that is, overflown.
- **Planner settings.** `planner` sets the search grid, the primitive length, the fraction of the bank limit used for the planning radius, and an extra safety distance on top of each zone's margin.

For each mission the report gives:

- the planner time, the turn radius and the planned length;
- flight times for UF and FC;
- the smallest distance to an active zone beyond its margin (≥ 0 is acceptable);
- the largest waypoint miss distance;
- the collocation defect on N and 4N nodes;
- the worst NFZ row of the NLP for the flown trajectory and for a straight-line cold start (≤ 0 satisfied).

The program exits with code 2 if a path is not found or a check fails. Outputs per mission: `<id>_path.csv`, `<id>_uf.csv`, `<id>_fc.csv`, `<id>_nfz.csv` and `<id>_points.csv`, plus a `summary.csv`.

## UC-08 — Optimal missions (needs IPOPT)

```json
{ "use_case": "UC-08", "aircraft": "...",
  "pipeline": {
    "seed": "warm", "stages": "UF>FC", "fc_from_uf": true,
    "nodes_uf": 60, "nodes_fc": 60,
    "alpha": 1.0, "beta_control": 1.0, "beta_state": 1.0, "sideslip_max_deg": 1.0, "verify_tol": 1e-4,
    "planner": { "cell_m": 20, "heading_bins": 72, "step_m": 30, "bank_fraction": 0.8, "safety_m": 10 },
    "solver": { "tol": 1e-6, "acceptable_tol": 1e-4, "max_iter": 1500, "max_cpu_time_s": 120, "print_level": 0 } },
  "missions": [ ... same format as UC-07 ... ],
  "output": "output/uc08" }
```

| Key | Meaning | Default |
|---|---|---|
| `seed` | `warm` (Hybrid A\* path flown by the guidance) or `cold` (straight line) | `warm` |
| `stages` | `UF>FC`, `UF` or `FC` | `UF>FC` |
| `fc_from_uf` | seed FC with the projected UF optimum; otherwise with the run flown by the FC autopilot | true |
| `nodes_uf`, `nodes_fc` | collocation nodes of each stage | 60 |
| `alpha` | 0 = minimum time, 1 = minimum fuel | 1 |
| `beta_control`, `beta_state` | smoothness weights on control and attitude increments | 1, 1 |
| `sideslip_max_deg` | sideslip limit of the FC problem (coordinated flight) | 1 |
| `verify_tol` | tolerance of the post-solve verification (rows are normalised) | 1e-4 |
| `solver` | IPOPT options: `tol`, `acceptable_tol`, `constr_viol_tol`, `max_iter`, `max_cpu_time_s`, `print_level`, `mu_strategy`, `lbfgs_history`, `linear_solver`, `derivative_test` | as shown |

For each mission the report gives:

- the warm-start time;
- for each stage: IPOPT status, iterations, solve time, flight time, fuel and the share of the cost due to smoothing;
- the smallest clearance beyond any zone's margin along the optimal paths (segments between nodes included);
- the largest waypoint miss;
- the local integration error of the UF optimum.

A converged stage whose verification fails is marked with `!` and its worst violated row is printed. Without IPOPT the case prints a notice and exits with code 0. Outputs per mission: `<id>_ws.csv` (warm start on the UF nodes), `<id>_uf.csv`, `<id>_fc.csv`, `<id>_path.csv`, `<id>_nfz.csv` and `<id>_points.csv`, plus `summary.csv` and `report.json` (every stage's status, IPOPT return code, seed, iterations, time, objective, initial defect, penalty share and verification summary).

## UC-09 — Warm-start benchmark (needs IPOPT)

```json
{ "use_case": "UC-09", "aircraft": "...",
  "pipelines": ["COLD>UF", "WS>UF", "WS>FC", "WS>UF>FC"], "repeats": 1,
  "pipeline": { ... as UC-08, seed and stages are set by each label ... },
  "missions": [ ... ], "output": "output/uc09" }
```

Each label is a seed (`WS` or `COLD`) followed by the stages. Every pipeline solves exactly the same problems: scales, cost references and t_f bounds come from the mission, never from the seed (only the node each waypoint is pinned to follows the initial point). With `repeats` > 1 the fastest run is kept. Outputs: `benchmark.csv` and `benchmark.json`.

[← Formulation in code](formulation.md) · [Next: Worked examples →](examples.md)
