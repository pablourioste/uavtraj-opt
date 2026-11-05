# 4. Reading the results

A UC-08 run reports its results three times: as a table on the console, as CSV files with every trajectory, and as a JSON report with the status of every stage. This chapter explains each of them and ends with a checklist for deciding whether a run can be trusted.

## 4.1 The console table

`./build/ofc data/cases/uc08_optimal_missions.json` prints one row per mission (Windows laptop, IPOPT 3.14.20):

```text
== UC-08  Optimal missions (warm start -> UF -> FC, IPOPT) ==
pipeline WS>UF>FC, N = 60 (UF) / 60 (FC), alpha = 1.00 (0 time, 1 fuel), beta = 1.00 (controls) / 1.00 (attitude)

id     | WS[ms] | UF           it   t[ms]  tf[s] fuel_g pen% | FC           it   t[ms]  tf[s] fuel_g pen% | clrUF clrFC  wpUF  wpFC  simUF
M-01   |    221 | solved      100     479  152.3    6.2    1 | solved       89     653  152.5    6.2    1 |     9     9     -     -   0.34
M-02   |    505 | solved      106     552  178.4    7.4    1 | acceptable   87    1046  178.7    7.4    1 |    11    11     -     -   0.38
M-03   |    182 | solved       96    1622  199.1    8.3    1 | solved       99    2902  199.2    8.3    1 |     -     -  40.0  40.0   0.89
M-04   |   1510 | solved      145    3699  185.8   11.5    0 | solved       98    3086  186.6   11.5    0 |    12    12  35.2  35.3   0.62
```

The header line repeats the settings actually used: the pipeline label, the node counts and the cost weights. Check it first; a misspelt key in the case file shows up here as a default value.

| Column | Meaning | Good value |
|---|---|---|
| `id` | Mission identifier | |
| `WS[ms]` | Time of the warm start: Hybrid A\* search plus both guidance runs | Small compared with the solves |
| `UF`, `FC` | Final status of each stage (section 4.2); `-` if the stage did not run | `solved` or `acceptable`, without `!` |
| `it` | IPOPT iterations of the stage | |
| `t[ms]` | Wall-clock time of the IPOPT call | Machine-dependent |
| `tf[s]` | Optimal flight time | |
| `fuel_g` | Fuel burnt, grams: initial minus final mass | |
| `pen%` | Share of the final cost due to the smoothness penalties | A few percent at most |
| `clrUF`, `clrFC` | Smallest horizontal distance beyond any active zone's margin, measured along the straight segments between nodes, metres. `-` if no zone is active | ≥ 0 |
| `wpUF`, `wpFC` | Largest horizontal distance between a waypoint and the trajectory, metres. `-` without waypoints | ≤ the waypoint's tolerance |
| `simUF` | Largest position error of an RK4 simulation of each interval, started from its node and driven by the optimal UF controls, metres | Small compared with the node spacing |

`simUF` measures the discretisation error of the collocation: the difference between what the collocated model predicts over one interval and what an accurate integration of the same controls gives. It is restarted at every node, so it does not grow with the flight length. (`summary.csv` also contains `sim_drift_uf_m`, a single open-loop simulation over the whole flight, which drifts by tens of metres because small heading and speed errors integrate; a tracking controller removes that drift in flight.)

## 4.2 Stage status

| Status | Meaning | Counts as converged |
|---|---|:---:|
| `solved` | IPOPT reached its tolerance `tol` | ✓ |
| `acceptable` | IPOPT reached the looser `acceptable_tol` for several consecutive iterations without reaching `tol` | ✓ |
| `infeasible` | IPOPT converged to a point that minimises the constraint violation but does not satisfy the constraints | |
| `max_iter` | The iteration limit `max_iter` was reached | |
| `max_time` | The time limit `max_cpu_time_s` was reached | |
| `failed` | Any other IPOPT outcome: restoration failure, step too small, invalid numbers, and so on | |
| `no_solver` | The program was built without IPOPT | |

A stage is **successful** only if IPOPT converged **and** the independent verification (section 4.3) found no violated constraint or bound. A stage that is not successful is marked with a trailing `!` (for example `max_iter!`, or `solved!` if IPOPT converged but the verification failed; a `failed` stage is shown without it), the row ends with `(CHECK)`, and one line per failed stage follows with IPOPT's return code and the verification summary. With `max_iter` reduced to 20, for example:

```text
T-03   |     53 | max_iter!    20     130  179.9   10.1    1 | max_iter!    20     158  183.5   10.2    1 |    22    22  41.5  39.0   0.93  (CHECK)
       UF: Maximum_Iterations_Exceeded; dynamics 179 (defect gamma @56 2.7e-02) | boundary ok | envelope ok | environment 1 (wp:WP1 @38 8.0e-02) | bounds ok
       FC: Maximum_Iterations_Exceeded; dynamics 1 (defect p @53 1.5e-04) | boundary ok | envelope ok | environment ok | bounds ok
```

If the warm start itself fails (no path found, or the guidance did not reach the goal), a line `warm start: <reason>` is printed, and the UF stage falls back to a straight-line initial point.

## 4.3 The verification report

After each solve, `nlp::verify` evaluates every constraint and every variable bound at the final point, outside IPOPT, and groups the violations larger than `verify_tol` (default 10⁻⁴) by kind:

| Group | Rows | Example label |
|---|---|---|
| `dynamics` | Collocation defects: the equations of motion between consecutive nodes | `defect gamma @56`: flight-path-angle equation, interval 56 |
| `boundary` | Initial state and final position and heading | `x0.V`, `xf.chi` |
| `envelope` | Path constraints from the aircraft envelope | `stall @12`, `power @30`, `sideslip @7` |
| `environment` | No-fly zones and waypoints | `nfz:A @41`, `wp:WP1 @38` |
| `bounds` | Box bounds of the variables, relative to the variable's scale | `V @3`, `tf` |

`@k` is the node (or, for defects, the interval starting at node k), counted from 0. For each group the report gives the number of violations, the worst one and its label, for example `dynamics 179 (defect gamma @56 2.7e-02)`. All rows are normalised (chapter 7), so a value of 2.7·10⁻² means a violation of 2.7 % of the row's reference scale.

The verification exists because a solver's own convergence test is made on its internally scaled problem; re-evaluating the original rows confirms that the optimum satisfies what the case file asked for.

## 4.4 Output files of UC-08

All files are written to the folder given by `output`. Numbers have ten significant digits.

### Per mission

| File | Rows | Columns |
|---|---|---|
| `<id>_ws.csv` | Warm start, resampled on the UF nodes (the initial point of the UF stage) | as `<id>_uf.csv` |
| `<id>_uf.csv` | Point-mass optimum, one row per node | `t, x, y, h, V, gamma_deg, chi_deg, bank_deg, m, T, CL, p_dps` |
| `<id>_fc.csv` | 6-DoF optimum, one row per node | `t, x, y, h, Va, alpha_deg, beta_deg, phi_deg, theta_deg, psi_deg, p_dps, q_dps, r_dps, m, T, de_deg, da_deg, dr_deg` |
| `<id>_path.csv` | Hybrid A\* path, sampled every 5 m | `x, y, psi_deg` |
| `<id>_nfz.csv` | Vertices of every zone | `zone` (index), `x, y, h_min, h_max, margin, active` (1 or 0) |
| `<id>_points.csv` | Start, waypoints and goal | `x, y, tolerance`; the start has tolerance -1 and the goal -2 |

Column symbols: `x` north and `y` east position [m], `h` altitude [m], `V`/`Va` airspeed [m/s], `gamma` flight-path angle, `chi` heading, `bank`/`phi` bank (roll) angle, `theta` pitch, `psi` yaw, `alpha` incidence, `beta` sideslip, `m` mass [kg], `T` thrust [N], `CL` lift coefficient, `p` UF bank rate or FC roll rate, `q` pitch rate, `r` yaw rate, `de`/`da`/`dr` elevator, aileron and rudder deflections. Angles are in degrees and rates in degrees per second.

The controls at the last node of a trajectory are decision variables like any other, but they only act through the final half interval, so they are less meaningful than the others.

### Per case

`summary.csv` has one row per mission, with the console values at full precision plus a few more:

| Column | Meaning |
|---|---|
| `index` | Position of the mission in the case file |
| `ws_ms` | Warm-start time |
| `uf_ok`, `fc_ok` | 1 if the stage succeeded (converged and verified), else 0 |
| `uf_iter`, `uf_ms`, `fc_iter`, `fc_ms` | Iterations and solve times |
| `tf_uf`, `tf_fc`, `fuel_uf_g`, `fuel_fc_g` | Flight time and fuel of each optimum |
| `clearance_uf_m`, `clearance_fc_m` | As `clrUF`, `clrFC`; `nan` without active zones |
| `wp_miss_uf_m`, `wp_miss_fc_m` | As `wpUF`, `wpFC`; 0 without waypoints |
| `sim_local_uf_m`, `sim_drift_uf_m` | Interval-restarted and whole-flight simulation errors (section 4.1) |
| `guess_defect_uf`, `guess_defect_fc` | Largest normalised collocation defect of each stage's initial point: how far the seed is from satisfying the dynamics |
| `penalty_uf_pct`, `penalty_fc_pct` | As `pen%` |

`report.json` holds one record per mission:

```json
{
  "id": "T-01", "title": "Empty airspace", "pipeline": "WS>UF>FC", "total_ms": 1319.4,
  "warm_start": { "ran": true, "ok": true, "status": "ok", "message": "ok", "millis": 36.0, ... },
  "uf": {
    "ran": true, "ok": true, "status": "solved", "message": "Solve_Succeeded",
    "seed": "warm start (UF guidance run)",
    "millis": 539.4, "iterations": 116, "objective": 0.37027,
    "guess_defect": 0.215, "penalty_share": 0.0065,
    "verification": "dynamics ok | boundary ok | envelope ok | environment ok | bounds ok"
  },
  "fc": { ..., "seed": "UF optimum (projected)", ... }
}
```

`seed` records where the stage's initial point came from (sections 6.5 and 6.7). `message` is IPOPT's own return code (chapter 8). `objective` is the final value of the cost function (chapter 7); `penalty_share` is the fraction of it due to the smoothness terms.

## 4.5 Output files of UC-09

UC-09 prints one row per mission and pipeline:

```text
id     pipeline   | WS[ms] | UF           it   t[ms] | FC           it   t[ms] |    total       J  tf[s] fuel_g
M-04   COLD>UF    |      0 | solved      124     625  | -                        |      626  0.6106  195.3  11.96
M-04   WS>UF      |    286 | solved      103     512  | -                        |      798  0.5877  185.8  11.55
```

`total` is the wall-clock time of the whole pipeline, and `J`, `tf[s]` and `fuel_g` describe the result of its last stage. With `repeats` > 1 the fastest repetition is reported. The same data are in `benchmark.csv` (stage flags are -1 when a stage did not run) and, with the full stage records of section 4.4, in `benchmark.json`.

## 4.6 Exit codes

| Code | Meaning |
|---|---|
| 0 | Every case ran and every check passed. Also returned when UC-08 or UC-09 are skipped because IPOPT is missing |
| 1 | No case file given, or a case stopped with an error (file not found, invalid JSON, missing key, invalid aircraft). The message is printed as `error (<file>): <reason>` |
| 2 | A case ran, but at least one mission did not pass its checks: a UC-08 stage did not succeed, or a UC-07 path was not found or violated a zone or waypoint |

UC-09 returns 0 even when some pipelines fail, because a failed pipeline is a legitimate benchmark result. Scripts that run cases unattended should test the exit code.

## 4.7 Figures

`python utils/plot_results.py` (or `--optimal <folder>` for your own case) draws, for UC-08:

- `uc08_cover.png`: top view of every mission, with the Hybrid A\* path (thin black), the flown warm start (grey dash-dot), the UF optimum (blue), the FC optimum (red dashed), the active zones with their margin (dotted) and the waypoints with their tolerance circle;
- `uc08_<id>.png`: the same top view next to altitude, airspeed, bank and thrust against time, for the warm start and both optima.

Zones below the flight are drawn hatched in grey: they are inactive and may be overflown.

## 4.8 Is the result trustworthy?

Go through these checks in order:

1. **Status.** Every stage is `solved` or `acceptable`, without `!`, and the row has no `(CHECK)`.
2. **Clearance and waypoints.** `clr` ≥ 0 and `wp` within the tolerance. Both are measured between nodes as well, so they are stricter than the constraints the solver saw.
3. **Discretisation error.** `simUF` is a small fraction of the node spacing (path length divided by N − 1). If it is not, increase the number of nodes.
4. **Penalty share.** `pen%` stays at a few percent. A large share means the smoothness terms, not the mission, shape the optimum; reduce `beta_control` and `beta_state`.
5. **Model agreement.** UF and FC flight time and fuel agree closely (within about 1 % in the bundled missions). A large gap points to a seed problem or to the sideslip limit (section 7.4).
6. **Physics.** The profiles make sense: for minimum fuel the airspeed settles near the best lift-to-drag speed (UC-01 prints it), turns use moderate bank, and thrust is smooth.

IPOPT finds **local** optima. Two different seeds can converge to two different optima of the same problem, as UC-09 shows for mission M-04 (cold start: 195 s and 11.96 g; warm start: 186 s and 11.55 g). The warm start makes a good local optimum likely, not certain.

[← 3. Input files](03-inputs.md) · [Next: 5. Tutorial →](05-tutorial.md)
