# results/

`benchmark_summary.csv` holds the aggregated warm-start benchmark of [chapter 14](../docs/14-results-warmstart.md). It covers 8 scenarios × 2 objectives, two repetitions each. Times are means and iteration counts come from the fastest repetition. All runs used the v1 solver, with finite-difference Jacobians ([Derivatives: v1 and v2](../docs/derivatives.md)).

## Naming: what `ws` and `no_ws` mean

The prefixes mean **different things for the two stages**, because the benchmark makes two separate comparisons:

| Prefix | UF columns (`*_uf_*`) | FC columns (`*_fc_*`) |
|---|---|---|
| `ws_` | Hybrid A\* + inverse dynamics → UF | full chain `WS → UF → FC` |
| `no_ws_` | linear-interpolation cold start → UF | direct seed `WS → FC` (inverse dynamics straight into FC, no UF stage) |

So `no_ws_fc_*` is **not** a cold start; it measures the value of the intermediate UF stage.

## Columns

| Column | Unit | Meaning |
|---|---|---|
| `objective` | – | `min_fuel` (α = 1) or `min_time` (α = 0) |
| `scenario_id`, `scenario_family` | – | `escenario1…8`; family `empty`, `nfz_only`, `waypoint_only`, `nfz_waypoint` |
| `repetitions` | – | runs averaged per case |
| `ws_uf_status`, `no_ws_uf_status`, `ws_fc_status`, `no_ws_fc_status` | – | `OK`, `INFEASIBLE` (restoration failed / infeasible) or `TIMEOUT` |
| `ws_uf_total_ms`, `no_ws_uf_total_ms` | ms | end-to-end time up to the end of the UF stage (warm-start stages included where used) |
| `uf_delta_ms` | ms | `ws_uf_total_ms − no_ws_uf_total_ms` (negative = warm start faster) |
| `uf_deviation_pct` | % | trajectory deviation between the two UF solutions |
| `uf_feasible` | bool | both UF runs converged |
| `ws_fc_total_ms`, `no_ws_fc_total_ms` | ms | end-to-end time of the whole pipeline (Hybrid A\* + inverse dynamics + [UF] + FC) |
| `fc_delta_ms`, `fc_deviation_pct`, `fc_feasible` | ms, %, bool | same as the UF columns, for the FC comparison |
| `ws_hastar_ms`, `ws_id_ms` | ms | Hybrid A\* and inverse-dynamics time |
| `*_solver_ms` | ms | IPOPT time of that stage only |
| `*_iters` | – | IPOPT iterations of that stage |
| `*_final_mass_kg` | kg | mass at the end of the trajectory (initial mass 4000 kg; exactly 4000.0 marks a non-physical, failed run) |
| `*_flight_time_s` | s | flight time of the resulting trajectory |
