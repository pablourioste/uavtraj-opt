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

[← Formulation in code](formulation.md) · [Next: Worked examples →](examples.md)
