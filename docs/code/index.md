# Code: public edition

!!! abstract "TL;DR"
    `code/` is a self-contained C++17 implementation of the thesis's **formulation block**: a generic aircraft described in JSON, the point-mass (UF) and 6-DoF (FC) models, the trapezoidal collocation of the optimal-control problem with an analytic sparse Jacobian, and its constraints and cost. Six runnable use cases cover the aircraft's design charts and four typical design scenarios. The optimiser and the mission-level features of the production solver are **not** included.

!!! warning "A restricted, clean-room edition"
    The production solver was developed as a product for Singular Aircraft and stays private. This edition was rewritten from the thesis equations. It has its own structure and naming, and a **generic aircraft** whose coefficients come from public sources, and it deliberately stops before the optimiser.

## Quick start

Requirements: CMake ≥ 3.14 and a C++17 compiler (GCC, Clang or MSVC). There are no other dependencies. Python with `numpy` and `matplotlib` is only needed for the figures.

```bash
cd code
bash run.sh          # Linux / macOS / Git Bash: build, 20 tests, 6 use cases, figures
.\run.ps1            # Windows PowerShell
```

Run a single case by hand, from `code/`:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
./build/ofc data/cases/uc06_design_scenarios.json
python utils/plot_results.py            # figures in output/figures/
```

## What it contains

| Use case | What it does | Page |
|---|---|---|
| UC-01 Aircraft report | Derived performance from the design data | [Examples](examples.md#uc-01-aircraft-report) |
| UC-02 Trim | Steady flight: UF closed form, FC Newton on the force/moment balance | [Examples](examples.md#uc-02-trim) |
| UC-03 Simulation | RK4: UF coordinated turn, FC elevator doublet | [Examples](examples.md#uc-03-open-loop-simulation) |
| UC-04 Transcription check | NLP size, sparsity, seed quality, analytic vs finite-difference Jacobian | [Examples](examples.md#uc-04-the-nlp-an-optimiser-would-see) |
| UC-05 Design charts | Power curves, flight envelope, trim curves, turn performance | [Examples](examples.md#uc-05-design-charts) |
| UC-06 Design scenarios | SC-00…SC-03 flown by UF and FC, speed trade-off, NLP evaluation | [Examples](examples.md#uc-06-typical-design-scenarios) |

## How it works

```mermaid
flowchart LR
    A[data/aircraft/*.json] --> B[Aircraft<br/>validate + derive]
    C[data/cases/*.json] --> D{use case}
    B --> D
    D --> P[performance<br/>UC-01, UC-05]
    D --> T[trim<br/>UC-02]
    D --> S[RK4 simulation<br/>UC-03]
    D --> N[TrapezoidalUF<br/>UC-04]
    D --> G[guidance + autopilot<br/>UC-06]
    G --> N
    S & N & G & P --> O[output/*.csv]
    O --> F[utils/plot_results.py]
```

1. **Aircraft.** The JSON is validated, with errors that name the field: mass budget, positive-definite inertia, static pitch stability, stall speed below V_NE. Everything derived (V_stall(h, m), CL_max, T_max, best L/D) is computed from it and never typed in. See the [aircraft file format](aircraft-format.md).
2. **Models.** `uf::rhs`, `uf::jacobian` and `fc::rhs` are pure functions f(x, u, wind).
3. **Formulation.** `TrapezoidalUF` lays out the decision vector and evaluates everything an interior-point solver needs at each iteration: defects and their Jacobian, boundary residuals, path constraints, cost and gradient. See [Formulation in code](formulation.md).
4. **Scenarios.** A common guidance law (altitude → flight-path angle, heading → bank) drives the UF model by dynamic inversion and the FC model through a small autopilot. The resulting flyable trajectories are then evaluated in the NLP.

## Scope

| Included | Production only (not published) |
|---|---|
| Generic aircraft model, data validation, point performance | Real aircraft data |
| UF and FC dynamics, ISA atmosphere, constant wind | Layered wind profiles, geodetic (WGS-84) mission frames |
| Trapezoidal transcription, analytic UF Jacobian, cost and gradient | NLP solver interface (IPOPT) and its tuning |
| Envelope and boundary constraints | Terrain (DEM), no-fly zones, geofences, waypoints |
| Cold-start, trimmed-line and guidance-flown trajectories | Hybrid A\* → inverse dynamics → UF → FC warm-start chain |
| Trim, RK4 simulation, closed-loop scenarios, 20 tests | Mission dispatcher, autopilot export, web viewer |

The excluded parts are described (equations, pseudocode and results) in the Theory and Results chapters of this documentation.

## Layout

```
code/
├── data/aircraft/     generic_uav.json
├── data/cases/        uc01 … uc06 case files
├── include/ofc/       one header per module
├── src/model/         atmosphere, aircraft, performance, point mass (UF), rigid body (FC)
├── src/numerics/      finite differences, Newton, RK4, trim
├── src/formulation/   trapezoidal transcription, constraints, cost, guesses
├── src/scenarios/     guidance, UF dynamic inversion, FC autopilot
├── src/app/           use cases, JSON/CSV I/O;  src/main.cpp = CLI
├── tests/             dependency-free unit tests (ctest)
├── third_party/       nlohmann/json 3.12.0 (MIT)
└── utils/             plot_results.py
```

[Next: Architecture →](architecture.md)
