# code/ — public edition of the trajectory-optimisation building blocks

C++17, no dependencies beyond the standard library and a bundled `nlohmann/json`. It contains:

- a **generic aircraft** described in JSON;
- the point-mass (UF) and 6-DoF (FC) models;
- the trapezoidal collocation of the optimal-control problem, with an analytic sparse Jacobian, constraints and cost;
- trim, simulation, design charts, and four typical design scenarios;
- missions that avoid no-fly zones and pass through waypoints (Hybrid A\* planner, L1 path following).

The optimiser and the remaining production features (terrain, geofences, the optimisation stages of the warm-start chain) are not included.

```bash
bash run.sh        # build, 25 tests, all 7 use cases, figures in output/figures/
.\run.ps1          # same on Windows PowerShell
```

| Use case | Case file |
|---|---|
| UC-01 Aircraft report | `data/cases/uc01_aircraft_report.json` |
| UC-02 Trim (UF and FC) | `data/cases/uc02_trim.json` |
| UC-03 Open-loop simulation | `data/cases/uc03_simulation.json` |
| UC-04 Transcription check | `data/cases/uc04_transcription.json` |
| UC-05 Design charts | `data/cases/uc05_design_charts.json` |
| UC-06 Typical design scenarios | `data/cases/uc06_design_scenarios.json` |
| UC-07 Missions with no-fly zones and waypoints | `data/cases/uc07_nfz_missions.json` |

Documentation:

- [Overview](../docs/code/index.md)
- [Architecture](../docs/code/architecture.md)
- [Formulation in code](../docs/code/formulation.md)
- [Use cases](../docs/code/use-cases.md)
- [**Worked examples and design scenarios**](../docs/code/examples.md)
- [Aircraft file format](../docs/code/aircraft-format.md)

Code style: `.clang-format` (clang-format 18, 4 spaces, 120 columns). CI rejects unformatted code. To reformat:

```bash
pip install clang-format==18.1.8
clang-format -i include/ofc/*.hpp src/*.cpp src/*/*.cpp tests/*.cpp tests/*.hpp
```

Licence: MIT ([LICENSE-CODE](../LICENSE-CODE)). `third_party/nlohmann/json.hpp`: MIT, © Niels Lohmann.
