# ✈️ Optimal 3D Trajectory Planning for a Fixed-Wing UAV

![License: CC BY 4.0](https://img.shields.io/badge/docs-CC%20BY%204.0-lightgrey)
![Solver](https://img.shields.io/badge/NLP-IPOPT-blue)
![Transcription](https://img.shields.io/badge/transcription-direct%20collocation-green)
![University](https://img.shields.io/badge/UPC-ESEIAAT-orange)

<p align="center">
  <img src="docs/assets/figures/benchmark/compare_fc_scenario8_min_fuel.png" width="85%" alt="Optimal trajectory in scenario 8: no-fly zone plus waypoints">
</p>

> Bachelor's thesis (Universitat Politècnica de Catalunya – ESEIAAT) developed with **CIMNE** and in collaboration with **Singular Aircraft**.
> Minimum-time / minimum-fuel 3D trajectory optimisation for a fixed-wing UAV, accelerated by a Hybrid A\* warm start.

📄 **Full thesis (PDF):** [`thesis/Optimal_3D_Trajectory_Planning_Thesis_Urioste.pdf`](thesis/Optimal_3D_Trajectory_Planning_Thesis_Urioste.pdf)

## TL;DR

- **Problem.** Compute flyable, energy-efficient 3D trajectories (climb, cruise, descent) that respect terrain (DEM), no-fly zones, waypoints and the flight envelope, for the amphibious heavy UAV **Flyox I** (Singular Aircraft).
- **Method.** Optimal control problem → trapezoidal **direct collocation** → large sparse NLP solved with **IPOPT**, in two fidelity levels (point-mass *UF*, 6-DoF rigid body *FC*), seeded by a **Hybrid A\* → inverse dynamics → UF → FC** warm-start chain.
- **Result.** Without a warm start the point-mass solver is infeasible in 3 of 8 scenarios. With the full chain, **all 8 converge**, and the 6-DoF stage is up to **4.4× faster** than seeding it directly from the geometric guess.

## Pipeline

```mermaid
flowchart LR
    A[Mission JSON] --> B[Dispatcher]
    B --> C[Hybrid A*<br/>2-D, min turn radius]
    C --> D[Inverse dynamics<br/>6-DoF trim]
    D --> E[UF NLP<br/>point mass, 12N+1 vars]
    E -->|analytic UF→FC projection| F[FC NLP<br/>6-DoF, 18N+1 vars]
    F --> G[Export<br/>cone compression + JSON]
    E -.UF-only.-> G
```

## Key results

| Result | Value |
|---|---|
| FC, full pipeline (`WS→UF→FC`) vs direct seed (`WS→FC`) | Faster in **7/7** converged scenarios, both objectives |
| Best FC speed-up, scenario 8 (NFZ + waypoints), min-fuel | 48.8 s → 11.1 s (**≈ 4.4×**) |
| Best FC speed-up, scenario 8, min-time | 41.4 s → 10.8 s (**≈ 3.8×**) |
| UF without warm start | **Infeasible in 3/8** (NFZ and waypoint scenarios) |
| UF with warm start | Converges **8/8**, in 0.5 – 3.0 s (meets the 4 s target) |
| `WS→FC` in scenario 7 | Times out (> 60 s); `WS→UF→FC` solves it in ≈ 11.9 s |
| Cost of the warm start | Hybrid A\* 40–220 ms + inverse dynamics 2–25 ms |

**Honest limitations.** The 6-DoF (FC) stage takes 4.5 – 12.9 s even with a warm start, so it does **not** meet the < 4 s real-time goal; only the point-mass (UF) layer does. For the point-mass stage the warm start is not always faster in the easy empty-airspace cases (it was faster in 3/5 min-fuel and 1/5 min-time cases); its real value is **robustness**. See [Conclusions](docs/15-conclusions.md).

<p align="center">
  <img src="docs/assets/figures/benchmark/speedup_min_fuel.png" width="48%" alt="FC speed-up, min-fuel">
  <img src="docs/assets/figures/benchmark/speedup_min_time.png" width="48%" alt="FC speed-up, min-time">
</p>

## Documentation

| Part | Pages |
|---|---|
| Project | [Problem](docs/01-problem.md) · [State of the art](docs/02-state-of-the-art.md) · [Methodology](docs/03-methodology.md) · [Design decisions](docs/04-design-decisions.md) |
| Theory | [OCP → NLP](docs/05-ocp-theory.md) · [UF dynamics](docs/06-dynamics-uf.md) · [FC dynamics](docs/07-dynamics-fc.md) · [Transcription](docs/08-transcription.md) · [Constraints](docs/09-constraints.md) · [Cost function](docs/10-cost-function.md) · [Warm start](docs/11-warm-start.md) |
| Results | [Validation](docs/12-results-validation.md) · [Regularisation](docs/13-results-penalties.md) · [Warm-start benchmark](docs/14-results-warmstart.md) · [Conclusions](docs/15-conclusions.md) |
| Appendix | [Coordinate frames](docs/appendix-frames.md) · [Adjoint method](docs/appendix-adjoint.md) · [References](docs/references.md) |

Browse locally with `pip install mkdocs-material && mkdocs serve`.

## Repository layout

```
├── thesis/        full thesis (PDF)
├── docs/          chapter-by-chapter write-up (MkDocs) + figures in docs/assets/
└── results/       aggregated benchmark table (CSV)
```

## Tech stack

Python · CasADi · IPOPT (MA57 / MUMPS) · C++17 (production port) · Linux · NumPy · Matplotlib · MkDocs.

## About the code

The production solver is proprietary to **Singular Aircraft** and is **not** part of this repository. This repo documents the **method and the results**: equations, diagrams, pseudocode and figures.

## Author & citation

**Pablo Urioste Alarcón** — supervised by Dr. Àlex Ferrer Ferré (CIMNE / UPC).
Cite via [`CITATION.cff`](CITATION.cff). Documentation and figures: CC BY 4.0 ([LICENSE](LICENSE)); example code: MIT ([LICENSE-CODE](LICENSE-CODE)).
