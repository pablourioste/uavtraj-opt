# 3. Methodology

!!! abstract "TL;DR"
    The problem is non-convex and the solver only converges locally, so the formulation was grown **one block at a time**, validating each stage before enabling the next.

## Why not solve it in one shot?

A realistic 6-DoF mission becomes an NLP with thousands of variables. Three properties decide whether IPOPT succeeds:

- **Scaling.** Mass in tonnes, deflections in fractions of a radian, positions in kilometres. Without non-dimensionalisation the barrier sub-problem is ill-conditioned.
- **Initial guess.** A guess that violates many constraints sends the solver on a long detour through infeasibility, often to a non-physical local minimum (negative thrust, bank reversals).
- **Sparsity and exact derivatives.** A dense finite-difference Jacobian makes each iteration prohibitively expensive.

## Three-tier development

| Tier | Formulation | Role | Output used as |
|---|---|---|---|
| 1 | Analytical / semi-analytical | Qualitative understanding, sanity checks | Reference for tier 2 |
| 2 | Benchmark NLP (UF, minimal constraints) | Tune scaling, weights, solver options | Warm start for tier 3 |
| 3 | Full constrained NLP (UF → FC) with warm start | Mission-ready trajectories | Final deliverable |

```mermaid
flowchart LR
    T1[Tier 1<br/>closed-form checks] --> T2[Tier 2<br/>benchmark NLP]
    T2 --> T3[Tier 3<br/>full constraints + warm start]
```

## Validation along four axes

Every scenario is solved with both formulations (UF and FC), for minimum time and minimum fuel. Each solution is checked on four redundant axes:

1. **Trajectory inspection**: ground track and altitude profile against the expected qualitative signature.
2. **Constraint satisfaction**: worst-case normalised margin per constraint and collocation defect (‖Δ‖∞ ≤ 1e-4).
3. **Time vs fuel trade-off**: min-time against min-fuel on the same model.
4. **Regularisation impact**: every run repeated with smoothness penalties on and off.

## Warm-start assessment

The warm-start chain is compared against a cold start (straight line, constant speed, zero attitude) on: seed quality (collocation defect), IPOPT iterations, wall-clock time broken down by stage, and sensitivity to the number of nodes.

!!! info "A deliberate choice"
    The full chain is not the fastest option in every single scenario; on easy geometries a leaner seed can save a few iterations. It is the default because it is the **only configuration that converges in every scenario**.

[← Previous](02-state-of-the-art.md) · [Next: Design decisions →](04-design-decisions.md)
