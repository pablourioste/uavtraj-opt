# 15. Conclusions

!!! abstract "TL;DR"
    A hierarchical warm start makes a 6-DoF trajectory optimiser **robust** (8/8 scenarios converge, against 5/8 from a cold start) and **faster** (1.2–4.4× for the 6-DoF stage) without degrading the optimum. Only the point-mass layer met the 4 s real-time target with the solver version benchmarked here (v1, finite-difference derivatives).

## What was delivered

An end-to-end pipeline for 3D minimum-time and minimum-fuel trajectory planning of a fixed-wing aircraft (climb, cruise, descent), with:

- a point-mass (UF) and a 6-DoF rigid-body (FC) formulation,
- trapezoidal direct collocation solved with IPOPT, with residual normalisation,
- terrain, NFZ, geofence, waypoint and flight-envelope constraints,
- a Bolza cost with a time–fuel parameter and smoothness regularisation,
- a Hybrid A\* → inverse dynamics → UF → FC warm-start chain,
- an external validation suite and a warm-start benchmark,
- a restricted public edition of the formulation, with a generic aircraft and runnable design examples ([Code](code/index.md)).

## What the evidence supports

1. **A cold start is not viable for constrained missions.** UF fails in 3 of 8 scenarios. Where it "stops", the trajectories are not physical: no fuel is burned.
2. **The full chain converged in every tested scenario.** It made the 6-DoF stage up to 4.4× faster, and one scenario was solved only with it. Both pipelines reach practically the same optimum (3.3–4.5 % trajectory deviation, ≤ 0.1 kg end-mass difference), and in scenario 8 the warm start found a better UF optimum.
3. **Regularisation makes controls flyable at a measurable cost in optimality.** The cost is much larger for FC than for UF.
4. **Derivatives matter as much as the initial guess.** The published timings use finite-difference Jacobians, whose cost grows as O(N²); exact derivatives scale as O(N). The later solver version uses them ([Derivatives: v1 and v2](derivatives.md)).

## Limitations

- **Real-time target met only by UF in these numbers.** FC takes 4.5–12.9 s with a warm start using the v1 solver. Exact derivatives (now implemented) and a C++ core are the routes to close the gap, but end-to-end v2 timings are not part of this benchmark.
- Open-loop trajectories only; no disturbance rejection.
- Deterministic wind only; single aircraft.
- Aerodynamic model limited to one airframe.
- Benchmark limited to 8 scenarios and 2 repetitions on one machine.
- Local optimality only: IPOPT does not certify global optima.

## Future work

1. Re-run the benchmark with the exact-derivative version (and later the C++ core) to verify the < 4 s target for FC.
2. Closed-loop integration with an MPC layer that uses the offline trajectory as its reference.
3. Stochastic wind and chance-constrained NFZ/altitude constraints.
4. Multi-aircraft coordination with shared airspace.
5. A data-driven aerodynamic surrogate trained on flight-test data.

[← Previous](derivatives.md) · [Home](index.md)
