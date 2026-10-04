# 15. Conclusions

!!! warning "Draft"
    The thesis' own Abstract and Conclusions chapter are still being finalised. This page summarises what the results already support; it will be updated at submission.

## What was delivered

An end-to-end pipeline for 3D minimum-time and minimum-fuel trajectory planning of a fixed-wing aircraft (climb, cruise, descent), with:

- a point-mass (UF) and a 6-DoF rigid-body (FC) formulation,
- trapezoidal direct collocation solved with IPOPT, with residual normalisation,
- terrain, NFZ, geofence, waypoint and flight-envelope constraints,
- a Bolza cost with a time–fuel parameter and smoothness regularisation,
- a Hybrid A\* → inverse dynamics → UF → FC warm-start chain,
- an external validation suite and a warm-start benchmark.

## What the evidence supports

1. A cold start is not viable for constrained missions: UF fails in 3 of 8 scenarios.
2. The full chain converged in every tested scenario and made the 6-DoF stage up to 4.4× faster; one scenario was solved only with it.
3. Regularisation makes controls flyable at a measurable cost in optimality, much larger for FC than for UF.

## Limitations

- **Real-time target met only by UF.** FC takes 4.5–12.9 s with a warm start. A C++ port and exact (analytical) derivatives are the main routes to close the gap; the requirements call for them, but they are not demonstrated in these numbers.
- Open-loop trajectories only; no disturbance rejection.
- Deterministic wind only; single aircraft.
- Aerodynamic model limited to one airframe.
- Benchmark limited to 8 scenarios and 2 repetitions on one machine.
- Local optimality only: IPOPT does not certify global optima.

## Future work

1. Closed-loop integration with an MPC layer using the offline trajectory as reference.
2. Stochastic wind and chance-constrained NFZ/altitude constraints.
3. Multi-aircraft coordination with shared airspace.
4. Data-driven aerodynamic surrogate trained on flight-test data.
5. Speeding up FC below 4 s (analytical Jacobians, C++ core, fewer nodes).

[← Previous](14-results-warmstart.md) · [Home](index.md)
