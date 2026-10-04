# 1. Problem and requirements

!!! abstract "TL;DR"
    Build a 3D trajectory solver for a Mission Management System (MMS) that plans accelerated climb/cruise/descent profiles, respects terrain and no-fly zones, and answers in **under 4 seconds**.

## Intuition

A flight plan for a small aircraft is usually a polyline of waypoints flown at constant speed. That is easy to compute but wasteful: a real aircraft accelerates in the climb, trades altitude for speed, and banks to turn. Taking this into account yields trajectories that are faster or burn less fuel, but turns the planning problem into a large nonlinear optimisation. If, in addition, the aircraft detects a restricted zone in flight, a new trajectory has to be produced *quickly*.

## Reference platform

**Flyox I** (Singular Aircraft): amphibious heavy UAV, MTOW 1,850 kg, payload 850 kg, range 1,200 km. The aircraft used in the validation is the reference airframe MSN012, a full-scale implementation of the platform; its data are in the [MSN012 appendix](appendix-msn012.md).

## Requirements (selected)

| Area | Requirement |
|---|---|
| Dynamics | Coupled 6-DoF model that optimises controls and vehicle dynamics as one system; wind vector field supported |
| Constraints | Terrain (DEM), no-fly zones (NFZ), geofence, waypoints, actuator deflection limits |
| Warm start | Several strategies, selectable per use case; analytical seed (Dubins / straight segments) for recovery |
| Transcription | Independent of the solver: direct collocation and multiple shooting; Euler, RK4 and trapezoidal schemes |
| Performance | Feasible solution within **4.0 s** per tactical look-ahead window; tolerance 1e-4 |
| Interfaces | JSON mission input; WGS-84 geodetic output for autopilots; native Linux SiL execution |
| Safety | Input range checks; fall back to last valid segment or analytical seed on timeout; log iterations and residuals |

## Scope of the thesis

The thesis delivers the numerical core: two dynamic formulations, the NLP transcription, the constraint set, the cost function with regularisation, the warm-start chain, and a validation and benchmarking campaign. Closed-loop control, stochastic wind and multi-aircraft coordination are out of scope (see [Conclusions](15-conclusions.md)).

[← Home](index.md) · [Next: State of the art →](02-state-of-the-art.md)
