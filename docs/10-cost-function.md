# 10. Cost function: time, fuel and smoothness

!!! abstract "TL;DR"
    One Bolza cost with a dial $\alpha\in[0,1]$ from minimum time to minimum fuel, plus a small quadratic penalty on control increments that removes numerical chatter.

## Time–fuel trade-off

$$
J = (1-\alpha)\,w_{t_f}\,t_f+\alpha\sum_{k=0}^{N-2}\bar T_k\,\bar V_k\,\Delta t+J_{pen}
$$

- First term (**Mayer**): flight time, like a race.
- Second term (**Lagrange**): propulsive power $T\cdot V$ integrated over the flight, a proxy for fuel.

| $\alpha$ | Objective | Behaviour |
|---|---|---|
| 0 | Minimum time | Thrust at the limit, tightest turns, high speed |
| 1 | Minimum fuel | Speed near best $L/D$, gentle climb and cruise |
| in between | Pareto front | Compromise |

UF uses the scalar airspeed in $T\cdot V$; FC uses the norm of the body velocity $\lVert V_b\rVert$ (equal in the no-wind validation).

## Why a regularisation term is needed

Two artefacts of direct collocation motivate $J_{pen}$:

1. **Jacobian noise.** With finite-difference derivatives the error floors near $10^{-6}$; the KKT residual oscillates and IPOPT cannot certify convergence. A quadratic term shifts the gradient locally and smooths the search direction.
2. **Control chatter.** The continuous problem has no rate limit, so the optimum may flip controls between bounds at adjacent nodes (bang-bang): pulse-like elevator, saw-tooth thrust that passes collocation but cannot be flown.

$$
J^{FC}_{pen}=\beta_c\sum_{k}\Big[w_{\Delta T}\Delta T_k^2+\sum_{s\in\{e,a,r\}}w_{\Delta\delta_s}\Delta\delta_{s,k}^2\Big]+\beta_s\sum_k\Big[w_{\Delta\phi}\Delta\phi_k^2+w_{\Delta\theta}\Delta\theta_k^2\Big]
$$

(UF penalises $\Delta T,\Delta C_L,\Delta p$ and $\Delta\gamma,\Delta\mu$.) The rudder term prevents the solver from settling on uncoordinated flight.

## Weight selection

Start from zero and raise each weight in geometric steps until (i) total variation $TV(u)=\sum_k|\Delta u_k|$ drops by about an order of magnitude and (ii) the penalty stays below **~15 %** of the total cost, so the physical optimum is not displaced.

[← Previous](09-constraints.md) · [Next: Warm start →](11-warm-start.md)
