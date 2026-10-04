# 8. Transcription: trapezoidal collocation and scaling

!!! abstract "TL;DR"
    Split the horizon into $N-1$ equal intervals, replace the ODE by one algebraic **defect** per interval, and scale every residual to order one.

## Defects

With $\Delta t=t_f/(N-1)$ and nodes $k=0,\dots,N-1$:

$$
\Delta_k = x_{k+1}-x_k-\frac{\Delta t}{2}\Big[f(x_k,u_k)+f(x_{k+1},u_{k+1})\Big]=0,\quad k=0,\dots,N-2.
$$

The scheme is second order in $\Delta t$. Because $t_f$ is a decision variable, the same NLP template expresses minimum-time and minimum-fuel problems.

## Decision vector and sparsity

| Formulation | States | Controls | Variables | Equality block |
|---|---|---|---|---|
| UF | 9 | 3 | $1+12N$ | $9(N-1)$ |
| FC | 14 | 4 | $1+18N$ | $14(N-1)$ |

Ordering variables as *all states, then all controls* gives a block-banded Jacobian: defect rows couple only adjacent nodes, and node-wise constraints (terrain, envelope) are block-diagonal.

## Residual normalisation

Rows mix metres, newtons and radians; left alone, the Jacobian is badly conditioned and IPOPT stalls. Every residual is divided by a reference scale:

$$
\tilde r = \operatorname{diag}(s_{ref})^{-1} r .
$$

| Quantity | Reference scale |
|---|---|
| Position | $L_{ref}$ (mission distance) |
| Velocity | $V_{ref}$ (cruise speed / $V_{NE}$) |
| Euler angles | active limit ($\phi_{max}$, $\theta_{max}$), $2\pi$ for yaw |
| Body rates | $2V_{ref}/b$ (roll, yaw), $2V_{ref}/\bar c$ (pitch) |
| Controls | $T_{max}$, $\delta_{max}$ |
| Mass | initial mass |
| Final time | $L_{ref}/V_{ref}$ |

## How it is implemented

```text
function assemble_NLP(model, mission, N):
    z       <- symbolic variables [t_f, X(N), U(N)]
    g, h    <- [], []
    for k in 0 .. N-2:
        g += normalise(defect(model.f, z, k))
    g += boundary_conditions(z, mission)
    h += domain_constraints(z, mission)       # DEM, NFZ, geofence, waypoints
    h += performance_constraints(z, model)    # stall, power, V_NE, rates...
    return NLP(cost(z), g, h, sparsity = block_banded)
```

[← Previous](07-dynamics-fc.md) · [Next: Constraints →](09-constraints.md)
