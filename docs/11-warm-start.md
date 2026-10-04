# 11. Warm-start pipeline

!!! abstract "TL;DR"
    Hybrid A\* gives a turn-radius-feasible 2D path; inverse dynamics lifts it to a full state–control guess; the UF solution is projected analytically to seed FC. Each stage cuts the work of the next one.

## Why the initial guess matters

An interior-point method first pushes the iterate into the interior of the feasible set. A linear-interpolation guess produces collocation defects several orders of magnitude above tolerance, and the solver spends many iterations (or fails) in a restoration phase. A good guess starts the solver near the feasible manifold.

```mermaid
flowchart LR
    A[Mission JSON] --> B[1 Dispatcher]
    B --> C[2a Hybrid A*]
    C --> D[2b Inverse dynamics]
    D -->|ws.uncoupled| E[3 UF NLP]
    E -->|fine_UF| F[4 FC NLP]
    F -->|fine_FC| G[5-6 Export]
    E -.UF-only bypass.-> G
```

## Stage 2a: Hybrid A\*

Searches the plane $(x,y,\psi)$ on an occupancy grid (25–50 m cells) that contains the NFZs. Nodes expand along unicycle arcs with $R\in\{-R_{min},\infty,+R_{min}\}$ where

$$
R_{min}=\frac{V_{cruise}^2}{g\tan\phi_{active}} .
$$

Cost $f=g+h$, with $h$ the **maximum** of the Euclidean distance and the Dubins lower bound (shortest bounded-curvature path, one of LSL, LSR, RSL, RSR, LRL, RLR). Both are admissible, so the maximum is too.

```text
function hybrid_astar(start, goal, grid, R_min):
    open <- priority_queue{start};  closed <- {}
    while open not empty:
        n <- open.pop_min(f = g + max(euclid(n,goal), dubins(n,goal,R_min)))
        if near(n, goal): return reconstruct(n)
        for R in (-R_min, inf, +R_min):
            child <- integrate_arc(n, R, ds)           # closed form
            if blocked(grid, child) or cell(child) in closed: continue
            g_child <- g(n) + ds + turn_switch_penalty
            open.push(child)
        closed.add(cell(n))
```

## Stage 2b: Inverse dynamics

1. Resample the path to $N$ arc-length nodes; $\Delta t=L/(V_0(N-1))$ (constant speed).
2. Heading and flight-path angle by finite differences.
3. Coordinated turn: $\phi=\arctan(V_0\dot\chi/g)$, $q=\dot\gamma$, $r=\dot\chi\cos\gamma$, $p=\dot\phi-r\tan\gamma$; clamp to rate limits.
4. Per node, a Newton **trim** solve gives $T,\delta_e,(u,v,w)$ satisfying the 6-DoF force and moment balance.
5. Integrate the mass forward.

```text
function inverse_dynamics(path, V0, N):
    r      <- resample_by_arclength(path, N)
    chi, gamma <- finite_differences(r)
    phi, p, q, r_rate <- coordinated_turn(V0, chi, gamma);  clamp_to_limits()
    for k in 0 .. N-1:
        T[k], de[k], uvw[k] <- newton_trim(phi[k], gamma[k], rates[k], m[k])
        m[k+1] <- m[k] - fuel_burn(T[k], V0) * dt
    return state_control_guess
```

Nodes are trimmed independently, so the result is not optimal, but its defect is typically 2–3 orders of magnitude below a linear guess.

## Stage 4: UF → FC projection

From $(V,\mu,\gamma,\xi)$ the angle of attack follows from the equilibrium lift coefficient $C_{L,eq}=2mg/(\rho V^2S_w)$ and the linearised lift curve, giving $\theta=\gamma+\alpha$. Then $(\phi,\theta,\psi)=(\mu,\theta,\xi)$; body velocities by the aerodynamic-to-body rotation; rates from the coordinated-turn relations; elevator from longitudinal trim.

## Export

Stages 5–6 compress the fine trajectory geometrically (cone / line-of-sight algorithm) and write JSON for the autopilot.

[← Previous](10-cost-function.md) · [Next: Validation results →](12-results-validation.md)
