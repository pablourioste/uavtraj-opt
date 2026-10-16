# Formulation in code

The equations implemented in `code/`, with pointers to the source files. They are the generic counterparts of chapters [6](../06-dynamics-uf.md)–[10](../10-cost-function.md). The state ordering and some modelling details differ on purpose from the production solver.

## Atmosphere and propulsion: `src/model/atmosphere.cpp`, `aircraft.cpp`

- **Atmosphere.** ISA troposphere: $T=T_0-Lh$, $p=p_0(T/T_0)^{g/RL}$, $\rho=p/RT$.
- **Propulsion.** A constant-power propeller. The shaft power lapses with altitude following Gagg–Ferrar, $P_{max}(h)=P_{max,0}\,(1.132\,\sigma-0.132)$, and the available thrust is $T_{max}=\eta_p P_{max}(h)/\max(V,V_{stall})$.

## Point mass (UF): `src/model/point_mass.cpp`

State and control:

$$x=[x_N,\,y_E,\,z_D,\,V,\,\gamma,\,\chi,\,\mu,\,m]^\top,\qquad u=[T,\,C_L,\,p_\mu]^\top$$

$$
\begin{aligned}
\dot x_N &= V\cos\gamma\cos\chi + w_N, &\dot y_E &= V\cos\gamma\sin\chi + w_E, &\dot z_D &= -V\sin\gamma + w_D,\\
\dot V &= \frac{T-D}{m}-g\sin\gamma, &\dot\gamma &= \frac{L\cos\mu}{mV}-\frac{g\cos\gamma}{V}, &\dot\chi &= \frac{L\sin\mu}{mV\cos\gamma},\\
\dot\mu &= p_\mu, &\dot m &= -\frac{\mathrm{sfc}}{3.6\times10^6}\,\frac{TV}{\eta_p}
\end{aligned}
$$

with $L=\tfrac12\rho V^2 S C_L$, $D=\tfrac12\rho V^2 S (C_{D0}+kC_L^2)$ and ρ(h) from the ISA. The lift coefficient is a control (time-scale separation). `uf::jacobian` returns ∂f/∂x and ∂f/∂u in closed form. It includes dρ/dh through the altitude dependence, which a finite-difference Jacobian only captures approximately.

## Rigid body (FC): `src/model/rigid_body.cpp`

$$x=[x_N,y_E,z_D,\,u,v,w,\,\phi,\theta,\psi,\,p,q,r,\,m]^\top,\qquad u=[T,\delta_e,\delta_a,\delta_r]^\top$$

Evaluation order at each call:

1. Compute $R_{nb}(\phi,\theta,\psi)$, remove the NED wind in body axes, and get $V_a$, $\alpha=\operatorname{atan2}(w_r,u_r)$, $\beta=\arcsin(v_r/V_a)$.
2. Coefficients from linear stability derivatives with non-dimensional rates $\hat p=pb/2V_a$, $\hat q=q\bar c/2V_a$, $\hat r=rb/2V_a$:
   $C_L=C_{L0}+C_{L\alpha}\alpha+C_{Lq}\hat q+C_{L\delta_e}\delta_e$, $C_D=C_{D0}+kC_L^2$, and $C_Y$, $C_l$, $C_m$, $C_n$ in the same way.
3. Rotate the wind-axis forces $[-D,\,Y,\,-L]$ to body axes, then add weight and thrust (along body x).
4. Moments: aerodynamic, plus the thrust-line offset $(0,\;z_T T,\;0)$.
5. Integrate:
   - $\dot{\mathbf v}_b = \mathbf F/m - \boldsymbol\omega\times\mathbf v_b$
   - $I\dot{\boldsymbol\omega}=\mathbf M-\boldsymbol\omega\times I\boldsymbol\omega$ (with $I_{xz}$, solved in closed form)
   - the Euler-angle kinematics and $\dot{\mathbf r}_{NED}=R_{nb}\mathbf v_b$.

## Trapezoidal collocation: `src/formulation/transcription.cpp`

$$\Delta_k=\operatorname{diag}(s_x)^{-1}\Big[x_{k+1}-x_k-\tfrac{\Delta t}{2}\big(f(x_k,u_k)+f(x_{k+1},u_{k+1})\big)\Big],\qquad \Delta t=\frac{t_f}{N-1}$$

Jacobian blocks for interval k (row-scaled by $s_x^{-1}$):

| w.r.t. | block |
|---|---|
| $x_k$ | $-I-\tfrac{\Delta t}{2}A_k$ |
| $x_{k+1}$ | $I-\tfrac{\Delta t}{2}A_{k+1}$ |
| $u_k,\;u_{k+1}$ | $-\tfrac{\Delta t}{2}B_k,\;-\tfrac{\Delta t}{2}B_{k+1}$ |
| $t_f$ | $-\tfrac{1}{2(N-1)}\big(f_k+f_{k+1}\big)$ |

Only structurally nonzero entries of $A$ and $B$ are stored. The tests verify the whole Jacobian against central differences, and check that the defect of a smooth reference trajectory shrinks like $\Delta t^3$ (second-order global accuracy).

## Constraints

**Boundary.** The full initial state is fixed. At the end, position and heading are fixed.

**Path** (per node, $h\le0$):

| name | expression |
|---|---|
| `stall` | $1-V/V_{stall}(h,m)$ |
| `power` | $TV/(\eta_p P_{max}(h))-1$ |
| `v_ne` | $V/V_{NE}-1$ |
| `load_max`, `load_min` | $n/n_{max}-1$, $(n_{min}-n)/\max(1,\lvert n_{min}\rvert)$ |
| `altitude` | $(h_{min}-h)/h_{min}$ |
| `bank`, `bank_rate` | $\mu^2/\phi_{max}^2-1$, $p_\mu^2/p_{max}^2-1$ (smooth two-sided bounds) |
| `gamma_max`, `gamma_min` | flight-path-angle limits |
| `cl_max`, `thrust_min` | $C_L/C_{L,max}-1$, $-T/T_{ref}$ |
| `nfz:<id>` (one per zone) | $(m_j-d_j(x,y))/\max(m_j,25)$ if the node's altitude is inside the zone's band, otherwise $-1$; $d_j$ = signed distance to the polygon, $m_j$ = margin |

## Cost: Bolza form with a time–fuel dial

$$J=(1-\alpha)\frac{t_f}{t_{ref}}+\alpha\,\frac{\sum_k \bar T_k\bar V_k\,\Delta t}{E_{ref}}+\frac{\beta}{N-1}\sum_k\sum_j\Big(\frac{u_{j,k+1}-u_{j,k}}{s_{u_j}}\Big)^2$$

- α = 0 gives minimum time and α = 1 minimum propulsive energy (a fuel proxy).
- The β term penalises control increments, which suppresses node-to-node chatter.

`costGradient` is analytic, and the tests check it against finite differences.

## Initial guesses

- `coldStartGuess`: a straight line at constant speed with zero controls. This is the linear-interpolation guess against which the thesis benchmarks its warm start. Its defects are O(1).
- `trimmedLineGuess`: the same geometry, but each node carries the UF trim controls for the local flight-path angle, and the mass history is integrated consistently. Its defects are at machine precision.

- `scenario::fly` (UC-06): a trajectory flown in closed loop by a guidance law, then resampled on the collocation nodes. It satisfies the dynamics up to the discretisation error of the mesh.

Neither straight-line guess bends around obstacles or meets a terminal heading that differs from the bearing. With a level start, the straight climbing line also violates the initial flight-path angle; UC-04 reports this as a nonzero boundary residual. Handling those cases is what the thesis's [Hybrid A\* → inverse-dynamics chain](../11-warm-start.md) does, and it is not part of this edition.

## Design-scenario guidance: `src/scenarios/scenario.cpp`

Both models share one guidance law:

$$\gamma_{cmd}=\operatorname{sat}_{\gamma_{max}}\big(k_h(h_{cmd}-h)\big),\qquad \mu_{cmd}=\operatorname{sat}_{\mu_{max}}\big(k_\chi(\chi_{cmd}-\chi)\big)$$

- **UF (dynamic inversion).** The point-mass equations are inverted for the controls:
  - $C_L$ from $L\cos\mu=m(V\dot\gamma_{des}+g\cos\gamma)$, with $\dot\gamma_{des}=k_\gamma(\gamma_{cmd}-\gamma)$;
  - $T=D+mg\sin\gamma+mk_V(V_{cmd}-V)$;
  - $p_\mu=k_\mu(\mu_{cmd}-\mu)$.

  Each control is clipped to $C_{L,max}$, $T_{max}$ and $p_{max}$.
- **FC (autopilot around the wings-level trim).**
  - Pitch: $\theta_{cmd}=\alpha_{trim}+C_{L,trim}(n-1)/C_{L\alpha}+\gamma_{cmd}$, held with $\delta_e=\delta_{e,trim}+k_\theta(\theta-\theta_{cmd})+k_q q$.
  - Roll: $\delta_a=k_\phi(\mu_{cmd}-\phi)-k_p p$.
  - Turn coordination: $\delta_r=-k_\beta\beta$.
  - Speed: thrust from the trim value plus climb and speed terms.

The guidance is deliberately simple. Its purpose is to produce realistic, flyable reference trajectories for the design scenarios, not to be optimal.

## Missions with no-fly zones: `src/model/environment.cpp`, `src/planning/planner.cpp`

**No-fly zones.** A zone is a simple polygon in the horizontal plane, extruded between $h_{min}$ and $h_{max}$, with a margin $m$. Its signed distance $d(x,y)$ is the distance to the nearest edge: positive outside, negative inside (crossing-number test). A zone only counts for a flight whose altitude band overlaps $[h_{min},h_{max}]$, so a low zone can be overflown.

**Dubins paths.** Between two poses $(x,y,\psi)$, the shortest forward path with turn radius $R$ is one of six words: LSL, LSR, RSL, RSR, RLR, LRL. Each word is computed in closed form, and the tests check that every word ends exactly at the goal pose.

**Hybrid A\*.** The search runs over $(x,y,\psi)$ cells (20 m, 5°).

- **Edges.** Five 30 m primitives with curvature $\{-1,-\tfrac12,0,\tfrac12,1\}/R$, so every path is flyable at the planning radius $R=V^2/(g\tan(0.8\,\mu_{max}))$. The 0.8 leaves bank for the guidance to correct errors.
- **Cost.** Length, plus a small penalty for curvature and for changes of steering.
- **Heuristic.** $\max(\text{Euclidean},\ \text{Dubins length})$. Both are admissible lower bounds, so the maximum is too.
- **Collisions.** Points every 5 m must keep $d\ge m+$ safety from every active zone.
- **Analytic expansion.** Every few expansions, and whenever the goal is within $4R$, a Dubins shot to the goal is tried. If it is collision-free it closes the search.
- **Waypoints.** The mission is planned leg by leg. Each waypoint's heading points to the next target.

**L1 path following.** The guidance finds the nearest point of the path, searching forward only so that loops are not skipped, and aims at a point $L_1=2.5\,V$ further along. With $\eta$ the angle between the course and the line to that point, the commanded lateral acceleration and bank are

$$a_{cmd}=\frac{2V^2}{L_1}\sin\eta,\qquad \mu_{cmd}=\operatorname{sat}_{\mu_{max}}\big(\arctan(a_{cmd}/g)\big).$$

The same command drives the UF model (through $p_\mu$) and the FC autopilot (through $\delta_a$). Altitude is handled by the guidance law of the previous section.

[← Architecture](architecture.md) · [Next: Use cases →](use-cases.md)
