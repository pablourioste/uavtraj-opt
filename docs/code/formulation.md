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
| `nfz:<id>` (one per zone) | $(m_j-d_j(x,y))/\max(m_j,25)$ if the zone's altitude band overlaps the mission's (start to end altitude), otherwise $-1$; $d_j$ = signed distance to the polygon, $m_j$ = margin |

The zone's activity is decided once per mission, the same rule the planner uses, rather than per node: a per-node switch would make the row discontinuous in altitude, which a gradient-based solver cannot handle.

## Cost: Bolza form with a time–fuel dial

$$J=(1-\alpha)\frac{t_f}{t_{ref}}+\alpha\,\frac{\sum_k \bar T_k\bar V_k\,\Delta t}{E_{ref}}+\frac{1}{N-1}\sum_k\Big[\beta_c\sum_j\Big(\frac{\Delta u_{j,k}}{s_{u_j}}\Big)^2+\beta_s\sum_{i\in\{\gamma,\mu\}}\Big(\frac{\Delta x_{i,k}}{s_{x_i}}\Big)^2\Big]$$

- α = 0 gives minimum time and α = 1 minimum propulsive energy (a fuel proxy).
- The β terms penalise control and attitude increments, the two families of [10. Cost function](../10-cost-function.md). Without them a minimum-fuel optimum is bang-bang: thrust flips between zero and its limit, and the aircraft follows a saw-tooth glide–climb profile.

`costGradient` is analytic, and the tests check it against finite differences.

## Initial guesses

- `coldStartGuess`: a straight line at constant speed with zero controls. This is the linear-interpolation guess against which the thesis benchmarks its warm start. Its defects are O(1).
- `trimmedLineGuess`: the same geometry, but each node carries the UF trim controls for the local flight-path angle, and the mass history is integrated consistently. Its defects are at machine precision.

- `scenario::fly` (UC-06): a trajectory flown in closed loop by a guidance law, then resampled on the collocation nodes. It satisfies the dynamics up to the discretisation error of the mesh.

Neither straight-line guess bends around obstacles or meets a terminal heading that differs from the bearing. With a level start, the straight climbing line also violates the initial flight-path angle; UC-04 reports this as a nonzero boundary residual. The warm start below handles those cases: a Hybrid A\* path flown by the guidance laws, in the spirit of the thesis's [Hybrid A\* → inverse-dynamics chain](../11-warm-start.md).

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

## Optimal-control problems: `src/formulation/ocp_*.cpp`

`CollocationProblem` turns a collocated model into a complete nonlinear program. Its constraint vector, with $g_l\le g(z)\le g_u$, is

$$g(z)=\big[\;\Delta\;(=0)\;\big|\;\text{boundary}\;(=0)\;\big|\;h(x_k,u_k)\;(\le0)\;\big|\;\text{waypoints}\;(\le0)\;\big].$$

- **Boundary.** The whole initial state is fixed: position, speed, flight-path angle 0, heading, wings level and mass m₀ for UF; the wings-level 6-DoF trim at the start for FC. At the end, the position and the heading (UF) or yaw (FC) are fixed, the angle matched modulo 2π.
- **Waypoints.** Each waypoint is pinned to the node of the initial guess nearest to it, visiting them in order, and that node must lie within the tolerance: $\big((x_k-x_w)^2+(y_k-y_w)^2\big)/r_w^2-1\le0$. This is simpler than the thesis's arc-length slack formulation. The price is that the share of flight time before each waypoint follows the guess.
- **Bounds.** Boxes keep every function defined (airspeed above 0.3 V_cruise, pitch within ±60° away from the Euler singularity, mass between m_empty and m₀). FC actuator deflections are bounded at their real limits. The other limits are path rows.
- **Jacobian.** The defect block comes from `trapezoidalJacobian`. UF supplies its analytic model Jacobians; FC supplies central differences of `fc::rhs`, scaled per variable, with the structural sparsity detected once. The path rows of node k depend only on $(x_k,u_k)$, so each node contributes a dense $n_c\times(n_x+n_u)$ block, filled by central differences. Boundary and waypoint rows are analytic. The tests compare the whole Jacobian with a dense central-difference one.
- **References.** Variable scales, the cost's $t_{ref}$ and $E_{ref}$ and the bounds on $t_f$ come from the mission: the length $L_{ref}$ of the polyline start → waypoints → goal and $L_{ref}/V$. Cold and warm starts therefore solve the same problem.

**FC path rows** ($h\le0$, normalised): stall and V_NE on the air-relative speed, power, incidence limits, **sideslip** $\beta^2/\beta_{max}^2-1$, load factor $qSC_L/(mg)$, altitude, bank, pitch limits, the three body rates, thrust sign, and one `nfz:<id>` row per zone.

The sideslip row is needed because the linear model's polar $C_D=C_{D0}+kC_L^2$ has no sideslip drag. Without the row, the minimum-fuel FC optimum flies 15–27° sideways with the rudder saturated, and banks so that the side force $C_{Y\beta}\beta$ carries part of the weight at no drag cost. It then appears to burn 6–9 % less fuel than the point mass, which is impossible for the same aircraft. With $\beta_{max}=1°$ (coordinated flight), UF and FC agree to 0.2 %.

**FC cost.** $J=(1-\alpha)\,t_f/t_{ref}+\alpha\,(m_0-m_{N-1})/m_{ref}$ plus the smoothness terms on $\Delta T,\Delta\delta_{e,a,r}$ and $\Delta\phi,\Delta\theta$. Fuel is read from the mass state, which integrates $\dot m=-c\,T V_a$, the same quantity as the UF energy integral up to the factor $c$. Its reference is $m_{ref}=c\,E_{ref}$, so UF and FC costs are on the same scale.

**No-fly zones between nodes.** The zone rows hold at the nodes. A chord of length ℓ whose two ends keep distance r from a zone corner stays at least $\sqrt{r^2-\ell^2/4}$ from it. The pipeline therefore enforces $r=\sqrt{m^2+(\ell/2)^2}$, with ℓ the node spacing of a path 50 % longer than $L_{ref}$, and the whole straight segment between nodes stays outside the real margin m. UC-08 checks this along the segments.

## UF → FC projection: `src/warmstart/warm_start.cpp`

`ws::ufToFc` maps a point-mass trajectory to rigid-body states and controls, node by node:

1. **Incidence and elevator.** For each thrust, lift and pitch-moment balance are linear in $(\alpha,\delta_e)$:
   $C_{L\alpha}\alpha+C_{L\delta_e}\delta_e=C_L-C_{L0}-C_{Lq}\hat q-T\sin\alpha/(qS)$ and $C_{m\alpha}\alpha+C_{m\delta_e}\delta_e=-(C_{m0}+C_{mq}\hat q+z_TT/(qS\bar c))$.
   The thrust is $T_{UF}/\cos\alpha$, because the point mass pushes along the velocity and the rigid body along its x axis. Three fixed-point passes on α converge.
2. **Attitude.** $R_{nb}=R_{nw}(\mu,\gamma,\chi)\,R_{wb}(\alpha)$ with zero sideslip; the Euler angles are extracted from $R_{nb}$, and ψ is unwrapped next to χ.
3. **Rates.** $p,q,r$ come from the time derivatives of the Euler angles (central differences).
4. **Velocity.** $(V\cos\alpha,0,V\sin\alpha)$ plus the wind in body axes.
5. **Aileron and rudder.** They zero the roll and yaw moments at zero sideslip (a 2 × 2 solve).

On a trimmed straight line, the projection reproduces the 6-DoF trim to five digits (θ, δ_e, w), with a residual pitch acceleration of about 10⁻¹⁶ rad/s²; a test checks this.

## Sequence: `src/pipeline/pipeline.cpp`

`pipeline::run` executes, for one mission:

1. **Warm start** (`ws::build`): Hybrid A\* path; UF and FC runs flown by the guidance.
2. **UF stage.** The UF run is resampled on N nodes (or a straight line, for a cold start), the problem is built with the mission references, and IPOPT solves it.
3. **FC stage.** It is seeded by the projected UF optimum, or, without a UF stage, by the run flown by the FC autopilot. Then IPOPT solves it.
4. **Verification.** `nlp::verify` checks each optimum against every constraint and bound. The stage records its status, iterations, time, objective, the defect of its initial point, and the share of the cost due to smoothing.

[← Architecture](architecture.md) · [Next: Use cases →](use-cases.md)
