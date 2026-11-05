# 7. Mathematical formulation

This chapter states the complete optimisation problem that the public code solves: the models, every constraint, the bounds, the cost, the discretisation and the scaling. Each section names the source file that implements it. The thesis chapters [5](../05-ocp-theory.md) to [10](../10-cost-function.md) give the theory and the production formulation; this chapter describes the code in `code/`, which differs from the production solver in a few places (section 7.11).

## 7.1 The optimal-control problem

Find the flight time $t_f$, the state history $x(t)$ and the control history $u(t)$ that solve

$$
\begin{aligned}
\min_{t_f,\;x(\cdot),\;u(\cdot)}\quad & J = (1-\alpha)\,\frac{t_f}{t_{ref}} + \alpha\,\frac{\text{fuel}(t_f)}{\text{fuel}_{ref}} + J_{smooth}\\[4pt]
\text{subject to}\quad & \dot x(t) = f\big(x(t),u(t)\big), && 0 \le t \le t_f\\
& x(0) = x_{start}\\
& \psi_f\big(x(t_f)\big) = 0 && \text{final position and heading}\\
& h\big(x(t),u(t)\big) \le 0 && \text{envelope and no-fly zones}\\
& \lVert p(t_w) - p_w\rVert \le r_w && \text{each waypoint } w\\
& x_{lo}\le x(t)\le x_{hi},\quad u_{lo}\le u(t)\le u_{hi},\quad t_{f,lo}\le t_f\le t_{f,hi}
\end{aligned}
$$

where $f$ is the point-mass model (UF, section 7.3) or the rigid-body model (FC, section 7.4), $\alpha\in[0,1]$ weighs time against fuel, and $J_{smooth}$ penalises control and attitude increments between nodes. The problem is transcribed into a finite-dimensional nonlinear program (NLP) by trapezoidal collocation (section 7.8) and solved with IPOPT (chapter 8).

## 7.2 Atmosphere, aerodynamics and propulsion

Implemented in [`atmosphere.cpp`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/src/model/atmosphere.cpp) and [`aircraft.cpp`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/src/model/aircraft.cpp).

**Atmosphere.** International Standard Atmosphere, troposphere, valid from -500 m to 11 km (altitudes outside are clamped):

$$
T = T_0 - \lambda h,\qquad p = p_0\left(\frac{T}{T_0}\right)^{g_0/(R\lambda)},\qquad \rho = \frac{p}{RT},\qquad \sigma=\frac{\rho}{\rho_0}
$$

with $T_0 = 288.15$ K, $p_0 = 101325$ Pa, $\lambda = 0.0065$ K/m, $R = 287.053$ J/(kg K), $g_0 = 9.80665$ m/s².

**Aerodynamics.** With the dynamic pressure $\bar q = \tfrac12\rho V^2$ and the wing area $S$, lift and drag are $L=\bar q S C_L$ and $D=\bar q S C_D$, with the parabolic polar

$$
C_D = C_{D0} + k\,C_L^2 .
$$

The maximum lift coefficient and the stall speed are derived from the aircraft data:

$$
C_{L,max} = C_{L0} + C_{L\alpha}\,\alpha_{max},\qquad V_{stall}(h,m) = \sqrt{\frac{2mg_0}{\rho(h)\,S\,C_{L,max}}} .
$$

**Propulsion.** A propeller driven by a naturally aspirated engine whose shaft power lapses with density (Gagg–Ferrar):

$$
P_{max}(h) = P_{max,0}\,\max\big(0,\;1.132\,\sigma - 0.132\big),\qquad
T_{max}(V,h) = \frac{\eta_p\,P_{max}(h)}{\max\big(V,\,V_{stall}(h,m_0)\big)} .
$$

**Fuel flow.** The specific fuel consumption $\text{sfc}$ (kg/kWh) refers to shaft power, and the shaft power is the thrust power divided by the propeller efficiency:

$$
\dot m = -c\,T\,V,\qquad c = \frac{\text{sfc}}{3.6\times10^{6}\,\eta_p}\quad[\text{kg/J}].
$$

## 7.3 Point-mass model (UF)

Implemented in [`point_mass.cpp`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/src/model/point_mass.cpp) (`uf::rhs`, and the analytic Jacobian `uf::jacobian`).

| State | Symbol | Unit | | Control | Symbol | Unit |
|---|---|---|---|---|---|---|
| North position | $x_N$ | m | | Thrust | $T$ | N |
| East position | $y_E$ | m | | Lift coefficient | $C_L$ | – |
| Down position ($-h$) | $z_D$ | m | | Bank rate | $p_\mu$ | rad/s |
| Airspeed | $V$ | m/s | | | | |
| Flight-path angle | $\gamma$ | rad | | | | |
| Heading | $\chi$ | rad | | | | |
| Bank angle | $\mu$ | rad | | | | |
| Mass | $m$ | kg | | | | |

$$
\begin{aligned}
\dot x_N &= V\cos\gamma\cos\chi, & \dot y_E &= V\cos\gamma\sin\chi, & \dot z_D &= -V\sin\gamma,\\
\dot V &= \frac{T-D}{m} - g_0\sin\gamma, & \dot\gamma &= \frac{L\cos\mu}{mV} - \frac{g_0\cos\gamma}{V}, & \dot\chi &= \frac{L\sin\mu}{mV\cos\gamma},\\
\dot\mu &= p_\mu, & \dot m &= -c\,T\,V .
\end{aligned}
$$

The thrust acts along the velocity, and the lift coefficient is a control: the incidence dynamics are much faster than the trajectory, so the aircraft is assumed to reach any lift coefficient instantly. The bank angle is a state, driven by the bank rate, so that bank changes take time. The load factor is $n = L/(mg_0)$. The model supports a constant wind (added to the position rates), which is zero in UC-08 and UC-09.

`uf::jacobian` returns $A=\partial f/\partial x$ and $B=\partial f/\partial u$ in closed form, including the altitude dependence of the density, $\partial\rho/\partial h = \rho\,(g_0/(R\lambda)-1)\,(-\lambda/T)$.

## 7.4 Rigid-body model (FC)

Implemented in [`rigid_body.cpp`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/src/model/rigid_body.cpp) (`fc::rhs`).

| State | Symbol | Unit | | Control | Symbol | Unit |
|---|---|---|---|---|---|---|
| Position (NED) | $x_N, y_E, z_D$ | m | | Thrust | $T$ | N |
| Body velocity | $u, v, w$ | m/s | | Elevator | $\delta_e$ | rad |
| Euler angles (roll, pitch, yaw) | $\phi, \theta, \psi$ | rad | | Aileron | $\delta_a$ | rad |
| Body rates | $p, q, r$ | rad/s | | Rudder | $\delta_r$ | rad |
| Mass | $m$ | kg | | | | |

Each evaluation proceeds in five steps.

**1. Air data.** With $R_{nb}(\phi,\theta,\psi)$ the body-to-NED rotation (3-2-1 Euler sequence) and $\mathbf w$ the wind in NED axes, the air-relative velocity in body axes is $\mathbf v_r = (u,v,w)^\top - R_{nb}^\top\mathbf w$, and

$$
V_a = \lVert\mathbf v_r\rVert,\qquad \alpha = \operatorname{atan2}(w_r,\,u_r),\qquad \beta = \arcsin(v_r/V_a).
$$

**2. Aerodynamic coefficients.** Linear stability derivatives, with non-dimensional rates $\hat p = pb/(2V_a)$, $\hat q = q\bar c/(2V_a)$, $\hat r = rb/(2V_a)$:

$$
\begin{aligned}
C_L &= C_{L0} + C_{L\alpha}\alpha + C_{Lq}\hat q + C_{L\delta_e}\delta_e, & C_D &= C_{D0} + k\,C_L^2,\\
C_Y &= C_{Y0} + C_{Y\beta}\beta + C_{Yp}\hat p + C_{Yr}\hat r + C_{Y\delta_a}\delta_a + C_{Y\delta_r}\delta_r,\\
C_l &= C_{l0} + C_{l\beta}\beta + C_{lp}\hat p + C_{lr}\hat r + C_{l\delta_a}\delta_a + C_{l\delta_r}\delta_r,\\
C_m &= C_{m0} + C_{m\alpha}\alpha + C_{mq}\hat q + C_{m\delta_e}\delta_e,\\
C_n &= C_{n0} + C_{n\beta}\beta + C_{np}\hat p + C_{nr}\hat r + C_{n\delta_a}\delta_a + C_{n\delta_r}\delta_r .
\end{aligned}
$$

**3. Forces.** The wind-axis forces $(-D,\,Y,\,-L)$, with $D=\bar qSC_D$, $Y=\bar qSC_Y$, $L=\bar qSC_L$, are rotated to body axes by $\alpha$ and $\beta$. Weight and thrust (along the body $x$ axis) are added:

$$
\mathbf F = R_{bw}(\alpha,\beta)\begin{pmatrix}-D\\Y\\-L\end{pmatrix} + m g_0\begin{pmatrix}-\sin\theta\\ \sin\phi\cos\theta\\ \cos\phi\cos\theta\end{pmatrix} + \begin{pmatrix}T\\0\\0\end{pmatrix}.
$$

**4. Moments.** Aerodynamic moments plus the pitching moment of a thrust line offset $z_T$ below the centre of gravity:

$$
\mathbf M = \big(\bar qSb\,C_l,\;\; \bar qS\bar c\,C_m + z_T T,\;\; \bar qSb\,C_n\big)^\top .
$$

**5. Equations of motion.**

$$
\begin{aligned}
\dot{\mathbf v}_b &= \frac{\mathbf F}{m} - \boldsymbol\omega\times\mathbf v_b, &
I\dot{\boldsymbol\omega} &= \mathbf M - \boldsymbol\omega\times I\boldsymbol\omega,\\
\dot{\mathbf r}_{NED} &= R_{nb}\,\mathbf v_b, &
\dot m &= -c\,T\,V_a,
\end{aligned}
$$

$$
\dot\phi = p + \tan\theta\,(q\sin\phi + r\cos\phi),\qquad
\dot\theta = q\cos\phi - r\sin\phi,\qquad
\dot\psi = \frac{q\sin\phi + r\cos\phi}{\cos\theta},
$$

with the inertia tensor $I$ including the product $I_{xz}$, inverted in closed form. The Euler kinematics are singular at $\theta=\pm90°$; the pitch bound of section 7.6 keeps the solver away from it.

The FC Jacobian is computed by central differences of `fc::rhs`, with a step proportional to each variable's scale. Its structural sparsity is detected once, at a generic point, and only those entries are passed to the solver.

### Coordinated flight and the sideslip limit

The parabolic polar has no drag due to sideslip. Without a limit on $\beta$, the minimum-fuel 6-DoF optimum exploits this: it flies 15–27° sideways with the rudder saturated, so that the side force $C_{Y\beta}\beta$ carries part of the weight at no drag cost, and appears to burn 6–9 % less fuel than the point mass, which is physically impossible for the same aircraft. The FC problem therefore includes the path constraint $\beta^2/\beta_{max}^2-1\le0$, with $\beta_{max}$ = `sideslip_max_deg` (1° by default). With it, the UF and FC optima agree to 0.2 % in fuel.

## 7.5 Path constraints

Path constraints are evaluated at every node $k$ and written as $h(x_k,u_k)\le0$. Each row is normalised by its limit, so that a value of 0.01 means "1 % beyond the limit" whatever the physical quantity. The rows are built in `TrapezoidalUF::pathAt` ([`transcription.cpp`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/src/formulation/transcription.cpp)) and `FcProblem::pathAt` ([`ocp_fc.cpp`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/src/formulation/ocp_fc.cpp)).

### Point mass: 12 rows per node, plus one per zone

| Label | Row $h\le0$ | Meaning |
|---|---|---|
| `stall` | $1 - V/V_{stall}(h,m)$ | Above the stall speed |
| `power` | $TV/(\eta_p P_{max}(h)) - 1$ | Thrust power within the available power |
| `v_ne` | $V/V_{NE} - 1$ | Below the never-exceed speed |
| `load_max` | $n/n_{max} - 1$ | Positive load-factor limit |
| `load_min` | $(n_{min} - n)/\max(1,\lvert n_{min}\rvert)$ | Negative load-factor limit |
| `altitude` | $(h_{min} - h)/\max(1,h_{min})$ | Above the minimum altitude |
| `bank` | $\mu^2/\phi_{max}^2 - 1$ | Bank within $\pm\phi_{max}$ |
| `gamma_max` | $(\gamma - \gamma_{max})/\lvert\gamma_{max}\rvert$ | Climb-angle limit |
| `gamma_min` | $(\gamma_{min} - \gamma)/\lvert\gamma_{min}\rvert$ | Descent-angle limit |
| `cl_max` | $C_L/C_{L,max} - 1$ | Lift coefficient below its maximum |
| `thrust_min` | $-T/T_{ref}$ | Non-negative thrust |
| `bank_rate` | $p_\mu^2/p_{max}^2 - 1$ | Bank rate within the roll-rate limit |
| `nfz:<id>` | see below | One row per zone |

### Rigid body: 16 rows per node, plus one per zone

| Label | Row $h\le0$ | Meaning |
|---|---|---|
| `stall` | $1 - V_a/V_{stall}(h,m)$ | Above the stall speed |
| `power` | $TV_a/(\eta_p P_{max}(h)) - 1$ | Thrust power within the available power |
| `v_ne` | $V_a/V_{NE} - 1$ | Below the never-exceed speed |
| `alpha_max` | $(\alpha - \alpha_{max})/\max(\lvert\alpha_{max}\rvert, 0.05)$ | Incidence limit |
| `alpha_min` | $(\alpha_{min} - \alpha)/\max(\lvert\alpha_{min}\rvert, 0.05)$ | Negative incidence limit |
| `sideslip` | $\beta^2/\beta_{max}^2 - 1$ | Coordinated flight |
| `load_max` | $n/n_{max} - 1$, with $n = \bar qSC_L/(mg_0)$ | Positive load-factor limit |
| `load_min` | $(n_{min} - n)/\max(1,\lvert n_{min}\rvert)$ | Negative load-factor limit |
| `altitude` | $(h_{min} - h)/\max(1,h_{min})$ | Above the minimum altitude |
| `bank` | $\phi^2/\phi_{max}^2 - 1$ | Roll angle within $\pm\phi_{max}$ |
| `theta_max`, `theta_min` | $(\theta - \theta_{max})/\lvert\theta_{max}\rvert$, $(\theta_{min} - \theta)/\lvert\theta_{min}\rvert$ | Pitch limits |
| `rate_p`, `rate_q`, `rate_r` | $p^2/p_{max}^2 - 1$, and likewise for $q$, $r$ | Body-rate limits |
| `thrust_min` | $-T/T_{ref}$ | Non-negative thrust |
| `nfz:<id>` | see below | One row per zone |

Two-sided symmetric limits (bank, rates, sideslip) are written as one smooth quadratic row instead of two linear ones.

### No-fly zones

For zone $j$ with polygon $\mathcal P_j$ and margin $m_j$, let $d_j(x,y)$ be the signed distance from the point to the polygon: the distance to the nearest edge, positive outside and negative inside (crossing-number test). The row is

$$
h_{nfz,j} = \begin{cases}\dfrac{\tilde m_j - d_j(x_k,y_k)}{\max(\tilde m_j,\,25\ \text{m})} & \text{if zone } j \text{ is active}\\[8pt] -1 & \text{otherwise}\end{cases}
$$

A zone is active if its altitude band $[h_{min,j},h_{max,j}]$ overlaps the mission's band $[\min(h_0,h_f),\max(h_0,h_f)]$. The decision is made once per mission: a row that switched on and off with the node's altitude would be discontinuous, which a gradient-based solver cannot handle. Inactive zones keep a constant row so that the problem size does not depend on the mission's altitudes.

**Margin between nodes.** The rows hold at the nodes only, and the trajectory between two nodes is (to first order) a straight chord. If both ends of a chord of length $\ell$ keep a distance $\tilde m$ from a polygon corner, every point of the chord keeps at least $\sqrt{\tilde m^2 - \ell^2/4}$. The pipeline therefore enforces at the nodes the inflated margin

$$
\tilde m_j = \sqrt{m_j^2 + \left(\frac{\ell}{2}\right)^2},\qquad \ell = \frac{1.5\,L_{ref}}{N-1},
$$

where $\ell$ is the node spacing of a path 50 % longer than the reference length (to allow for detours). Where the nearest feature is a straight edge rather than a corner, the distance varies linearly along the chord, so its minimum is at an end and the bound holds as well. For mission M-01 ($L_{ref}$ = 3000 m, $N$ = 60, $m$ = 40 m): $\ell$ = 76 m and $\tilde m$ = 55 m. The post-check of section 6.8 confirms the real margin along the whole path.

## 7.6 Boundary conditions, waypoints and bounds

### Boundary conditions

Equality rows, each divided by the scale of its state (section 7.10). Built in the constructors of `UfProblem` ([`ocp_uf.cpp`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/src/formulation/ocp_uf.cpp)) and `FcProblem`.

| Model | Initial node (fixed) | Final node (fixed) | Rows |
|---|---|---|---|
| UF | $x_N, y_E$ = `start`; $z_D=-h_0$; $V$ = `V_mps`; $\gamma=0$; $\chi$ = `start_heading_deg`; $\mu=0$; $m=m_0$ | $x_N, y_E$ = `end`; $z_D=-(h_0+\Delta h)$; $\chi$ = `end_heading_deg` | 8 + 4 |
| FC | Wings-level 6-DoF trim at `V_mps` and $h_0$ (body velocities and pitch from the trim, zero roll and rates), at the start position, with $\psi$ = `start_heading_deg` and $m=m_0$ | $x_N, y_E, z_D$ as UF; $\psi$ = `end_heading_deg` | 13 + 4 |

The final heading is matched modulo $2\pi$: the target is shifted by the multiple of $2\pi$ nearest to the final heading of the initial point, so a mission that ends after a full turn is not forced to unwind it.

### Waypoints

Each waypoint $w$ is assigned to one node $k_w$ (section 6.5) and contributes the row

$$
\frac{(x_{N,k_w} - x_w)^2 + (y_{E,k_w} - y_w)^2}{r_w^2} - 1 \le 0 ,
$$

with $r_w$ = `tolerance_m`. The rows are labelled `wp:<id>` and verified with the environment group.

### Variable bounds

Box bounds keep every function defined and the solver in a sensible region. The operational limits are path constraints, not bounds, except for the FC control surfaces.

| Variable | UF bound | FC bound |
|---|---|---|
| Flight time $t_f$ | $[0.3,\,3]\,t_{ref}$ | $[0.3,\,3]\,t_{ref}$ |
| Airspeed $V$ (UF), forward velocity $u$ (FC) | $[0.3\,V_{cruise},\;1.3\,V_{NE}]$ | $[0.3\,V_{cruise},\;1.3\,V_{NE}]$ |
| Lateral and vertical body velocity $v$, $w$ | – | $\pm0.5\,V_{NE}$ |
| Flight-path angle $\gamma$ | $\pm45°$ | – |
| Bank $\mu$ (UF), roll $\phi$ (FC) | $\pm80°$ | $\pm80°$ |
| Pitch $\theta$ | – | $\pm60°$ (away from the Euler singularity) |
| Body rates $p,q,r$ | – | $\pm2\times$ their limits |
| Mass $m$ | $[m_{empty},\;1.001\,m_0]$ | $[m_{empty},\;1.001\,m_0]$ |
| Thrust $T$ | $[0,\;T_{max}(0,0)]$ | $[0,\;T_{max}(0,0)]$ |
| Lift coefficient $C_L$ | $[-0.5,\;1.5\,C_{L,max}]$ | – |
| Bank rate $p_\mu$ | $\pm2\,p_{max}$ | – |
| Elevator, aileron, rudder | – | $\pm\delta_{e,max}$, $\pm\delta_{a,max}$, $\pm\delta_{r,max}$ |
| Positions, heading, yaw | free | free |

## 7.7 Cost function

Implemented in `TrapezoidalUF::cost` and `FcProblem::cost`, with analytic gradients (`costGradient`) checked against finite differences by the tests.

**Point mass.**

$$
J_{UF} = (1-\alpha)\,\frac{t_f}{t_{ref}} + \alpha\,\frac{E}{E_{ref}}
+ \frac{1}{N-1}\sum_{k=0}^{N-2}\left[\beta_c\sum_{j\in\{T,\,C_L,\,p_\mu\}}\left(\frac{u_{j,k+1}-u_{j,k}}{s_{u_j}}\right)^{2} + \beta_s\sum_{i\in\{\gamma,\,\mu\}}\left(\frac{x_{i,k+1}-x_{i,k}}{s_{x_i}}\right)^{2}\right]
$$

with the propulsive energy (thrust work) integrated by the trapezoidal rule,

$$
E = \sum_{k=0}^{N-2}\bar T_k\,\bar V_k\,\Delta t,\qquad \bar T_k = \tfrac12(T_k+T_{k+1}),\quad \bar V_k = \tfrac12(V_k+V_{k+1}).
$$

Since the fuel burnt is $cE$, minimising $E$ minimises fuel.

**Rigid body.**

$$
J_{FC} = (1-\alpha)\,\frac{t_f}{t_{ref}} + \alpha\,\frac{m_0-m_{N-1}}{m_{ref}}
+ \frac{1}{N-1}\sum_{k=0}^{N-2}\left[\beta_c\sum_{j\in\{T,\,\delta_e,\,\delta_a,\,\delta_r\}}\left(\frac{\Delta u_{j,k}}{s_{u_j}}\right)^{2} + \beta_s\sum_{i\in\{\phi,\,\theta\}}\left(\frac{\Delta x_{i,k}}{s_{x_i}}\right)^{2}\right]
$$

Here the fuel is read directly from the mass state, which integrates $\dot m=-cTV_a$.

**References.** $t_{ref} = \max(L_{ref}/V,\,1\ \text{s})$, $E_{ref} = T_{ref}\,V_{cruise}\,t_{ref}$ and $m_{ref} = c\,E_{ref}$, with $T_{ref} = T_{max}(V_{cruise},0)$. Because $m_{ref}=cE_{ref}$, the fuel terms of $J_{UF}$ and $J_{FC}$ are equal for the same flight, and the two costs are on the same scale. The references depend on the mission only, so they are the same for every seed.

**The weights.**

- $\alpha$ moves the optimum between minimum time ($\alpha=0$) and minimum fuel ($\alpha=1$). Both terms are of order one, so intermediate values give a genuine trade-off (tutorial, section 5.7).
- $\beta_c$ and $\beta_s$ penalise node-to-node changes of the controls and of the attitude-like states. Without them a minimum-fuel optimum is bang-bang: thrust jumps between zero and its maximum, and the aircraft follows a saw-tooth of glides and climbs. With $\beta_c=\beta_s=1$ the penalty is 0–1 % of the final cost in the bundled missions. The division by $N-1$ makes the penalty of node-to-node oscillations independent of the number of nodes, while the penalty of a smooth trajectory vanishes as the mesh is refined. The thesis discusses this regularisation in [10. Cost function](../10-cost-function.md) and [13. Regularisation](../13-results-penalties.md).

## 7.8 Transcription by trapezoidal collocation

Implemented in [`collocation.cpp`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/src/formulation/collocation.cpp) (`trapezoidalDefects`, `trapezoidalJacobian`).

The time interval is divided into $N-1$ equal intervals of length $\Delta t = t_f/(N-1)$. States and controls are decision variables at every node, and the dynamics are imposed on each interval by the trapezoidal rule. The **defect** of interval $k$ is

$$
\boldsymbol\Delta_k = \operatorname{diag}(s_x)^{-1}\left[x_{k+1} - x_k - \frac{\Delta t}{2}\Big(f(x_k,u_k) + f(x_{k+1},u_{k+1})\Big)\right] = 0,\qquad k = 0,\dots,N-2 .
$$

The scaling by the state scales $s_x$ makes every defect row dimensionless and comparable. The local truncation error of the trapezoidal rule is $O(\Delta t^3)$ and the global error $O(\Delta t^2)$; a test checks the $\Delta t^3$ decay on a smooth reference trajectory. Because $t_f$ is a variable, $\Delta t$ is too, and every defect depends on $t_f$.

The Jacobian of interval $k$, with $A_k=\partial f/\partial x$ and $B_k=\partial f/\partial u$ at node $k$, has the blocks (before row scaling):

| With respect to | Block |
|---|---|
| $x_k$ | $-I - \tfrac{\Delta t}{2}A_k$ |
| $x_{k+1}$ | $I - \tfrac{\Delta t}{2}A_{k+1}$ |
| $u_k$ | $-\tfrac{\Delta t}{2}B_k$ |
| $u_{k+1}$ | $-\tfrac{\Delta t}{2}B_{k+1}$ |
| $t_f$ | $-\tfrac{1}{2(N-1)}\big(f_k + f_{k+1}\big)$ |

Each interval couples only two neighbouring nodes, so the defect Jacobian is block-banded and very sparse: at $N=200$, only 0.44 % of the UF defect Jacobian's entries are nonzero ([UC-04](../code/examples.md#uc-04-the-nlp-an-optimiser-sees)).

## 7.9 The resulting NLP

The decision vector stacks the flight time, the states and the controls:

$$
z = \big[\,t_f \;\big|\; x_0,\,x_1,\dots,x_{N-1} \;\big|\; u_0,\,u_1,\dots,u_{N-1}\,\big] .
$$

The constraint vector $g(z)$ has four blocks, in this order:

| Block | Rows | Bounds | Content |
|---|---|---|---|
| Defects | $(N-1)\,n_x$ | $= 0$ | Section 7.8 |
| Boundary | $n_x + 4$ | $= 0$ | Section 7.6 |
| Path | $N\,(n_c + n_z)$ | $\le 0$ | Section 7.5, node by node |
| Waypoints | $n_w$ | $\le 0$ | Section 7.6 |

with $n_x$ states, $n_u$ controls, $n_c$ envelope rows, $n_z$ zones and $n_w$ waypoints.

| | UF | FC |
|---|---|---|
| $n_x$, $n_u$, $n_c$ | 8, 3, 12 | 13, 4, 16 |
| Variables | $1 + 11N$ | $1 + 17N$ |
| Equality constraints | $8(N-1) + 12$ | $13(N-1) + 17$ |
| Inequality constraints | $(12 + n_z)N + n_w$ | $(16 + n_z)N + n_w$ |
| Example: $N = 60$, one zone, no waypoint | 661 variables, 484 equalities, 780 inequalities | 1021 variables, 784 equalities, 1020 inequalities |

The UF numbers are those IPOPT prints for mission M-01 (section 8.4).

**Jacobian.** `CollocationProblem::constraintJacobian` ([`ocp_problem.cpp`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/src/formulation/ocp_problem.cpp)) returns the nonzero entries as (row, column, value) triplets, always in the same order:

- defect rows: the blocks of section 7.8, from the analytic UF Jacobian or the central-difference FC Jacobian, keeping only structurally nonzero entries;
- boundary rows: one constant entry each;
- path rows: the rows of node $k$ depend only on $(x_k,u_k)$, so each node contributes a dense $(n_c+n_z)\times(n_x+n_u)$ block, filled by central differences (for M-01: 60 × 13 × 11 = 8580 entries, the inequality count IPOPT reports);
- waypoint rows: two analytic entries each.

The tests compare the whole Jacobian with a dense finite-difference one.

**Hessian.** Second derivatives are not computed. IPOPT approximates the Hessian of the Lagrangian with a limited-memory BFGS update (section 8.2).

## 7.10 Scaling

IPOPT works best when variables and constraints are of order one. Every variable is divided by a scale, and the solver sees $y = z/s$. The scales come from the aircraft and the mission, never from the initial point:

| UF variable | Scale | FC variable | Scale |
|---|---|---|---|
| $x_N, y_E, z_D$ | $L_{ref}$ | $x_N, y_E, z_D$ | $L_{ref}$ |
| $V$ | $V_{cruise}$ | $u$ | $V_{cruise}$ |
| $\gamma$ | $\gamma_{max}$ | $v$, $w$ | $0.2\,V_{cruise}$ |
| $\chi$ | $2\pi$ | $\phi$, $\theta$, $\psi$ | $\phi_{max}$, $\theta_{max}$, $2\pi$ |
| $\mu$ | $\phi_{max}$ | $p, q, r$ | $p_{max}, q_{max}, r_{max}$ |
| $m$ | $m_0$ | $m$ | $m_0$ |
| $T$ | $T_{ref}$ | $T$ | $T_{ref}$ |
| $C_L$ | $C_{L,max}$ | $\delta_e, \delta_a, \delta_r$ | their limits |
| $p_\mu$ | $p_{max}$ | | |
| $t_f$ | $t_{ref}$ | $t_f$ | $t_{ref}$ |

Constraint rows are already dimensionless: defects and boundary rows are divided by the state scales, path rows by their limits, waypoint rows by the tolerance. The cost terms are divided by $t_{ref}$, $E_{ref}$ or $m_{ref}$.

## 7.11 Differences from the thesis formulation

The public code follows the thesis method with these simplifications, chosen for clarity:

| Topic | Public code | Thesis solver |
|---|---|---|
| Waypoints | Pinned to the node of the initial point nearest to them | Arc-length formulation with slack variables, so the solver chooses when each waypoint is passed |
| Terrain and geofences | Not included | Terrain clearance from a digital elevation model, geofences |
| No-fly zones | Polygonal prisms with a margin; activity decided per mission | Polyhedral zones with strictness levels |
| Warm start | Hybrid A\* path flown by guidance laws | Hybrid A\* path converted by inverse dynamics |
| FC derivatives | Central differences of the model | Finite differences in v1; automatic differentiation in v2 ([Derivatives: v1 and v2](../derivatives.md)) |
| Aircraft | Generic UAV from public data | Flyox I (Singular Aircraft) |

[← 6. The optimisation process](06-optimisation-process.md) · [Next: 8. Using IPOPT →](08-ipopt.md)
