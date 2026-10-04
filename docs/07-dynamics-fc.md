# 7. Full-control formulation (FC): 6-DoF rigid body

!!! abstract "TL;DR"
    Newton–Euler equations in body axes with wind, aerodynamic angles and stability derivatives. 14 states, 4 physical controls $[T,\delta_e,\delta_a,\delta_r]$, $18N+1$ variables. Every control is an actuator command an autopilot can execute.

## The coupling dilemma

Aerodynamic forces depend on the incidence angles $\alpha,\beta$, which are functions of the body velocities $(u,v,w)$, which are in turn driven by those same forces. UF breaks the loop by making $C_L$ a free control; FC resolves it **node by node** in a fixed evaluation order.

## Variables

$$
z=[x_e,y_e,z_e,m,u,v,w,\phi,\theta,\psi,p,q,r,C]^\top,\qquad u=[T,\delta_e,\delta_a,\delta_r]^\top.
$$

## Evaluation order at each node

1. Build the NED→body rotation from the Euler angles.
2. Rotate the wind into body axes; wind-relative velocity $(u_r,v_r,w_r)$ and airspeed $V_a$.
3. Aerodynamic angles: $\alpha=\operatorname{atan2}(w_r,u_r)$, $\beta=\arcsin(v_r/V_a)$.
4. Coefficients $[C_L,C_D,C_Y,C_l,C_m,C_n]$ from linear stability derivatives.
5. Forces and moments in wind axes from dynamic pressure, span and chord.
6. Rotate to body axes; add the thrust pitching moment $T\,d_{Tz}$.
7. Translational dynamics: $\dot V_b=\tfrac1m F-\omega\times V_b$.
8. Rotational dynamics: $I\dot\omega=M-\omega\times(I\omega)$, including the cross-inertia $I_{xz}$.
9. Euler-angle kinematics (singular at $\theta\to\pm90^\circ$; not reached in normal flight).
10. Position kinematics and mass/energy evolution.

```text
function f_FC(state, control, wind):
    R            <- rotation_NED_to_body(phi, theta, psi)
    Vrel         <- (u, v, w) - R * wind
    alpha, beta  <- incidence_angles(Vrel)
    coeffs       <- stability_derivatives(alpha, beta, rates, control)
    F_body, M_body <- rotate_wind_to_body(coeffs, alpha, beta) + thrust_terms
    return [position_rates, mass_rate,
            F_body/m - cross(omega, V_body),
            solve(I, M_body - cross(omega, I*omega)),
            euler_rates(phi, theta, p, q, r)]
```

## UF vs FC at a glance

| | UF | FC |
|---|---|---|
| States / controls | 9 / 3 | 14 / 4 |
| Decision variables | $12N+1$ | $18N+1$ |
| Lift | $C_L$ is a control | From $\alpha$, $\delta_e$ and derivatives |
| Output | Path + quasi-steady lift | Path + actuator commands |
| Typical use | Fast planning, warm start | Fidelity, autopilot-ready output |

The aerodynamic coefficients and mass data of the reference airframe are not reproduced here.

[← Previous](06-dynamics-uf.md) · [Next: Transcription →](08-transcription.md)
