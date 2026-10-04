# 6. Uncoupled formulation (UF): point-mass dynamics

!!! abstract "TL;DR"
    Treat the aircraft as a point mass in wind axes. The lift coefficient $C_L$ is a **direct control**, so no attitude dynamics are integrated. 9 states, 3 controls, $12N+1$ variables.

## Intuition

The short-period and roll modes of a fixed-wing aircraft settle in a few tenths of a second, whereas a trajectory evolves over tens of seconds. The optimiser can therefore request a lift level and bank angle and assume the inner attitude loop tracks them instantly (*time-scale separation*). The price is that the result is not directly a set of surface commands.

## Variables

$$
z=[x,y,z,V,\mu,\gamma,\xi,m,C]^\top,\qquad u=[T,\,C_L,\,p]^\top,\qquad t_f\ \text{free}.
$$

Position is in NED, $V$ is airspeed, $\mu$ the bank angle, $\gamma$ the flight-path angle, $\xi$ the heading, $m$ the mass and $C$ the battery charge (electric aircraft only).

## Equations of motion

$$
\begin{aligned}
\dot x&=V\cos\gamma\cos\xi, &\dot y&=V\cos\gamma\sin\xi, &\dot z&=-V\sin\gamma,\\
\dot V&=\frac{T-D-mg\sin\gamma}{m}, &\dot\gamma&=\frac{L\cos\mu-mg\cos\gamma}{mV}, &\dot\xi&=\frac{L\sin\mu}{mV\cos\gamma},\\
\dot\mu&=p, &\dot m&=-\frac{c_{sfc}\,P}{3.6\times10^{6}}, &P&=\frac{TV}{\eta_p}.
\end{aligned}
$$

Aerodynamics: $L=\tfrac12\rho V^2 S\,C_L$, $D=\tfrac12\rho V^2 S\,C_D$ with a parabolic polar $C_D=C_{D_0}+kC_L^2$ and an ISA atmosphere.

## What each equation means

- $\dot V$: thrust accelerates, drag and the weight component along the path oppose.
- $\dot\gamma$: the vertical component of lift bends the path against gravity.
- $\dot\xi$: the horizontal component of lift turns the aircraft; without sideforce, a bank is the only way to turn.

## How it is implemented

```text
function f_UF(state, control):          # evaluated at every collocation node
    rho  <- ISA(altitude = -z)
    L, D <- dynamic_pressure * S * (C_L, C_D0 + k*C_L^2)
    return [kinematics(V, gamma, xi),
            (T - D - m*g*sin(gamma))/m,
            (L*cos(mu) - m*g*cos(gamma))/(m*V),
            L*sin(mu)/(m*V*cos(gamma)),
            p,
            -c_sfc * (T*V/eta_p) / 3.6e6]
```

[← Previous](05-ocp-theory.md) · [Next: Rigid-body dynamics →](07-dynamics-fc.md)
