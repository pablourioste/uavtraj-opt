# Appendix: reference aircraft MSN012 (Flyox I)

!!! abstract "TL;DR"
    MSN012 is the full-scale implementation of Singular Aircraft's **Flyox I** platform and the reference aircraft for every example, test and benchmark in this work. Data below are reproduced from the thesis annex.

!!! note "Mass figures"
    This annex lists the MSN012 configuration used in the simulations (MTOW 4,000 kg). The 1,850 kg MTOW quoted in other summaries of the Flyox I platform refers to a different configuration; the values below are the ones the results were computed with.

## Geometry and mass

| Parameter | Symbol | Value | Unit |
|---|---|---:|---|
| Maximum take-off mass | $m$ | 4000.0 | kg |
| Basic empty weight | $BEW$ | 2450.0 | kg |
| Max payload mass | $PM_{max}$ | 1425.0 | kg |
| Max fuel mass | $FM_{max}$ | 400.0 | kg |
| Wing reference area | $S_w$ | 28.0 | m² |
| Wing span | $b$ | 14.0 | m |
| Mean aerodynamic chord | $c$ | 2.0 | m |
| Aspect ratio | $AR$ | 7.0 | – |
| Oswald efficiency | $e$ | 0.839 | – |
| Inertia $I_{xx}, I_{yy}, I_{zz}$ | | 18340 / 20980 / 29390 | kg·m² |
| Product of inertia | $I_{xz}$ | 0.0 | kg·m² |

## Flight envelope (hard constraints in the NLP)

| Constraint | Symbol | Value | Unit |
|---|---|---:|---|
| Never-exceed speed | $V_{NE}$ | 72.016 | m/s |
| Cruise speed | $V_{cruise}$ | 50.411 | m/s |
| Stall speed | $V_{stall}$ | 38.066 | m/s |
| Minimum altitude | $h_{min}$ | 150 | m AGL |
| Pitch limits | $\theta$ | −10 … +15 | deg |
| Bank limit | $\phi_{max}$ | ±35 | deg |
| Load factor, flaps up | $n$ | [−1.4, 3.5] | g |
| Load factor, flaps down | $n$ | [0.0, 2.5] | g |

## Aerodynamic derivatives

| Coeff. | Value | Coeff. | Value | Coeff. | Value |
|---|---:|---|---:|---|---:|
| $C_{L,0}$ | 0.410 | $C_{l,\beta}$ | −0.112 | $C_{n,\beta}$ | 0.300 |
| $C_{L,\alpha}$ | 3.340 | $C_{l,p}$ | −0.550 | $C_{n,p}$ | −0.072 |
| $C_{L,\delta_e}$ | 0.750 | $C_{l,r}$ | 0.230 | $C_{n,r}$ | −0.600 |
| $C_{D,0}$ | 0.048 | $C_{l,\delta_a}$ | 0.340 | $C_{n,\delta_a}$ | −0.312 |
| $k_2$ | 0.054 | $C_{l,\delta_r}$ | 0.004 | $C_{n,\delta_r}$ | −0.200 |
| $C_{m,0}$ | 0.061 | $C_{Y,\beta}$ | −0.570 | $C_{Y,p}$ | −0.226 |
| $C_{m,\alpha}$ | −1.030 | $C_{m,\delta_e}$ | −2.640 | $C_{Y,\delta_r}$ | 0.260 |

Drag polar: $C_D=C_{D,0}+k_2C_L^2$.

## Propulsion

| Parameter | Symbol | Value | Unit |
|---|---|---:|---|
| Maximum power | $P_{max}$ | 521.99 | kW |
| Propeller efficiency | $\eta_p$ | 0.8 | – |
| Specific fuel consumption | $SFC$ | 0.210 | kg/(kW·h) |
| Thrust line offset | $PT_z$ | −1.1 | m |

[Back to home](index.md)
