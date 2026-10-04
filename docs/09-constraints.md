# 9. Constraint architecture

!!! abstract "TL;DR"
    Four layers: environment, dynamics (defects), boundaries, and performance envelope. All are normalised, and all but the defects act node-wise.

## 1. Domain and environment

- **Terrain (DEM).** Bilinear interpolation of a raster elevation model gives a differentiable surface. $z_i\ge h_{DEM}(x_i,y_i)+\epsilon_{DEM}$, with $\epsilon_{DEM}=5$ m (absorbs the 500 m DEM resolution and between-node excursions).
- **No-fly zones.** Extruded polygon prisms between $z^{min}_j$ and $z^{max}_j$. One signed-distance inequality per polygon face, $d_{ij\ell}\ge\epsilon_{NFZ}=25$ m, active only inside the zone's vertical extent.
- **Geofence.** Same machinery with reversed sign: stay *inside* the authorised polygon (margin 5 m).
- **Waypoints.** To avoid if/else logic that would break a gradient-based solver, each waypoint introduces an arc-length slack $s_{wp,k}\ge0$ and three coupled conditions (position within tolerance, attitude, speed). If a waypoint cannot be met exactly, the solver returns a minimum-violation solution rather than declaring infeasibility.

## 2. Dynamics

The $N-1$ collocation defects ([page 8](08-transcription.md)).

## 3. Boundary conditions

Initial state fixed to the current telemetry; final position and attitude enforced as equalities (speed and mass optionally).

## 4. Performance envelope

| Constraint | UF (point mass) | FC (rigid body) |
|---|---|---|
| Anti-stall | $V/V_{stall}(h,m)\ge1$ | $V_{aer}/V_{stall}\ge1$ (wind-relative) |
| Power available | $TV/(\eta_pP_{max}(h))\le1$ | $T\,U_r/(\eta_pP_{max}(h))\le1$ |
| Structural | $V\le V_{NE}$ | $\lVert V_b\rVert/V_{NE}\le1$ |
| $C_L$ limits | box bound on control | nonlinear in $\alpha,\delta_e$ |
| Minimum altitude | $-z\ge h_{min}$ | $-z\ge h_{min}$ |
| Actuators | n/a | $\lvert\delta_{e,a,r}\rvert\le\delta_{max}$ |
| Rates | $\lvert p\rvert\le p_{max}$ | $\lvert p,q,r\rvert\le(\cdot)_{max}$ |

with $V_{stall}=\sqrt{2mg/(\rho S_wC_{L,max})}$.

## How it is implemented

```text
for each node i:
    h += -(z_i) - dem.interpolate(x_i, y_i) - eps_dem          # <= 0
    for zone in nfz:
        for face in zone.faces:
            h += eps_nfz - signed_distance(x_i, y_i, face)     # <= 0, gated by altitude band
    h += 1 - V_i / stall_speed(h_i, m_i)
```

[← Previous](08-transcription.md) · [Next: Cost function →](10-cost-function.md)
