# Aircraft file format

An aircraft is a JSON file with the groups below. Fields marked *opt.* may be omitted and default to 0. Angles are in **degrees**; the code converts them to radians. Missing or inconsistent data stop the program with a message that names the field.

| Group | Field | Unit | Meaning |
|---|---|---|---|
| — | `name`, `source` | text | identifier and data provenance |
| `mass` | `m0_kg`, `m_empty_kg`, `fuel_max_kg` | kg | take-off, empty and usable fuel mass (`m_empty + fuel_max ≤ m0`) |
| `geometry` | `S_m2`, `b_m`, `c_mac_m` | m², m | wing area, span, mean aerodynamic chord |
| `inertia` | `Ixx`, `Iyy`, `Izz`, `Ixz` *(opt.)* | kg m² | body-axis inertia (must be positive definite) |
| `propulsion` | `P_max_W` | W | sea-level shaft power; lapses with altitude as 1.132 σ − 0.132 (Gagg–Ferrar) |
| | `eta_p` | – | propeller efficiency, in (0, 1] |
| | `sfc_kg_per_kWh` | kg/kWh | specific fuel consumption |
| | `thrust_z_offset_m` *(opt.)* | m | thrust line below (+) or above (−) the CG |
| `envelope` | `V_ne`, `V_cruise` | m/s | never-exceed and cruise speed |
| | `h_min_m` | m | minimum altitude |
| | `n_min`, `n_max` | g | load-factor limits (must bracket 1 g) |
| | `phi_max_deg`, `theta_*_deg`, `gamma_*_deg`, `alpha_*_deg` | deg | attitude, flight-path and incidence limits |
| `actuators` | `de_max_deg`, `da_max_deg`, `dr_max_deg` | deg | surface deflection limits |
| | `p_max_dps`, `q_max_dps`, `r_max_dps` | deg/s | body-rate limits |
| `aero.lift` | `CL0`, `CLa`, `CLq` *(opt.)*, `CLde` | –, 1/rad | lift build-up |
| `aero.drag` | `CD0` and either `k` or `oswald_e` | – | polar CD = CD0 + k CL², with k = 1/(π e AR) if `oswald_e` is given |
| `aero.side` | `CY0`, `CYb`, `CYp`, `CYr`, `CYda` *(opt.)*, `CYdr` | – | side force |
| `aero.roll` | `Cl0`, `Clb`, `Clp`, `Clr`, `Clda`, `Cldr` | – | rolling moment |
| `aero.pitch` | `Cm0`, `Cma`, `Cmq`, `Cmde` | – | pitching moment (`Cma < 0` required) |
| `aero.yaw` | `Cn0`, `Cnb`, `Cnp`, `Cnr`, `Cnda`, `Cndr` | – | yawing moment |

Rate derivatives use the non-dimensional rates p b/(2V), q c/(2V) and r b/(2V). Derivatives with respect to angles are per radian.

**Never stored, always derived** (see `Aircraft` in `code/include/ofc/aircraft.hpp`):

- CL_max = CL0 + CLa · α_max
- V_stall(h, m) = √(2mg / (ρ S CL_max))
- P_max(h) = P_max (1.132 σ − 0.132) and T_max(V, h) = η_p P_max(h) / max(V, V_stall)
- (L/D)max and its speed
- R_min = V² / (g tan φ)

To use your own aircraft, copy `code/data/aircraft/generic_uav.json`, change the values, check them with UC-01 and UC-05, and point the case files at the new file.

[← Worked examples](examples.md) · [Code overview](index.md)
