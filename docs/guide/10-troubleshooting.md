# 10. Troubleshooting

Problems are grouped by the moment they appear: while building, while reading the input, during the warm start, during the optimisation, and when looking at the results. Solver failures are covered in more depth in [section 8.7](08-ipopt.md#87-when-a-solve-fails).

## 10.1 Build and installation

| Symptom | Cause | Remedy |
|---|---|---|
| CMake prints `IPOPT not found: building without the optimiser` | Neither `pkg-config` nor `IPOPT_DIR` leads to IPOPT | Linux/macOS: install the development package and `pkg-config` (section 1.3). Custom installation: add its `lib/pkgconfig` folder to `PKG_CONFIG_PATH`, or pass `-DIPOPT_DIR=<folder>`. Then delete `build/` and configure again |
| `OFC_IPOPT=ON but IPOPT was not found` | Same, with IPOPT required | As above |
| `ofc --info` says `not built` after installing IPOPT | `build/` was configured before IPOPT was installed; CMake caches the result | Delete `build/` (or `build/CMakeCache.txt`) and build again |
| Windows: link errors against `ipopt.dll.lib`, or IPOPT ignored with MinGW | The official IPOPT binaries are built with MSVC | Build with Visual Studio: `run.ps1` does it automatically when `IPOPT_DIR` is set, or use `-G "Visual Studio 17 2022" -A x64` |
| Windows: `Error: generator ... does not match the generator used previously` | `build/` was configured with another generator | `run.ps1` resets the cache itself; by hand, delete `build/` |
| Windows: `ipopt-3.dll was not found` when starting `ofc.exe` | The executable was copied elsewhere without the DLLs | Run it from `build\Release\`, where CMake copies the DLLs, or add `%IPOPT_DIR%\bin` to `PATH` |
| Windows: `install_ipopt.ps1` cannot be loaded | Script execution is disabled | `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` |
| Windows: `IPOPT_DIR` not seen by CMake in an old terminal | The variable was set after the terminal opened | Open a new terminal, or use `run.ps1`, which reads the user variable |
| Compiler errors about `<filesystem>` or C++17 | Compiler too old | GCC 9, Clang 9 or MSVC 2019 or newer |
| A test fails | A modified model, data file or derivative | Run `build/ofc_tests` directly to see every failing check |

## 10.2 Input errors

Errors in the input stop the case with `error (<case file>): <message>` and exit code 1. The other case files given on the same command line still run.

| Message | Meaning |
|---|---|
| `cannot open data/cases/x.json` | The case file path is wrong |
| `cannot open aircraft file: data/aircraft/...` | The program was not started from `code/`, or the `aircraft` path is wrong |
| `[json.exception.parse_error.101] parse error at line L, column C` | Invalid JSON: usually a missing or extra comma, or a comment in the aircraft file (comments are accepted in case files only) |
| `[json.exception.out_of_range.403] key 'missions' not found` | A required key is missing (here `missions`; also `aircraft`, `polygon`, `position`) |
| `[json.exception.type_error.302] type must be number, but is string` | A number was written in quotes |
| `unknown use_case 'UC-8' in ... (expected UC-01 ... UC-09)` | Misspelt `use_case` |
| `aircraft: missing number 'envelope.V_ne'` | A required aircraft field is missing |
| `aircraft: m_empty + fuel_max exceeds m0` | Inconsistent mass budget |
| `aircraft: inertia tensor is not positive definite` | $I_{xx}I_{zz} \le I_{xz}^2$ |
| `aircraft: Cma >= 0 (statically unstable in pitch)` | The pitch-stability derivative must be negative |
| `aircraft: stall speed exceeds V_ne` | Wing too small, mass too large or `alpha_max_deg` too small |
| `aircraft: load-factor limits must bracket 1 g` | Need `n_min < 1 < n_max` |
| `mission M-10: 'end' [north, east] is required` | Every mission needs a goal |
| `expected [north, east]` | A position with other than two numbers |
| `NFZ A: polygon needs at least 3 vertices` | Degenerate zone |
| `NFZ A: h_max_m must be > h_min_m` | Empty altitude band |
| `pipeline.seed must be 'warm' or 'cold'` | Invalid seed |
| `pipeline label 'WS>UFC': unknown stage 'UFC'` | Invalid `stages` value or UC-09 label |
| `mission M-10: rigid-body trim did not converge` | The 6-DoF model cannot trim at `V_mps` and `h0_m`: speed too low for the elevator authority, or unrealistic aircraft data. Check with UC-02 and UC-05 |

Remember that unknown keys are ignored. If a setting seems to have no effect, compare its spelling with chapter 3 and check the header line that UC-08 prints.

## 10.3 Warm start

| Symptom | Cause | Remedy |
|---|---|---|
| `warm start: planner found no path after N expansions` | The zones (plus margin and `safety_m`) block every route at the planning turn radius, the start or goal is inside a zone, or a waypoint lies inside one | Check the geometry with a plot; reduce `safety_m`; refine the grid (`cell_m` 10, `heading_bins` 144); increase `bank_fraction` for tighter planned turns; reduce `V_mps` or increase `bank_deg` |
| `warm start: guidance did not reach the end of the path` | The path has turns too tight for the guidance, or the time budget ran out | Reduce `bank_fraction` so the guidance has bank in reserve; check that `V_mps` is a comfortable speed for the aircraft |
| The warm start is slow (seconds) | Large search space: long missions, fine grid, many zones | Coarser grid (`cell_m` 30–40); it only needs to be good enough to seed the optimiser |

When the warm start fails, the UF stage starts from a straight line and the row is marked `(CHECK)` even if the optimisation succeeds. The optimum may still be valid: check its status and clearances.

## 10.4 Optimisation

| Symptom | Remedy |
|---|---|
| UC-08 prints `skipped: this build has no IPOPT` | Build with IPOPT (chapter 1) |
| A stage ends `infeasible`, `max_iter`, `max_time` or `failed` | See [section 8.7](08-ipopt.md#87-when-a-solve-fails) |
| A stage converges but is marked `!` | The verification found a row or bound violated by more than `verify_tol`; its label is printed (section 4.3). Tighten `constr_viol_tol`, or look at the named row: it identifies the constraint that IPOPT's scaled convergence test did not enforce tightly enough |
| The run is very slow | Reduce the nodes, run `"stages": "UF"` only, or lower `max_cpu_time_s` to fail fast. With many missions, run several case files in parallel processes |
| Different iteration counts from those in this documentation | Expected: they depend on the IPOPT version and its linear solver. Optima agree to the printed digits |

## 10.5 Results that look wrong

| Observation | Explanation or remedy |
|---|---|
| The optimum passes close to the zone, not in the middle of the gap | Expected: the cost decides the path, and the margin is the only distance required. Increase `margin_m` for more distance |
| The waypoint is passed exactly on its tolerance circle | Expected for minimum fuel: going closer costs fuel. Reduce `tolerance_m` if you need to pass closer |
| The airspeed falls sharply at the end | The final speed is free; a minimum-fuel optimum keeps no kinetic energy it does not need. Add a final-speed condition in code if required (section 11.6) |
| The optimum flies over a zone | The zone's altitude band does not overlap the mission's altitudes, so it is inactive (drawn hatched in grey). Raise `h_max_m` if it should be avoided |
| Thrust or bank oscillate from node to node | The smoothness weights are too small. Use `beta_control` and `beta_state` of about 1 |
| The 6-DoF optimum burns noticeably less fuel than the point mass | The sideslip limit was relaxed; the 6-DoF model is using side force as free lift. Keep `sideslip_max_deg` at 1 (section 7.4) |
| `clr` is much larger than expected | Few nodes: the zone margin is inflated by half the node spacing (section 7.5). More nodes reduce the inflation |
| `simUF` of several metres | Too few nodes for the manoeuvres; increase `nodes_uf` |
| A cold start gives a different optimum from a warm start | Both are local optima of the same problem; the lower cost is the better one. The warm start usually finds it |

## 10.6 Figures and documentation

| Symptom | Remedy |
|---|---|
| `plots skipped: needs numpy and matplotlib` | `pip install numpy matplotlib` |
| `skip UC-08 (output/uc08): run a UC-08 case with an IPOPT build first` | The default plot looks for `output/uc08/M-*`. For your own case use `--optimal <output folder>` |
| No figure for a mission of your own case | Plot that folder with `--optimal`; without it, only missions whose identifier starts with `M-` in `output/uc08` are drawn |
| Equations shown as raw `$...$` on a local site | Serve the site with `mkdocs serve` from the repository root; the MathJax configuration is in `mkdocs.yml` |

[← 9. Example catalogue](09-examples.md) · [Next: 11. Using and extending the library →](11-extending.md)
