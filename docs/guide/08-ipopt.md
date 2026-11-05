# 8. Using IPOPT

[IPOPT](https://github.com/coin-or/Ipopt) (Interior Point OPTimizer) is the open-source solver that computes every optimum in this repository. This chapter explains what it does, how the code hands it the problem, what every option does, how to read its log, and what to do when a solve fails. Installation is covered in chapter 1.

## 8.1 The problem IPOPT solves

IPOPT solves smooth nonlinear programs of the form

$$
\min_{y\in\mathbb R^n} f(y)\quad\text{subject to}\quad g_L \le g(y) \le g_U,\qquad y_L \le y \le y_U ,
$$

where $f$ and $g$ are twice continuously differentiable. Equality constraints have $g_L = g_U$; one-sided inequalities have an infinite bound. The collocated trajectory problem of chapter 7 has exactly this form: $y$ is the scaled decision vector, $f$ the cost, and $g$ stacks the defects and boundary rows ($g_L = g_U = 0$) and the path and waypoint rows ($g_L = -\infty$, $g_U = 0$).

IPOPT finds a **local** solution: a point that satisfies the first-order optimality (KKT) conditions. It does not guarantee the global optimum, which is why the initial point matters (chapter 6).

## 8.2 How IPOPT works

The method is described in Wächter and Biegler, *On the implementation of an interior-point filter line-search algorithm for large-scale nonlinear programming*, Mathematical Programming 106 (2006). The main ideas, in the order they act in each iteration:

**Slacks and barrier.** Inequalities are turned into equalities with slack variables, $g(y) - s = 0$ with $g_L\le s\le g_U$. The bounds on $y$ and $s$ are then replaced by logarithmic barrier terms, which tend to infinity at the bounds:

$$
\min_{y,s}\; \varphi_\mu(y,s) = f(y) - \mu\sum_i \ln(y_i - y_{L,i}) - \mu\sum_i \ln(y_{U,i} - y_i) - \mu\sum_j \ln(s_j - g_{L,j}) - \mu\sum_j\ln(g_{U,j} - s_j)
$$

subject to $g(y) - s = 0$ (terms with infinite bounds are omitted). For a fixed barrier parameter $\mu>0$ the iterates stay strictly inside the bounds; as $\mu\to0$, the solutions of the barrier problems approach the solution of the original problem.

**Newton step.** For the current $\mu$, IPOPT applies Newton's method to the optimality conditions of the barrier problem. Each iteration solves one sparse, symmetric, indefinite linear system:

$$
\begin{bmatrix} W_k + \Sigma_k + \delta_w I & J_k^\top \\ J_k & -\delta_c I \end{bmatrix}
\begin{bmatrix} \Delta y \\ \Delta\lambda \end{bmatrix}
= -\begin{bmatrix} \nabla\varphi_\mu + J_k^\top\lambda_k \\ c(y_k) \end{bmatrix},
$$

where $J_k$ is the constraint Jacobian, $W_k$ the Hessian of the Lagrangian, $\Sigma_k$ a diagonal matrix from the barrier terms and $\lambda$ the constraint multipliers. The regularisations $\delta_w$ and $\delta_c$ are added only when the matrix has the wrong inertia (column `lg(rg)` of the log). Because each collocation interval couples only neighbouring nodes, this matrix is very sparse, and a sparse direct solver (MUMPS by default) factorises it at a cost that grows roughly linearly with the number of nodes.

**Hessian approximation.** The code does not provide second derivatives. IPOPT builds a limited-memory BFGS (L-BFGS) approximation of $W_k$ from the last few changes of the gradient of the Lagrangian (`lbfgs_history` pairs, default 6). This needs more iterations than an exact Hessian, but each iteration is cheap and nothing beyond first derivatives has to be coded.

**Filter line search.** The step is accepted if it sufficiently reduces either the barrier objective or the constraint violation, compared with a "filter" of previously seen pairs. This avoids a penalty parameter to balance the two. If the step is rejected, the step length $\alpha$ is halved (column `ls` counts the trials), possibly after a second-order correction that reduces the constraint violation.

**Restoration phase.** If no acceptable step can be found, IPOPT temporarily ignores the objective and minimises the constraint violation, to return to a region where the filter accepts steps. Iterations in this phase are marked with `r` in the log. A solve that keeps entering restoration usually has a poor initial point or locally inconsistent constraints.

**Barrier update.** When the barrier problem is solved accurately enough, $\mu$ is reduced. With `mu_strategy` = `monotone`, $\mu$ only decreases, by a fixed rule. With `adaptive` (the default in this code), $\mu$ is recomputed every iteration from the current complementarity, and may also increase temporarily.

**Termination.** IPOPT stops with `Optimal Solution Found` when the scaled optimality error (the largest of dual infeasibility, constraint violation and complementarity) is below `tol`, and the unscaled constraint violation is below `constr_viol_tol` (plus absolute tests on dual infeasibility and complementarity, left at IPOPT's defaults). It stops with `Solved To Acceptable Level` if the looser `acceptable_tol` is met for 15 consecutive iterations without reaching `tol`. Otherwise it stops at `max_iter`, `max_cpu_time_s`, or on an error.

## 8.3 How the code calls IPOPT

The solver interface is [`src/solver/ipopt_solver.cpp`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/src/solver/ipopt_solver.cpp). It is the only file that includes IPOPT headers. Its `Adapter` class implements IPOPT's C++ interface `Ipopt::TNLP` for any `nlp::Problem` ([`nlp.hpp`](https://github.com/pablourioste/uavtraj-opt/blob/main/code/include/ofc/nlp.hpp)):

| IPOPT callback | What the adapter returns | From `nlp::Problem` |
|---|---|---|
| `get_nlp_info` | Number of variables and constraints, number of Jacobian nonzeros, no Hessian | `numVariables`, `numConstraints`, structure of `constraintJacobian` |
| `get_bounds_info` | Scaled variable bounds $y_{L,U} = z_{L,U}/s$; constraint bounds | `variableBounds`, `constraintBounds` |
| `get_starting_point` | Scaled initial point $y_0 = z_0/s$ | `initialPoint` |
| `eval_f` | Cost at $z = s\,y$ | `cost` |
| `eval_grad_f` | Gradient with respect to $y$: $\partial f/\partial y_j = s_j\,\partial f/\partial z_j$ | `costGradient` |
| `eval_g` | Constraint values | `constraints` |
| `eval_jac_g` | First call: row and column of every nonzero. Later calls: values, scaled by the column's variable scale | `constraintJacobian` |
| `eval_h` | Not implemented: limited-memory Hessian | |
| `intermediate_callback` | Records the iteration count | |
| `finalize_solution` | Stores the final point (unscaled), the objective and the status | |

Four details matter in practice:

1. **Scaling.** IPOPT sees $y = z/s$, with the scales of section 7.10, so every variable it handles is of order one. IPOPT's own gradient-based scaling (`nlp_scaling_method`, on by default) still applies on top.
2. **Fixed sparsity.** The Jacobian structure is recorded once, from the initial point, and every later evaluation must return the same entries in the same order. `CollocationProblem` guarantees this; a custom problem must too.
3. **Robustness.** If an evaluation produces a non-finite number (for example a square root of a negative number far from the solution), the callback returns `false`. IPOPT then shortens the step instead of failing.
4. **Options.** The adapter always sets `hessian_approximation = limited-memory` and suppresses IPOPT's banner (`sb = yes`). The other options come from the `solver` block of the case file (section 8.5).

Without IPOPT, the same file compiles a stub whose `nlp::solve` returns the status `no_solver` immediately, so the rest of the library builds and runs.

### Status mapping

IPOPT's outcome is reduced to seven statuses (section 4.2); its own return code is kept as the stage `message`:

| IPOPT return code (`message`) | Status | Usual cause |
|---|---|---|
| `Solve_Succeeded` | `solved` | |
| `Solved_To_Acceptable_Level` | `acceptable` | Progress stalled between `acceptable_tol` and `tol`, often because of finite-difference noise in the derivatives |
| `Infeasible_Problem_Detected` | `infeasible` | Contradictory constraints (start or goal inside a zone, unreachable waypoint, too short a time bound), or a poor initial point |
| `Maximum_Iterations_Exceeded` | `max_iter` | Hard problem or poor seed; see section 8.7 |
| `Maximum_CpuTime_Exceeded` | `max_time` | As above, or `max_cpu_time_s` too small |
| `Restoration_Failed`, `Search_Direction_Becomes_Too_Small`, `Error_In_Step_Computation`, `Diverging_Iterates`, `Invalid_Number_Detected`, others | `failed` | Badly scaled or inconsistent problem, non-smooth constraints, or evaluation errors |

## 8.4 Reading the log

With `"print_level": 5` in the `solver` block, IPOPT prints one line per iteration. The log below is the UF stage of mission M-01 (60 nodes, one zone, minimum fuel), shortened in the middle. To reproduce it, copy `uc08_optimal_missions.json`, keep mission M-01 only, and set `"stages": "UF"` and `"solver": { "print_level": 5 }`.

```text
This is Ipopt version 3.14.20, running with linear solver MUMPS 5.9.1.

Number of nonzeros in equality constraint Jacobian...:     4614
Number of nonzeros in inequality constraint Jacobian.:     8580
Number of nonzeros in Lagrangian Hessian.............:        0

Total number of variables............................:      661
                     variables with only lower bounds:        0
                variables with lower and upper bounds:      421
                     variables with only upper bounds:        0
Total number of equality constraints.................:      484
Total number of inequality constraints...............:      780
        inequality constraints with only lower bounds:        0
   inequality constraints with lower and upper bounds:        0
        inequality constraints with only upper bounds:      780

iter    objective    inf_pr   inf_du lg(mu)  ||d||  lg(rg) alpha_du alpha_pr  ls
   0  4.5808486e-01 3.00e-01 1.23e+00   0.0 0.00e+00    -  0.00e+00 0.00e+00   0
   1  4.8704629e-01 2.20e-01 2.94e+01  -0.4 6.35e-01    -  8.48e-01 1.00e+00f  1
   2  6.0260821e-01 6.15e-01 2.26e+02  -0.3 3.78e+00    -  2.53e-01 1.00e+00f  1
   ...
   9  1.5615540e+00 4.25e-03 2.76e-02  -1.7 2.73e+00    -  1.00e+00 1.00e+00h  1
  10  1.2051763e+00 2.89e-01 9.79e+00  -7.7 1.03e+01    -  4.76e-01 1.00e+00h  1
   ...
  42  3.8999483e-01 4.78e-05 3.88e-04 -11.0 9.99e-03    -  1.00e+00 1.00e+00h  1
   ...
  59  3.8985347e-01 5.47e-05 4.42e-04  -9.8 5.41e-01    -  1.00e+00 9.10e-02h  3
  60  3.8985098e-01 5.84e-05 5.90e-04  -7.8 5.37e-01    -  1.00e+00 3.17e-02h  4
   ...
  99  3.8982744e-01 6.07e-09 1.26e-06 -11.0 1.10e-04    -  1.00e+00 1.00e+00h  1
 100  3.8982744e-01 4.39e-09 9.35e-07 -11.0 9.50e-05    -  1.00e+00 1.00e+00h  1

Number of Iterations....: 100

                                   (scaled)                 (unscaled)
Objective...............:   3.8982744055891649e-01    3.8982744055891649e-01
Dual infeasibility......:   9.3478619523031041e-07    9.3478619523031041e-07
Constraint violation....:   4.3864530684784514e-09    4.3864530684784514e-09
Variable bound violation:   0.0000000000000000e+00    0.0000000000000000e+00
Complementarity.........:   1.0000000000000274e-11    1.0000000000000274e-11
Overall NLP error.......:   9.3478619523031041e-07    9.3478619523031041e-07

Number of objective function evaluations             = 129
Number of objective gradient evaluations             = 101
Number of equality constraint evaluations            = 129
Number of inequality constraint evaluations          = 129
Number of equality constraint Jacobian evaluations   = 101
Number of inequality constraint Jacobian evaluations = 101
Number of Lagrangian Hessian evaluations             = 0
Total seconds in IPOPT                               = 0.511

EXIT: Optimal Solution Found.
```

### The header

The problem sizes match section 7.9: 661 variables ($1 + 11\times60$), 484 equalities (8 × 59 defects + 12 boundary rows) and 780 inequalities (13 rows × 60 nodes). The 421 bounded variables are $t_f$ and the seven bounded states and controls at each node ($1 + 7\times60$). The 8580 inequality nonzeros are the dense 13 × 11 path blocks of the 60 nodes. "Number of nonzeros in Lagrangian Hessian: 0" confirms the limited-memory approximation.

### The iteration columns

| Column | Meaning |
|---|---|
| `iter` | Iteration number. A trailing `r` marks the restoration phase |
| `objective` | Cost $f$ at the current point (here: the normalised fuel plus the smoothness penalty) |
| `inf_pr` | Primal infeasibility: largest constraint violation of the internal (scaled, slack) problem |
| `inf_du` | Dual infeasibility: size of the gradient of the Lagrangian, which is zero at an optimum |
| `lg(mu)` | $\log_{10}$ of the barrier parameter |
| ‖d‖ | Size of the step (largest component) |
| `lg(rg)` | $\log_{10}$ of the Hessian regularisation $\delta_w$; `-` when none was needed |
| `alpha_du`, `alpha_pr` | Step lengths of the multipliers and of the variables; 1 is a full Newton step |
| letter after `alpha_pr` | Type of step: `f` reduces the objective, `h` reduces the infeasibility, `F`/`H` the same with a second-order correction, `R` start of restoration, `w` watchdog, `s` soft restoration, `t`/`T` tiny step |
| `ls` | Number of line-search trials |

### What this log shows

- **Iterations 0–9.** The warm start violates the collocation equations by 0.3 (largest scaled defect). With $\mu$ still large, IPOPT reduces the violation to 0.004 and lets the cost rise from 0.46 to 1.56 on the way.
- **Iterations 10–42.** The adaptive barrier update drops $\mu$, not monotonically, to its floor of $10^{-11}$. The cost falls to 0.390, its final value to three digits.
- **Iterations 42–100.** IPOPT refines the solution at the smallest $\mu$. Several `H` steps (second-order corrections) and two short steps at iterations 59–60 (`alpha_pr` 0.09 and 0.03, 3 and 4 line-search trials) show the line search working around curvature that the L-BFGS approximation captures imperfectly.
- **End.** Dual infeasibility $9.3\cdot10^{-7}$ and constraint violation $4.4\cdot10^{-9}$ are below `tol` = $10^{-6}$ and `constr_viol_tol` = $10^{-6}$. The scaled and unscaled values coincide because IPOPT's own gradient scaling did not need to act on this already scaled problem.
- **Cost of evaluations.** 129 cost and constraint evaluations for 101 Jacobians: the line search rarely needed extra trials.

`EXIT: Optimal Solution Found.` corresponds to `solved`; `EXIT: Solved To Acceptable Level.` to `acceptable`.

## 8.5 Options reference

All options go in the `solver` block of the `pipeline` object (section 3.4). They are passed to IPOPT under the names in the third column; the IPOPT documentation describes them in full.

| Case-file key | Default here | IPOPT option (IPOPT's own default) | When to change it |
|---|---|---|---|
| `tol` | $10^{-6}$ | `tol` ($10^{-8}$) | Smaller for very accurate optima; larger (10⁻⁵) for faster, rougher studies. Very small values may be hard to reach, because the path Jacobian is computed by finite differences |
| `acceptable_tol` | $10^{-4}$ | `acceptable_tol` ($10^{-6}$) | Smaller to forbid early `acceptable` stops; the verification still checks the result with `verify_tol` |
| `constr_viol_tol` | $10^{-6}$ | `constr_viol_tol` ($10^{-4}$) | Rows are normalised, so $10^{-6}$ means one part per million of each limit |
| `max_iter` | 1500 | `max_iter` (3000) | Raise if a slow but steady solve hits the limit; lower to fail fast in batch studies |
| `max_cpu_time_s` | 120 | `max_cpu_time` ($10^{20}$) | Per stage, in seconds. Raise for long missions with many nodes |
| `print_level` | 0 | `print_level` (5) | 5 for the iteration log; up to 12 for internal detail (very verbose) |
| `mu_strategy` | `adaptive` | `mu_strategy` (`monotone`) | Try `monotone` if the adaptive update oscillates or a solve stalls with $\mu$ jumping up and down |
| `lbfgs_history` | 6 | `limited_memory_max_history` (6) | Larger values (10–20) can reduce the iteration count at a higher cost per iteration; worth testing on slow FC stages |
| `linear_solver` | IPOPT's default | `linear_solver` (`mumps` in most builds) | `ma27`, `ma57`, `ma86` or `ma97` if your IPOPT is linked with the HSL library: often faster on large problems |
| `derivative_test` | `false` | `derivative_test` = `first-order` (`none`) | When you change a model or add a constraint (section 8.6) |

Options not in this table are not exposed by the case file. To set others (for example `bound_push`, `nlp_scaling_method` or `max_wall_time`), add one line to `nlp::solve` in `ipopt_solver.cpp`, next to the existing `SetNumericValue` and `SetStringValue` calls.

## 8.6 Checking derivatives

`"derivative_test": true` makes IPOPT compare the gradient and the Jacobian provided by the code with its own forward finite differences at the initial point, before the first iteration. Entries whose relative difference exceeds $10^{-4}$ are printed:

```text
Starting derivative checker for first derivatives.

* jac_g [    0,    5] = -1.5296645151557546e-04 v  ~ -1.5298873279334657e-04  [ 1.456e-04]
* jac_g [    8,   13] =  7.2836162465619590e-05 v  ~  7.2741812573440257e-05  [ 9.435e-04]
...
Derivative checker detected 14 error(s).
```

Each line gives the constraint row and the variable column, the value from the code, IPOPT's finite-difference estimate, and the relative difference. The run above (UF stage of M-01 with 12 nodes) flags 14 entries, all of order $10^{-4}$ with relative differences between $10^{-4}$ and $10^{-3}$. That is the noise of a one-sided finite difference, not an error: the unit tests confirm the same Jacobian against central differences to about $10^{-10}$. A real derivative error shows up differently, as relative differences of order one, often along a whole row or column. Use the checker on a small problem (few nodes), because it costs one constraint evaluation per variable.

## 8.7 When a solve fails

| Symptom | Likely cause | What to try |
|---|---|---|
| `infeasible` from the first stage | The mission is contradictory: start or goal within a zone's margin, a waypoint that cannot be reached with the turn radius, a climb steeper than the envelope allows over the distance | Check the geometry (section 3.7); plot the warm start with `--optimal` |
| `infeasible` or many `r` iterations with a cold start | The straight-line seed crosses the zones | Use `"seed": "warm"` |
| `max_iter` with slowly decreasing `inf_du` | Badly resolved trajectory or a flat optimum | More nodes; or accept a looser `tol`; or raise `max_iter` |
| `max_iter` with `lg(mu)` jumping up and down | Adaptive barrier update oscillating | `"mu_strategy": "monotone"` |
| `acceptable` instead of `solved` | Progress stalls near the tolerance, usually from finite-difference noise in the path Jacobian | Normally harmless: the verification checks the result. Raise `tol` to $10^{-5}$ if you prefer `solved` |
| `failed` with `Restoration_Failed` | Locally inconsistent constraints, often at the very start | Check the boundary conditions: `V_mps` within the envelope, `h0_m` above `h_min_m` |
| `failed` with `Invalid_Number_Detected` | An evaluation returned NaN or infinity at the initial point | Check the aircraft data; the initial point is evaluated before any step can be shortened |
| Stage `solved!` (verification failed) | IPOPT's scaled convergence test passed, but a row exceeds `verify_tol` in the original scaling | Tighten `constr_viol_tol`; read the worst row in the summary line |
| FC fails while UF succeeds | Seed far from the 6-DoF optimum, or a 6-DoF limit (incidence, sideslip, rates) that the point mass ignores | Check the worst row; increase `nodes_fc`; try `"fc_from_uf": false` to seed with the FC autopilot run |

Chapter 10 lists failures outside the solver (build, input, planner).

## 8.8 Using IPOPT for your own problem

Any problem that implements the `nlp::Problem` interface can be solved with `nlp::solve` and checked with `nlp::verify`, with the same options and status reporting. Chapter 11 shows the interface and an example.

[← 7. Mathematical formulation](07-formulation.md) · [Next: 9. Example catalogue →](09-examples.md)
