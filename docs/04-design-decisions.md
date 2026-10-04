# 4. Design decisions

!!! abstract "TL;DR"
    Hybrid A\* + inverse dynamics → UF collocation NLP → FC collocation NLP, solved with IPOPT. Each choice favours **robustness and sparsity** over peak speed.

| Block | Chosen option | Main driver |
|---|---|---|
| Path planner / warm start | Hybrid A\* + inverse dynamics | Deterministic, obstacle-aware, kinematically feasible seed; robust over the whole mission set |
| Optimisation method | Direct trapezoidal collocation + IPOPT | Sparse NLP, friendly to warm starts and to imperfect guesses; standard for 6-DoF aerospace problems |
| Dynamic model | UF → FC hierarchy | Fast, well-conditioned UF; FC adds fidelity and inherits a near-feasible iterate |
| Cost function | Bolza form (time + fuel + regularisation) | Smooth Pareto sweep; regularisation absorbs numerical noise without biasing the optimum |

## Alternatives considered and left out

- **Direct single shooting.** Small NLP, but sensitive to initial guesses and unstable dynamics; path constraints are awkward.
- **Indirect (adjoint) methods.** Elegant optimality structure (see [Appendix](appendix-adjoint.md)) but fragile for constrained, high-dimensional problems. Used as a scaffold for understanding, not as the solver.
- **Reeds–Shepp paths.** Not applicable: the aircraft cannot fly backwards.

## Why the hierarchy

The point-mass model solves quickly and robustly, even with NFZs. Its solution is then lifted into the 14-state rigid-body space analytically, so the expensive FC stage starts close to feasibility.

[← Previous](03-methodology.md) · [Next: OCP and NLP →](05-ocp-theory.md)
