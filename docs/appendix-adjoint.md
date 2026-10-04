# Appendix: indirect (adjoint) method

!!! abstract "TL;DR"
    The indirect route derives optimality conditions analytically (Hamiltonian, costates) before discretising. It was used to understand the problem structure, not to build the solver.

## Outline

For the OCP of [page 5](05-ocp-theory.md), introduce costates $\lambda(t)$ and the Hamiltonian

$$
H(x,u,\lambda)=L(x,u)+\lambda^\top f(x,u).
$$

Necessary conditions: $\dot\lambda=-\partial H/\partial x$, $\ \partial H/\partial u=0$ (or minimisation of $H$ over admissible $u$), plus transversality at $t_f$. Inequality path constraints are handled by multipliers or, as explored in the thesis, by an Uzawa-type iteration.

## Direct vs. indirect

| | Direct (used) | Indirect |
|---|---|---|
| Idea | Discretise, then optimise | Optimise, then discretise |
| Initial guess | Tolerant | Needs costate guess (non-intuitive) |
| Path constraints | Natural | Hard (switching structure) |
| Insight | Numerical | Analytical structure |

For point-mass fixed-fuel dynamics, closed-form regimes (constant-speed cruise, equilibrium turn) served as quasi-stationary references to detect non-physical local minima in early solver runs.

[Back to home](index.md)
