# 5. From optimal control to nonlinear programming

!!! abstract "TL;DR"
    We want the control history that minimises a cost subject to the aircraft's equations of motion and operational limits. Discretising it turns an infinite-dimensional problem into a large but sparse NLP.

## Intuition

A trajectory is a curve in time. An optimiser cannot search over all curves, so we sample it at a finite number of points and ask the solver to find the best values at those points, while forcing the physics to hold between neighbouring points.

## The continuous problem (Bolza form)

With state $x(t)$ (position, velocity, attitude, mass…) and control $u(t)$ (thrust, surface deflections…):

$$
\min_{u(t),\,t_f}\; J = \underbrace{\Phi(x(t_f),t_f)}_{\text{Mayer}} + \int_{t_0}^{t_f} \underbrace{L(x,u,t)}_{\text{Lagrange}}\,dt
$$

subject to

$$
\dot x = f(x,u,t),\qquad x(t_0)=x_0,\qquad \Psi(x(t_f),t_f)=0,\qquad h_{\min}\le h(x,u,t)\le h_{\max}.
$$

- **Mayer term**: cost of the final instant, e.g. total flight time $t_f$.
- **Lagrange term**: running cost, e.g. propulsive power integrated over the flight (fuel).

## Three ways to discretise

| Method | What is discretised | Strength | Weakness |
|---|---|---|---|
| Single shooting | Controls only; states by forward integration | Tiny NLP | Sensitive to guesses; hard path constraints |
| Multiple shooting | Controls and states at interval boundaries | Limits error growth | More machinery |
| **Direct collocation** | Controls **and** states at all nodes; dynamics as algebraic constraints | Robust to poor guesses, sparse, handles path constraints well | Large NLP, needs a sparse solver |

The decision vector then stacks everything: $z=[t_f,\,x_0,\dots,x_{N-1},\,u_0,\dots,u_{N-1}]$.

## What the NLP solver sees

Equality constraints $g(z)=0$ (dynamics defects, boundary conditions) and inequalities $h(z)\le 0$ (terrain, NFZ, envelope). The Lagrangian

$$
\mathcal L(z,\lambda,\nu)=F(z)+\lambda^\top g(z)+\nu^\top h(z)
$$

and the **KKT conditions** (primal feasibility, dual feasibility $\nu\ge0$, complementary slackness $\nu_j h_j=0$, stationarity $\nabla_z\mathcal L=0$) characterise a local optimum. IPOPT replaces the inequalities by a logarithmic barrier and drives the barrier parameter $\mu\to0$:

$$
\mathcal L_\mu = F(z)-\mu\sum_j\ln s_j+\lambda^\top g(z)+\nu^\top\big(h(z)+s\big).
$$

Each iteration solves a Newton system on the KKT matrix

$$
\begin{bmatrix} W_k+\Sigma_k & A_k^\top\\ A_k & 0\end{bmatrix}\begin{bmatrix}\Delta z\\ \Delta\lambda\end{bmatrix}=-\begin{bmatrix}r_{dual}\\ r_{primal}\end{bmatrix}
$$

which is extremely sparse because each node only couples with its neighbour. A sparse direct factorisation (MA57 / MUMPS) therefore costs roughly $O(N)$ per iteration. A filter line-search certifies global progress without hand-tuned penalty parameters.

[← Previous](04-design-decisions.md) · [Next: Point-mass dynamics →](06-dynamics-uf.md)
