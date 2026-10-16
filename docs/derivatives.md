# Derivatives: finite differences (v1) and exact derivatives (v2)

!!! abstract "TL;DR"
    Every number in the Results chapters comes from **v1** of the solver, which built the constraint Jacobian and the cost gradient by **forward finite differences**. A later version (**v2**) computes them exactly with CasADi's algorithmic differentiation, and is substantially faster. The public code reproduces the mechanism: on the generic aircraft, an analytic sparse Jacobian is 54–529× cheaper to build than a finite-difference one, and the gap grows with the number of nodes.

## Why it matters for an interior-point solver

At every iteration IPOPT needs the gradient of the cost and the Jacobian of all constraints.

| | Forward differences (v1) | Exact derivatives (v2) |
|---|---|---|
| Cost of one Jacobian | one constraint evaluation per decision variable: O(n) evaluations of an O(N) function, so **O(N²)** | one sweep through the expression graph that respects sparsity: **O(N)** |
| Accuracy | truncation and round-off, error floor ≈ 1e-6 | exact to machine precision |
| Effect on convergence | noisy KKT residual near the optimum, so the solver struggles to certify convergence | clean Newton steps |

The noise floor of v1 is one of the two reasons the smoothness regularisation was introduced (see [10. Cost function](10-cost-function.md)). With exact derivatives that reason disappears, and the regularisation is only needed against control chatter.

## What the published results therefore mean

- The **relative** conclusions hold for both versions: the warm start's robustness (8/8 against 5/8 convergence) and the advantage of the intermediate UF stage. They come from the quality of the initial guess, not from how the derivatives are computed.
- The **absolute** times (FC 4.5–12.9 s) are v1 times. They overstate what v2 needs. The end-to-end v2 timings are not part of this benchmark, so the < 4 s real-time target for FC is still unverified.

## The mechanism, reproduced with the public code

Use case UC-04 of the [public code](code/index.md) builds the trapezoidal-collocation NLP of the point-mass model for a generic UAV. It then times the defect Jacobian built both ways:

| N | variables | analytic [ms] | forward differences [ms] | ratio | max relative difference |
|---:|---:|---:|---:|---:|---:|
| 25 | 276 | 0.046 | 2.49 | 54× | 6.5e-11 |
| 50 | 551 | 0.082 | 10.1 | 123× | 8.1e-11 |
| 100 | 1101 | 0.193 | 39.7 | 206× | 8.8e-11 |
| 200 | 2201 | 0.303 | 160 | 529× | 7.7e-11 |

<figure markdown>
  ![Jacobian cost](assets/figures/code/uc04_jacobian_and_seed.png){ width="90%" }
  <figcaption>Left: Jacobian cost against the number of nodes. Finite differences grow quadratically, the sparse analytic Jacobian linearly. Right: seed quality (see Worked examples in the Code section).</figcaption>
</figure>

These are the costs of the Jacobian alone on a small model; the full-solver speed-up of v2 is smaller, because the factorisation of the KKT matrix and the function evaluations stay the same. The direction and the scaling are the point.

[← Warm-start benchmark](14-results-warmstart.md) · [Next: Conclusions →](15-conclusions.md)
