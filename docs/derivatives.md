# Derivatives: finite differences (v1) and exact derivatives (v2)

!!! abstract "TL;DR"
    Every number in the Results chapters comes from **v1** of the solver, which built the constraint Jacobian and the cost gradient by **forward finite differences**. A later version (**v2**) computes them exactly by automatic differentiation. Preliminary measurements reported in the thesis conclusions show the point-mass (UF) stage about **3× faster**. The public code reproduces the mechanism: on the generic aircraft, an analytic sparse Jacobian is 57–487× cheaper to build than a finite-difference one, and the gap grows with the number of nodes.

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
- The **absolute** times (FC 4.5–12.9 s) are v1 times. They overstate what v2 needs. The first v2 measurements give about 3× on the UF stage; the full benchmark has not been repeated with v2, so the < 4 s real-time target for FC is still unverified.

## The mechanism, reproduced with the public code

Use case UC-04 of the [public code](code/index.md) builds the trapezoidal-collocation NLP of the point-mass model for a generic UAV. It then times the defect Jacobian built both ways:

| N | variables | analytic [ms] | forward differences [ms] | ratio | max relative difference |
|---:|---:|---:|---:|---:|---:|
| 25 | 276 | 0.098 | 5.5 | 57× | 6.5e-11 |
| 50 | 551 | 0.180 | 20.7 | 115× | 8.1e-11 |
| 100 | 1101 | 0.377 | 113 | 299× | 8.8e-11 |
| 200 | 2201 | 0.654 | 319 | 487× | 7.7e-11 |

<figure markdown>
  ![Jacobian cost](assets/figures/code/uc04_jacobian_and_seed.png){ width="90%" }
  <figcaption>Left: Jacobian cost against the number of nodes. Finite differences grow quadratically, the sparse analytic Jacobian linearly. Right: seed quality (see Worked examples in the Code section).</figcaption>
</figure>

These are the costs of the Jacobian alone on a small model. The full-solver speed-up of v2 is smaller (about 3× on the UF stage in the first measurements), because the factorisation of the KKT matrix and the function evaluations stay the same. The direction and the scaling are the point.

[← Warm-start benchmark](14-results-warmstart.md) · [Next: Conclusions →](15-conclusions.md)
