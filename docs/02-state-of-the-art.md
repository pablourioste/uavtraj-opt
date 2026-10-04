# 2. State of the art

!!! abstract "TL;DR"
    Graph planners are fast but ignore dynamics; numerical optimal control respects dynamics but is slow and fragile without a good initial guess. The gap this work addresses is the **bridge** between the two.

## Three families of methods

1. **Geometric and graph-based planning** (A\*, Hybrid A\*, RRT-like methods). They find collision-free paths quickly. *Hybrid A\** extends the search to continuous headings so that the path respects a minimum turn radius, but it still says nothing about speed, thrust or fuel.
2. **Numerical trajectory optimisation** (direct shooting, multiple shooting, direct collocation, indirect methods). They enforce the equations of motion and produce control histories, at the price of a large nonlinear program that is only locally convergent.
3. **Acceleration and seeding techniques.** Interior-point methods are very sensitive to the initial guess; exploiting sparsity and exact derivatives (algorithmic differentiation) keeps each iteration cheap.

## Research gap

Most published work treats path planning and trajectory optimisation separately, or reports a single speed-up number on easy cases. This project combines them into a **hierarchical pipeline**, evaluates it across a span of scenarios from empty airspace to NFZ-plus-waypoint missions, and reports where it helps, where it does not, and what it costs.

Full bibliography: [References](references.md).

[← Previous](01-problem.md) · [Next: Methodology →](03-methodology.md)
