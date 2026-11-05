# User guide

This guide explains how to install, run and understand the public implementation in [`code/`](https://github.com/pablourioste/uavtraj-opt/tree/main/code). It is written to be read in order: each chapter assumes the previous ones and refers back to them where needed. Readers who only want to run an optimisation can stop after chapter 5; chapters 6 to 8 explain what happens inside the solver.

The guide covers the public code only. The thesis chapters (Problem, Theory, Results) describe the original solver and the research behind it, and are referenced where they add depth.

## Reading order

| Chapter | Content | You will be able to |
|---|---|---|
| [1. Installation](01-installation.md) | Compiler, CMake, IPOPT on Linux, macOS and Windows; build options; tests | Build the program with the optimiser and check that IPOPT is linked |
| [2. First run](02-first-run.md) | Command line, bundled case files, scripts, output folders, figures | Run every bundled use case and find its results |
| [3. Input files](03-inputs.md) | Case file, aircraft file, mission, pipeline, planner and solver settings; units and frames | Write a valid case file for your own mission and aircraft |
| [4. Reading the results](04-results.md) | Console table, CSV and JSON files, verification report, figures, exit codes | Decide whether a run succeeded and where the numbers come from |
| [5. Tutorial: your first mission](05-tutorial.md) | Three missions of increasing difficulty, three cost settings, real outputs | Set up, run, plot and interpret an optimisation from scratch |
| [6. The optimisation process](06-optimisation-process.md) | Warm start, point-mass stage, projection, 6-DoF stage, verification; what each stage passes on | Understand the pipeline and choose seeds, stages and node counts |
| [7. Mathematical formulation](07-formulation.md) | Models, constraints, bounds, boundary conditions, cost, collocation, scaling, problem size | Relate every row and variable of the NLP to its equation and source file |
| [8. Using IPOPT](08-ipopt.md) | Interior-point method, the adapter, every solver option, the iteration log, return codes, tuning | Read an IPOPT log, diagnose a failed solve and tune the solver |
| [9. Example catalogue](09-examples.md) | Every bundled case file: purpose, key inputs, what to look at, where the results are discussed | Pick the example closest to your problem and adapt it |
| [10. Troubleshooting](10-troubleshooting.md) | Build, input, planner and solver failures, with their remedies | Recover from the most common errors |
| [11. Using and extending the library](11-extending.md) | C++ API, building a problem by hand, adding new constraints | Call the optimiser from your own code or add a new constraint type |

## Before you start

You need a C++17 compiler, CMake 3.14 or newer and, for the optimisation itself, the IPOPT solver. Python 3 with `numpy` and `matplotlib` is only needed for the figures. Chapter 1 gives the exact commands for each operating system.

No prior knowledge of IPOPT is assumed. Some familiarity with flight mechanics (lift, drag, bank angle, flight-path angle) and with the idea of constrained optimisation helps from chapter 6 onwards.

## Conventions used in the guide

- Commands are run from the `code/` folder unless stated otherwise. Paths inside case files are relative to the folder the program is started from.
- Commands are shown for Linux, macOS and Git Bash. On Windows PowerShell, the executable is `.\build\Release\ofc.exe` instead of `./build/ofc`.
- Numbers quoted from runs (iterations, times, fuel) were produced with the files in the repository. Iteration counts depend on the IPOPT version and its linear solver, and times depend on the machine, so expect small differences on your system.
- Two model names appear throughout: **UF** is the point-mass model (8 states, 3 controls) and **FC** is the rigid-body, six-degree-of-freedom model (13 states, 4 controls).

## Where the rest of the documentation fits

| If you need | Read |
|---|---|
| The background of the problem and the thesis results | [Problem and requirements](../01-problem.md), [Warm-start benchmark](../14-results-warmstart.md), [Conclusions](../15-conclusions.md) |
| The theory behind collocation, constraints and cost | [OCP and NLP](../05-ocp-theory.md) to [Warm start](../11-warm-start.md) |
| The code's module structure | [Architecture](../code/architecture.md) |
| Every field of the aircraft file | [Aircraft file format](../code/aircraft-format.md) |
| The input keys of use cases UC-01 to UC-07 | [Use cases: input reference](../code/use-cases.md) |
| The results of every bundled example, with figures | [Worked examples](../code/examples.md) |

[Next: 1. Installation →](01-installation.md)
