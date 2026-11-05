# 2. First run

This chapter runs the bundled examples, explains the command line, and shows where the results are written.

## 2.1 Run everything

```bash
bash run.sh           # Linux, macOS, Git Bash
```

```powershell
.\run.ps1             # Windows PowerShell
```

The script builds the program, runs the tests, runs the nine bundled case files `data/cases/uc01_*.json` to `uc09_*.json` in order, and finally calls `utils/plot_results.py` if Python is available. UC-01 to UC-07 finish in a second or two. UC-08 and UC-09 call IPOPT many times and take most of the run: between a few tens of seconds and a couple of minutes, depending on the machine.

At the end, a timing summary lists each case. This one is from a Windows laptop with IPOPT 3.14.20:

```text
== Summary ==
    0.00 s  data/cases/uc01_aircraft_report.json
    0.01 s  data/cases/uc02_trim.json
    0.11 s  data/cases/uc03_simulation.json
    0.34 s  data/cases/uc04_transcription.json
    0.02 s  data/cases/uc05_design_charts.json
    0.31 s  data/cases/uc06_design_scenarios.json
    1.34 s  data/cases/uc07_nfz_missions.json
   23.02 s  data/cases/uc08_optimal_missions.json
   68.68 s  data/cases/uc09_warmstart_benchmark.json
```

## 2.2 The command line

```text
ofc <case.json> [<case.json> ...]     run each case file in turn
ofc --info                            version, and whether the optimiser is built in
ofc --help                            usage
```

A **case file** is a JSON file that names its use case, its aircraft, its settings and its output folder (chapter 3). The program runs the case files in the order given. A case that fails does not stop the others; the exit code reports the worst outcome (section 4.6).

Run a single case (output shortened):

```bash
./build/ofc data/cases/uc01_aircraft_report.json
```

```text
== UC-01  Aircraft report ==
name     : generic_uav
mass     : m0 11.00 kg, empty 8.50 kg, fuel 2.50 kg
geometry : S 0.550 m2, b 2.896 m, c 0.190 m, AR 15.24
aero     : CL_max 1.405, k 0.0550, (L/D)max 12.31 at CL 0.739

   h [m]        rho    V_stall    V_(L/D)  P_max [W] T_max@Vc [N]
       0     1.2250      15.10      20.82        800         22.4
     500     1.1673      15.47      21.33        757         21.2
     ...
min turn radius at V_cruise, phi_max: 76.0 m
```

Run the optimal missions:

```bash
./build/ofc data/cases/uc08_optimal_missions.json
```

**Always start the program from `code/`.** The paths inside the bundled case files (`data/aircraft/generic_uav.json`, `output/uc08`) are relative to the current folder. Started from elsewhere, the program stops with `cannot open aircraft file: data/aircraft/generic_uav.json`.

## 2.3 The bundled use cases

| Use case | Case file | What it does | Needs IPOPT |
|---|---|---|:---:|
| UC-01 | `uc01_aircraft_report.json` | Prints the aircraft data and its derived performance | |
| UC-02 | `uc02_trim.json` | Steady-flight trim with both models | |
| UC-03 | `uc03_simulation.json` | Open-loop simulation: a coordinated turn (UF) and an elevator doublet (FC) | |
| UC-04 | `uc04_transcription.json` | Size, sparsity and derivatives of the collocated problem | |
| UC-05 | `uc05_design_charts.json` | Power curves, flight envelope, trim curves, turn performance | |
| UC-06 | `uc06_design_scenarios.json` | Four reference manoeuvres flown by guidance laws with both models | |
| UC-07 | `uc07_nfz_missions.json` | Hybrid A\* paths around no-fly zones and through waypoints, flown by both models | |
| UC-08 | `uc08_optimal_missions.json` | Optimal missions: warm start, point-mass optimum, 6-DoF optimum | ✓ |
| UC-09 | `uc09_warmstart_benchmark.json` | The same missions solved by several pipelines, for comparison | ✓ |
| Tutorial | `tutorial_first_mission.json` | Three missions of increasing difficulty, used in chapter 5 | ✓ |

UC-01 to UC-07 check the aircraft, the models and the warm start. UC-08 is the optimiser itself, and most of this guide is about it. Chapter 9 describes every example in detail.

## 2.4 Where the results go

Each case file has an `output` key, a folder relative to the current directory. The folder is created if it does not exist, and files from an earlier run are overwritten.

```text
code/output/
├── uc03/ ... uc07/        CSV files of the simulations, charts and flown missions
├── uc08/                  per mission: <id>_ws.csv, <id>_uf.csv, <id>_fc.csv,
│                          <id>_path.csv, <id>_nfz.csv, <id>_points.csv;
│                          for the whole case: summary.csv and report.json
├── uc09/                  benchmark.csv, benchmark.json
└── figures/               PNG figures written by utils/plot_results.py
```

UC-01 and UC-02 print their results only. `code/output/` is listed in `.gitignore`, so nothing in it is committed. Chapter 4 describes every output file.

## 2.5 Figures

```bash
python utils/plot_results.py                           # every use case found in output/ → output/figures/
python utils/plot_results.py --optimal output/tutorial # one folder written by a UC-08 case
python utils/plot_results.py --out ../docs/assets/figures/code   # refresh the figures of this documentation
```

Without arguments, the script reads the fixed folders `output/uc03` to `output/uc09` and skips those that do not exist. For UC-08 it plots the missions whose identifier starts with `M-`. Use `--optimal <folder>` to plot the results of your own UC-08 case file, whatever its mission identifiers; the figures are then named after the folder, for example `tutorial_cover.png` and `tutorial_T-01.png`.

## 2.6 What to try next

- Read the UC-08 table printed on the console and compare it with [Worked examples: UC-08](../code/examples.md#uc-08-optimal-missions).
- Open `output/figures/uc08_cover.png` and `uc08_M-04.png`.
- Run the same case with `"print_level": 5` in its `solver` block to watch IPOPT iterate (chapter 8).

[← 1. Installation](01-installation.md) · [Next: 3. Input files →](03-inputs.md)
