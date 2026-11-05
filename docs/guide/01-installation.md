# 1. Installation

This chapter builds the `ofc` program and its tests, with or without the IPOPT optimiser, and shows how to confirm which of the two you have.

## 1.1 What is needed

| Component | Version | Needed for |
|---|---|---|
| C++ compiler | C++17 with `<filesystem>` (GCC ≥ 9, Clang ≥ 9, MSVC 2019 or newer) | Everything |
| CMake | ≥ 3.14 | Everything |
| IPOPT | 3.11 or newer (tested with 3.11.9 on Linux and 3.14.20 on Windows) | Use cases UC-08 and UC-09, and the optimisation tests |
| `pkg-config` | any | Finding IPOPT on Linux and macOS |
| Python 3 with `numpy` and `matplotlib` | any recent | The figures only |

The only bundled third-party code is `nlohmann/json` (a header-only JSON library, MIT licence) in `code/third_party/`. IPOPT is linked, never bundled.

IPOPT is optional at build time. Without it, the program builds, the tests run, and use cases UC-01 to UC-07 work; UC-08 and UC-09 print a notice and are skipped. Every other part of this guide that runs an optimisation needs IPOPT.

## 1.2 Get the code

```bash
git clone https://github.com/pablourioste/uavtraj-opt.git
cd uavtraj-opt/code
```

All later commands are run from this `code/` folder.

## 1.3 Install IPOPT

### Linux and WSL (Ubuntu, Debian)

```bash
sudo apt update
sudo apt install build-essential cmake coinor-libipopt-dev pkg-config
```

The distribution package provides IPOPT with the MUMPS linear solver. CMake finds it through `pkg-config`.

### macOS (Homebrew)

```bash
brew install cmake ipopt pkg-config
```

### Windows (PowerShell)

Install Visual Studio 2022 or newer with the "Desktop development with C++" workload, and CMake (included with Visual Studio, or from cmake.org). Then, from `code/`:

```powershell
.\utils\install_ipopt.ps1
```

The script downloads the official COIN-OR binaries (IPOPT 3.14.20, built with MSVC) into `%LOCALAPPDATA%\Programs\Ipopt-3.14.20`, checks the expected files, and sets the user environment variable `IPOPT_DIR` to that folder. Another version can be requested with `.\utils\install_ipopt.ps1 -Version 3.14.x`. Terminals opened before the script ran do not see `IPOPT_DIR`; `run.ps1` reads it from the user environment anyway.

If PowerShell refuses to run scripts, allow local scripts for the current user once:

```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```

### Other systems

Any IPOPT installation works if CMake can find it in one of two ways:

1. through `pkg-config` (the module is called `ipopt`), or
2. through a folder that contains `include/coin-or/IpTNLP.hpp` (or `include/coin/`), the library in `lib/` and, on Windows, the DLLs in `bin/`. Pass it as `-DIPOPT_DIR=<folder>` or as the environment variable `IPOPT_DIR`.

## 1.4 Build

The scripts configure, build and test in one step:

```bash
bash run.sh -t        # Linux, macOS, Git Bash
```

```powershell
.\run.ps1 -t          # Windows PowerShell
```

Without `-t`, the scripts also run every bundled use case and plot the results (chapter 2).

The equivalent manual commands are:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build --output-on-failure -C Release
```

On Windows with the prebuilt IPOPT, the build must use the Visual Studio generator, because the IPOPT binaries are built with MSVC:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

`run.ps1` selects this generator automatically when `IPOPT_DIR` is set, and resets the CMake cache if `build/` was configured earlier with another generator. With the prebuilt IPOPT, CMake also copies the IPOPT DLLs next to `ofc.exe` and `ofc_tests.exe` after each build, so no change to `PATH` is needed.

### Build products

| File | Content |
|---|---|
| `build/ofc` (Windows: `build\Release\ofc.exe`) | The command-line program |
| `build/ofc_tests` (Windows: `build\Release\ofc_tests.exe`) | The unit tests, run by `ctest` |
| `build/libofc_core.a` (Windows: `ofc_core.lib`) | The library with everything except `main.cpp`, for use in your own programs (chapter 11) |

### CMake options

| Option | Values | Default | Effect |
|---|---|---|---|
| `OFC_IPOPT` | `AUTO`, `ON`, `OFF` | `AUTO` | `AUTO` links IPOPT if it is found; `ON` stops the configuration with an error if it is not; `OFF` never links it |
| `IPOPT_DIR` | folder | environment variable `IPOPT_DIR` | Prebuilt IPOPT, used when `pkg-config` does not find one |
| `OFC_BUILD_TESTS` | `ON`, `OFF` | `ON` | Build `ofc_tests` |
| `CMAKE_BUILD_TYPE` | `Release`, `Debug`, ... | `Release` | Optimisation level of single-configuration generators |

Use `-DOFC_IPOPT=ON` whenever the optimiser is required: a build that silently lacks IPOPT is otherwise easy to miss.

## 1.5 Check the installation

During configuration, CMake reports what it found. One of these three lines appears:

```text
-- IPOPT 3.11.9: trajectory optimisation enabled                     (pkg-config)
-- IPOPT from C:/Users/.../Ipopt-3.14.20-win64-msvs2022-md: trajectory optimisation enabled
-- IPOPT not found: building without the optimiser (use cases UC-08/09 will be skipped)
```

After the build, the program states it directly:

```bash
./build/ofc --info
```

```text
ofc 0.2.0
optimiser: IPOPT (UC-08 and UC-09 available)
```

A build without IPOPT prints `optimiser: not built (UC-08 and UC-09 are skipped; see code/README.md)` instead.

The tests check the models, the analytic derivatives against finite differences, the planner, the warm start and, when IPOPT is present, complete optimisations. A successful run ends with:

```text
100% tests passed, 0 tests failed out of 1
```

(`ctest` reports one test because all checks run inside a single executable; the individual checks are printed with `--output-on-failure` only when one fails.)

## 1.6 Figures and documentation (optional)

```bash
pip install numpy matplotlib          # figures: utils/plot_results.py
pip install mkdocs-material           # this documentation as a local website
mkdocs serve                          # run from the repository root, then open http://127.0.0.1:8000
```

## 1.7 Code formatting (contributors only)

The C++ sources follow `code/.clang-format` (clang-format 18, four-space indentation, 120 columns), and the continuous integration rejects unformatted code:

```bash
pip install clang-format==18.1.8
clang-format -i include/ofc/*.hpp src/*.cpp src/*/*.cpp tests/*.cpp tests/*.hpp
```

If the build or the IPOPT detection fails, see [10. Troubleshooting](10-troubleshooting.md#101-build-and-installation).

[← User guide](index.md) · [Next: 2. First run →](02-first-run.md)
