#!/usr/bin/env bash
# Build, test and run every public use case, then plot the results.
#   bash run.sh        build + tests + all use cases + figures (output/figures/)
#   bash run.sh -t     build + tests only
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${PROJECT_DIR}/build"
cd "${PROJECT_DIR}"

echo "-> Configuring"
cmake -S . -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release
echo "-> Building"
cmake --build "${BUILD_DIR}" --config Release
echo "-> Testing"
ctest --test-dir "${BUILD_DIR}" --output-on-failure -C Release

if [ "${1:-}" != "-t" ]; then
    echo "-> Running use cases"
    "${BUILD_DIR}/ofc" data/cases/uc0*.json
    if command -v python3 >/dev/null 2>&1; then
        echo "-> Plotting"
        python3 utils/plot_results.py || echo "(plots skipped: needs numpy and matplotlib)"
    fi
fi
