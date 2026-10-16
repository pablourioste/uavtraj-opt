# Build, test and run every public use case, then plot the results (Windows PowerShell).
#   .\run.ps1          build + tests + all use cases + figures (output/figures/)
#   .\run.ps1 -t       build + tests only
param([switch]$t)
$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
if ($LASTEXITCODE) { exit $LASTEXITCODE }
cmake --build build --config Release
if ($LASTEXITCODE) { exit $LASTEXITCODE }
ctest --test-dir build --output-on-failure -C Release
if ($LASTEXITCODE) { exit $LASTEXITCODE }

if (-not $t) {
    $exe = if (Test-Path build/ofc.exe) { 'build/ofc.exe' } else { 'build/Release/ofc.exe' }
    & $exe (Get-ChildItem data/cases/uc0*.json | ForEach-Object { $_.FullName })
    if (Get-Command python -ErrorAction SilentlyContinue) {
        python utils/plot_results.py
    }
}
