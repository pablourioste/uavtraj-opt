# Build, test and run every public use case, then plot the results (Windows PowerShell).
#   .\run.ps1          build + tests + all use cases + figures (output/figures/)
#   .\run.ps1 -t       build + tests only
# With IPOPT installed by utils/install_ipopt.ps1 (IPOPT_DIR set), the build uses
# Visual Studio, as the IPOPT binaries require, and UC-08/09 run the optimiser.
param([switch]$t)
$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot

# IPOPT_DIR may have been set for the user after this terminal was opened.
if (-not $env:IPOPT_DIR) { $env:IPOPT_DIR = [Environment]::GetEnvironmentVariable('IPOPT_DIR', 'User') }

$gen = @()
if ($env:IPOPT_DIR) {
    $gen = @('-G', 'Visual Studio 17 2022', '-A', 'x64')
    # A build folder configured with another generator cannot be reused: reset its cache.
    $cache = 'build/CMakeCache.txt'
    if (Test-Path $cache) {
        $old = (Select-String -Path $cache -Pattern '^CMAKE_GENERATOR:INTERNAL=(.*)$').Matches.Groups[1].Value
        if ($old -ne $gen[1]) {
            Write-Host "build/ was configured with '$old'; resetting its CMake cache for '$($gen[1])'"
            Remove-Item -Recurse -Force 'build/CMakeCache.txt', 'build/CMakeFiles'
        }
    }
}

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release @gen
if ($LASTEXITCODE) { exit $LASTEXITCODE }
cmake --build build --config Release
if ($LASTEXITCODE) { exit $LASTEXITCODE }
ctest --test-dir build --output-on-failure -C Release
if ($LASTEXITCODE) { exit $LASTEXITCODE }

if (-not $t) {
    $exe = if (Test-Path build/Release/ofc.exe) { 'build/Release/ofc.exe' } else { 'build/ofc.exe' }
    & $exe (Get-ChildItem data/cases/uc0*.json | ForEach-Object { $_.FullName })
    if (Get-Command python -ErrorAction SilentlyContinue) {
        python utils/plot_results.py
    }
}
