# Install the official IPOPT Windows binaries (COIN-OR release, MSVC build) for
# the current user and point IPOPT_DIR at them, so that CMake finds IPOPT.
#   .\utils\install_ipopt.ps1                    # default version
#   .\utils\install_ipopt.ps1 -Version 3.14.20
# Needs Visual Studio 2022 or newer with the C++ tools to build against it
# (run.ps1 selects that generator when IPOPT_DIR is set).
param([string]$Version = "3.14.20")
$ErrorActionPreference = 'Stop'

$name = "Ipopt-$Version-win64-msvs2022-md"
$root = Join-Path $env:LOCALAPPDATA "Programs\Ipopt-$Version"
$dir = Join-Path $root $name

if (-not (Test-Path (Join-Path $dir "include\coin-or\IpTNLP.hpp"))) {
    $zip = Join-Path $env:TEMP "$name.zip"
    $url = "https://github.com/coin-or/Ipopt/releases/download/releases/$Version/$name.zip"
    Write-Host "Downloading $url"
    $ProgressPreference = 'SilentlyContinue'
    Invoke-WebRequest -Uri $url -OutFile $zip
    New-Item -ItemType Directory -Force $root | Out-Null
    Expand-Archive -Path $zip -DestinationPath $root -Force
    Remove-Item $zip
}
foreach ($f in "include\coin-or\IpTNLP.hpp", "lib\ipopt.dll.lib", "bin\ipopt-3.dll") {
    if (-not (Test-Path (Join-Path $dir $f))) { throw "unexpected layout: $f missing in $dir" }
}

[Environment]::SetEnvironmentVariable("IPOPT_DIR", $dir, "User")
$env:IPOPT_DIR = $dir
Write-Host "IPOPT $Version installed in $dir"
Write-Host "IPOPT_DIR set for this user (open a new terminal elsewhere to pick it up)."
