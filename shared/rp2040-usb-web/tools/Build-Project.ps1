#Requires -Version 7.0
[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$ProjectRoot,
    [Parameter(Mandatory)][string]$FirmwareTarget,
    [ValidateSet('pico', 'pico_w')][string]$Board = 'pico',
    [switch]$RebuildToolchain
)
$ErrorActionPreference = 'Stop'
$workspace = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$project = Split-Path $ProjectRoot -Leaf
$image = 'rp2040-usb-webserver-toolchain:sdk-2.3.1-ncm-ios-v1'
& docker info --format '{{.OSType}}'
if ($LASTEXITCODE -ne 0) { throw 'Docker Desktop must be running Linux containers.' }
& docker image inspect $image *> $null
if ($RebuildToolchain -or $LASTEXITCODE -ne 0) {
    & docker build --tag $image (Join-Path $workspace 'toolchains/rp2040')
    if ($LASTEXITCODE -ne 0) { throw 'Could not build the Docker toolchain.' }
}
$build = "/workspace/$project/build/$Board"
# Migrate caches made before the shared workspace mount without forcing a full
# reconfiguration/recompile on every subsequent build.
$fresh = @()
$cache = Join-Path $ProjectRoot "build/$Board/CMakeCache.txt"
if (Test-Path -LiteralPath $cache) {
    $configuration = Get-Content -LiteralPath $cache -Raw
    if ($configuration -notmatch "(?m)^CMAKE_HOME_DIRECTORY:INTERNAL=/workspace/$([regex]::Escape($project))\r?$") {
        $fresh = @('--fresh')
    }
}
& docker run --rm --mount "type=bind,source=$workspace,target=/workspace" $image `
    cmake @fresh -S "/workspace/$project" -B $build -G Ninja "-DPICO_BOARD=$Board" -DCMAKE_BUILD_TYPE=Release
if ($LASTEXITCODE -ne 0) { throw 'CMake configuration failed.' }
& docker run --rm --mount "type=bind,source=$workspace,target=/workspace" $image `
    cmake --build $build --parallel
if ($LASTEXITCODE -ne 0) { throw 'Firmware build failed.' }
$uf2 = Join-Path $ProjectRoot "build/$Board/$FirmwareTarget.uf2"
if (!(Test-Path -LiteralPath $uf2)) { throw "UF2 not found: $uf2" }
Get-Item -LiteralPath $uf2
Get-FileHash -LiteralPath $uf2 -Algorithm SHA256
