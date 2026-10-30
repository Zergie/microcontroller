#Requires -Version 7.0
[CmdletBinding()]
param(
    [ValidateSet('pico', 'pico_w')][string]$Board = 'pico',
    [switch]$RebuildToolchain
)
$ErrorActionPreference = 'Stop'
$image = 'rp2040-usb-webserver-toolchain:sdk-2.3.1-ncm-ios-v1'
& docker info --format '{{.OSType}}'
if ($LASTEXITCODE -ne 0) { throw 'Docker Desktop must be running Linux containers.' }
& docker image inspect $image *> $null
if ($RebuildToolchain -or $LASTEXITCODE -ne 0) {
    & docker build --tag $image $PSScriptRoot
    if ($LASTEXITCODE -ne 0) { throw 'Could not build the Docker toolchain.' }
}
$build = "build/$Board"
& docker run --rm --mount "type=bind,source=$PSScriptRoot,target=/project" $image `
    cmake -S /project -B "/project/$build" -G Ninja "-DPICO_BOARD=$Board" -DCMAKE_BUILD_TYPE=Release
if ($LASTEXITCODE -ne 0) { throw 'CMake configuration failed.' }
& docker run --rm --mount "type=bind,source=$PSScriptRoot,target=/project" $image `
    cmake --build "/project/$build" --parallel
if ($LASTEXITCODE -ne 0) { throw 'Firmware build failed.' }
$uf2 = Join-Path $PSScriptRoot "$build/rp2040_usb_webserver.uf2"
if (!(Test-Path -LiteralPath $uf2)) { throw "UF2 not found: $uf2" }
Get-Item -LiteralPath $uf2
Get-FileHash -LiteralPath $uf2 -Algorithm SHA256
