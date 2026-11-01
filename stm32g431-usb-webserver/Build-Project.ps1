#requires -Version 7.0
<#
.SYNOPSIS
Builds STM32G431 firmware with Docker and reports flash/RAM allocations.
.DESCRIPTION
Fetches pinned dependencies automatically when missing. Docker Desktop's Linux
engine must be running. Use -FetchDependencies to check existing dependency pins.
#>
[CmdletBinding()]
param([switch]$FetchDependencies)
$ErrorActionPreference = 'Stop'
if (!(Get-Command docker -ErrorAction SilentlyContinue)) { throw 'Docker Desktop is required. Install it and start the Linux engine.' }
docker info *> $null
if ($LASTEXITCODE) { throw 'Docker is unavailable. Start Docker Desktop with Linux containers, then retry.' }
foreach ($dependency in 'tinyusb', 'lwip', 'cmsis', 'cmsis-device-g4') {
    if (!(Test-Path "$PSScriptRoot/deps/$dependency/.git")) { $FetchDependencies = $true }
}
if ($FetchDependencies) { & "$PSScriptRoot/Fetch-Dependencies.ps1" }
$image = 'rp2040-usb-webserver-toolchain:sdk-2.3.1-ncm-ios-v1'
docker image inspect $image *> $null
if ($LASTEXITCODE) {
    docker build -t $image "$PSScriptRoot/../toolchains/rp2040"
    if ($LASTEXITCODE) { throw 'Toolchain image build failed' }
}
New-Item -ItemType Directory -Force "$PSScriptRoot/build" | Out-Null
$mount = "type=bind,source=$PSScriptRoot,target=/project,readonly"
$sharedMount = "type=bind,source=$([IO.Path]::GetFullPath("$PSScriptRoot/../shared")),target=/shared,readonly"
$outputMount = "type=bind,source=$PSScriptRoot/build,target=/project/build"
docker run --rm --mount $mount --mount $outputMount --mount $sharedMount $image cmake -S /project -B /project/build -G Ninja '-DCMAKE_TOOLCHAIN_FILE=/project/cmake/arm-none-eabi.cmake' '-DCMAKE_BUILD_TYPE=MinSizeRel'
if ($LASTEXITCODE) { throw 'CMake configure failed' }
docker run --rm --mount $mount --mount $outputMount --mount $sharedMount $image cmake --build /project/build
if ($LASTEXITCODE) { throw 'Firmware build failed' }
docker run --rm --mount $mount --mount $outputMount --mount $sharedMount $image python3 /project/tools/report_memory.py /project/build/stm32g431_usb_webserver.elf
if ($LASTEXITCODE) { throw 'Memory report failed' }
Write-Host "Firmware: $PSScriptRoot/build/stm32g431_usb_webserver.hex"
