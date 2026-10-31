#Requires -Version 7.0
[CmdletBinding()]
param([Parameter(Mandatory)][string]$ProjectRoot, [switch]$NoCache)
$ErrorActionPreference = 'Stop'
$ProjectRoot = (Resolve-Path -LiteralPath $ProjectRoot).Path
$workspace = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$project = Split-Path $ProjectRoot -Leaf
$image = 'rp2040-usb-webserver-toolchain:sdk-2.3.1-ncm-ios-v1'
$runImage = $image
$cacheArguments = @()
if (!$NoCache) {
    $cache = Join-Path $ProjectRoot 'build/native-test-cache'
    New-Item -ItemType Directory -Path $cache -Force | Out-Null
    $imageId = & docker image inspect --format '{{.Id}}' $image
    if ($LASTEXITCODE -ne 0) { throw 'Cannot identify native test toolchain image.' }
    $runImage = $imageId
    $cacheArguments = @('--mount', "type=bind,source=$cache,target=/native-test-cache", `
        '--env', 'NATIVE_TEST_CACHE=/native-test-cache', '--env', "NATIVE_TEST_TOOLCHAIN=$imageId")
}
& docker run --rm --mount "type=bind,source=$workspace,target=/workspace,readonly" @cacheArguments `
    $runImage python3 "/workspace/$project/tests/run_native.py"
if ($LASTEXITCODE -ne 0) { throw 'Native regression tests failed.' }
