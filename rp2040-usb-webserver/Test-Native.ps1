#Requires -Version 7.0
[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
& docker run --rm --mount "type=bind,source=$PSScriptRoot,target=/project,readonly" `
    rp2040-usb-webserver-toolchain:sdk-2.3.1-ncm-ios-v1 python3 /project/tests/run_native.py
if ($LASTEXITCODE -ne 0) { throw 'Native regression tests failed.' }
