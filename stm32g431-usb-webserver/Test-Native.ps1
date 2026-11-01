$ErrorActionPreference = 'Stop'
New-Item -ItemType Directory -Force "$PSScriptRoot/build" | Out-Null
docker run --rm --mount "type=bind,source=$PSScriptRoot,target=/project,readonly" --mount "type=bind,source=$PSScriptRoot/build,target=/project/build" --mount "type=bind,source=$([IO.Path]::GetFullPath("$PSScriptRoot/../shared")),target=/shared,readonly" rp2040-usb-webserver-toolchain:sdk-2.3.1-ncm-ios-v1 python3 /project/tools/test_native.py
if ($LASTEXITCODE) { throw 'Native network test failed' }
