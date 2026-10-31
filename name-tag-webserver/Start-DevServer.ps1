#Requires -Version 7.0
[CmdletBinding()]
param(
    [ValidateRange(1, 65535)][int]$Port = 5174,
    [string]$Target = 'http://192.168.7.1',
    [switch]$Background
)
& (Join-Path $PSScriptRoot "../shared/rp2040-usb-web/tools/Start-DevServer.ps1") -ProjectRoot $PSScriptRoot -Port $Port -Target $Target -Background:$Background
