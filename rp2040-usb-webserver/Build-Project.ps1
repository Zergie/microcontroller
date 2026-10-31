#Requires -Version 7.0
[CmdletBinding()]
param(
    [ValidateSet('pico', 'pico_w')][string]$Board = 'pico',
    [switch]$RebuildToolchain
)
& (Join-Path $PSScriptRoot "../shared/rp2040-usb-web/tools/Build-Project.ps1") -ProjectRoot $PSScriptRoot -FirmwareTarget rp2040_usb_webserver @PSBoundParameters
