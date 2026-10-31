#Requires -Version 7.0
[CmdletBinding()]
param(
    [ValidateSet('pico', 'pico_w')][string]$Board = 'pico',
    [string]$Drive,
    [switch]$EnterBootsel,
    [string]$BaseUri = 'http://192.168.7.1',
    [ValidateRange(0, 600)][int]$WaitSeconds = 30
)
& (Join-Path $PSScriptRoot "../shared/rp2040-usb-web/tools/Flash-Project.ps1") -ProjectRoot $PSScriptRoot -FirmwareTarget rp2040_usb_webserver @PSBoundParameters
