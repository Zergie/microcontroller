#Requires -Version 7.0
[CmdletBinding()]
param(
    [string]$BaseUri = 'http://192.168.7.1',
    [ValidateRange(2, 120)][int]$WaitSeconds = 20
)
& (Join-Path $PSScriptRoot "../shared/rp2040-usb-web/tools/Enter-Bootsel.ps1") @PSBoundParameters
