#Requires -Version 7.0
[CmdletBinding()]
param([switch]$NoCache)
& (Join-Path $PSScriptRoot "../shared/rp2040-usb-web/tools/Test-Native.ps1") -ProjectRoot $PSScriptRoot @PSBoundParameters
