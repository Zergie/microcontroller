#Requires -Version 7.0
[CmdletBinding()]
param()
& (Join-Path $PSScriptRoot "../shared/rp2040-usb-web/tools/Stop-DevServer.ps1") -ProjectRoot $PSScriptRoot @PSBoundParameters
