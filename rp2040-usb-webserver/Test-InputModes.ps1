#Requires -Version 7.0
[CmdletBinding()]
param([string]$BaseUri = 'http://192.168.7.1')
$ErrorActionPreference = 'Stop'
. "$PSScriptRoot/tools/Gpio-TestClient.ps1"
function Command([string]$Path, [int]$Expected = 200, [string]$Body = '') {
    Invoke-GpioTestRequest "/api/gpio/$Path" Post $Expected $Body
}
$initial = Read-State
try {
    foreach ($command in @('input-pulldown','input-buskeeper','analog','open-drain/low','open-drain/release')) {
        foreach ($pin in @(23,24,29)) { Command "$pin/$command" 403 > $null }
    }
    Command '25/analog' 400 > $null
    Command '25/open-drain/high' 404 > $null
    foreach ($pair in @(@('input-pulldown','down'),@('input-buskeeper','keeper'),@('input-pullup','up'),@('input','none'))) {
        $state = Command "25/$($pair[0])"
        if ($state.pins[25].mode -ne 'input' -or $state.pins[25].pull -ne $pair[1]) { throw "Input test: $($pair[0])" }
    }
    foreach ($action in @('low','release')) {
        $state = Command "25/open-drain/$action"
        $pin = $state.pins[25]
        if ($pin.mode -ne 'open-drain' -or $pin.pull -ne 'none' -or $pin.output_latch -ne $false -or
            $pin.output_enabled -ne ($action -eq 'low')) { throw "Open Drain test: $action" }
    }
    $state = Command '25/on'
    if ($state.pins[25].mode -ne 'output' -or $state.pins[25].value -ne $true) { throw 'Transition from Open Drain to output' }
    # ADC tests configure only inputs, without driving any external pin.
    foreach ($pin in @(26,27,28)) { Command "$pin/analog" > $null }
    $snapshot = Read-State
    foreach ($number in @(26,27,28)) {
        $pin = $snapshot.pins[$number]
        if ($pin.mode -ne 'analog' -or $pin.pull -ne 'none' -or $null -ne $pin.value -or
            $null -eq $pin.adc_raw -or $pin.adc_raw -lt 0 -or $pin.adc_raw -gt 4095 -or
            $pin.millivolts -ne [Math]::Floor(($pin.adc_raw * 3300 + 2048) / 4096)) {
            throw "ADC test: GP$number"
        }
        $state = Command "$number/input"
        if ($state.pins[$number].mode -ne 'input' -or $state.pins[$number].pull -ne 'none' -or
            $null -eq $state.pins[$number].value) { throw "Transition from ADC to digital input: GP$number" }
    }
    Write-Host 'Modes OK: pull-down, bus keeper, Open Drain (LOW/Hi-Z), three ADC channels and return transitions.'
} catch {
    $primaryError = $_
    throw
} finally {
    Restore-GpioTestState -Pins (@(25,26,27,28) | ForEach-Object { $initial.pins[$_] }) -PrimaryError $primaryError
}
