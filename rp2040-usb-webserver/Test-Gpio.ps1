#Requires -Version 7.0
[CmdletBinding()]
param([string]$BaseUri = 'http://192.168.7.1')
$ErrorActionPreference = 'Stop'
. "$PSScriptRoot/tools/Gpio-TestClient.ps1"
function Command([string]$Path, [int]$Expected = 200, [string]$Body = '') {
    Invoke-GpioTestRequest $Path Post $Expected $Body
}
$initial = Read-State
if ($initial.pins.Count -ne 30 -or @($initial.pins | Where-Object available).Count -ne 27) {
    throw 'Unexpected GPIO availability.'
}
if (@($initial.pins | Where-Object { $_.mode -eq 'pwm' -and $_.slice -eq 4 -and $_.pin -ne 25 }).Count) {
    throw 'PWM slice 4 is already in use. Run the test with no other PWM outputs on slice 4.'
}
$original = $initial.pins[25]
try {
    foreach ($pin in @(23, 24, 29)) {
        Command "/api/gpio/$pin/on" 403 > $null
        Command "/api/gpio/$pin/input-pullup" 403 > $null
    }
    foreach ($path in @('/api/gpio/30/on', '/api/gpio/-1/on', '/api/gpio/42949672960/on',
        '/api/gpio/25/pwm/9/50', '/api/gpio/25/pwm/100001/50', '/api/gpio/25/pwm/1000/101',
        '/api/gpio/25/pwm/1000/50junk')) { Command $path 400 > $null }
    Command '/api/gpio/25/on' 400 'unexpected body' > $null
    foreach ($command in @('off', 'on')) {
        $state = Command "/api/gpio/25/$command"
        if ($state.pins[25].mode -ne 'output' -or $state.pins[25].value -ne ($command -eq 'on')) {
            throw "Digital GPIO test failed: $command"
        }
    }
    $state = Command '/api/gpio/25/input-pullup'
    if ($state.pins[25].mode -ne 'input' -or $state.pins[25].pull -ne 'up') {
        throw 'Input pull-up was not enabled.'
    }
    $state = Command '/api/gpio/25/input'
    if ($state.pins[25].mode -ne 'input' -or $state.pins[25].pull -ne 'none') {
        throw 'Pull-up was not removed when switching to input.'
    }
    Command '/api/gpio/25/input-pullup' > $null
    foreach ($frequency in @(10, 1000, 100000)) {
        foreach ($duty in @(0, 50, 100)) {
            $state = Command "/api/gpio/25/pwm/$frequency/$duty"
            $pin = $state.pins[25]
            if ($pin.mode -ne 'pwm' -or $pin.pull -ne 'none' -or $pin.frequency_hz -ne $frequency -or $pin.duty_percent -ne $duty -or
                [Math]::Abs($pin.actual_frequency_hz - $frequency) -gt [Math]::Max(1, $frequency * 0.01)) {
                throw "PWM test failed: $frequency Hz, $duty percent"
            }
        }
    }
    Command '/api/gpio/9/pwm/100000/50' 409 > $null # alias of LED's channel
    Command '/api/gpio/8/pwm/2000/50' 409 > $null # other frequency on same slice
    $state = Read-State
    if ($state.pins[8].mode -ne $initial.pins[8].mode -or $state.pins[9].mode -ne $initial.pins[9].mode -or
        $state.pins[25].frequency_hz -ne 100000 -or $state.pins[25].duty_percent -ne 100) {
        throw 'PWM conflict changed an output.'
    }
    $state = Command '/api/gpio/25/input'
    if ($state.pins[25].mode -ne 'input' -or $state.pins[25].pull -ne 'none') { throw 'Input mode failed.' }
    Command '/api/gpio/25/input-pullup' > $null
    $state = Command '/api/gpio/25/on'
    if ($state.pins[25].pull -ne 'none') { throw 'Pull-up remained active on digital output.' }
    Write-Host 'GPIO/PWM OK: digital output, input with/without pull-up, PWM, conflicts and invalid commands.'
} catch {
    $primaryError = $_
    throw
} finally {
    Restore-GpioTestState -Pins ($original) -PrimaryError $primaryError
}
