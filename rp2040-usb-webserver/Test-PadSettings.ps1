#Requires -Version 7.0
[CmdletBinding()]
param([string]$BaseUri = 'http://192.168.7.1')
$ErrorActionPreference = 'Stop'
. "$PSScriptRoot/tools/Gpio-TestClient.ps1"
function Command([string]$Path, [int]$Expected = 200, [string]$Body = '') {
    Invoke-GpioTestRequest "/api/gpio/$Path" Post $Expected $Body
}
$initial = Read-State
$original = $initial.pins[25]
try {
    foreach ($mode in @('off','pwm/1000/50','open-drain/release')) {
        foreach ($drive in @(2,4,8,12)) {
            foreach ($slew in @('slow','fast')) {
                $state = Command "25/$mode/drive/$drive/$slew"
                if ($state.pins[25].drive_strength_ma -ne $drive -or $state.pins[25].slew_rate -ne $slew) {
                    throw "Drive setting was not applied: $mode, $drive mA, $slew"
                }
            }
        }
    }
    foreach ($mode in @('input','input-pullup','input-pulldown','input-buskeeper')) {
        foreach ($hysteresis in @('on','off')) {
            $state = Command "25/$mode/hysteresis/$hysteresis"
            if ($state.pins[25].mode -ne 'input' -or $state.pins[25].hysteresis -ne ($hysteresis -eq 'on')) {
                throw "Hysteresis was not applied: $mode, $hysteresis"
            }
        }
    }
    $before = (Read-State).pins[25] | ConvertTo-Json -Compress
    foreach ($path in @('25/on/drive/3/slow','25/on/drive/16/fast','25/on/drive/4/turbo',
        '25/on/drive/4/slowjunk','25/input/hysteresis/maybe','25/input/drive/4/slow',
        '25/on/hysteresis/off','25/analog/hysteresis/on','25/analog/drive/4/slow',
        '25/pwm/9/50/drive/2/slow','25/on/drive/4/fast/hysteresis/on')) {
        Command $path 400 > $null
        if (((Read-State).pins[25] | ConvertTo-Json -Compress) -ne $before) { throw "Rejected setting changed the state: $path" }
    }
    foreach ($pin in @(23,24,29)) {
        Command "$pin/on/drive/4/fast" 403 > $null
        Command "$pin/input/hysteresis/off" 403 > $null
    }
    Command '25/pwm/1000/50/drive/8/fast' > $null
    $before8 = (Read-State).pins[8] | ConvertTo-Json -Compress
    Command '8/pwm/2000/50/drive/12/slow' 409 > $null
    if (((Read-State).pins[8] | ConvertTo-Json -Compress) -ne $before8) { throw 'PWM conflict changed pad settings.' }
    Write-Host 'Pad settings OK: 2/4/8/12 mA, Slow/Fast, hysteresis, mode restrictions and rejected commands.'
} catch {
    $primaryError = $_
    throw
} finally {
    Restore-GpioTestState -Pins ($original) -PrimaryError $primaryError
}
