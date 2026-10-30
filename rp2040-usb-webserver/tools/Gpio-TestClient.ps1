#Requires -Version 7.0
# Test-only HTTP client. Dot-source after setting the caller's $BaseUri.
function Invoke-GpioTestRequest {
    param([string]$Path, [ValidateSet('Get', 'Post')][string]$Method = 'Get',
        [int]$Expected = 200, [string]$Body = '', [int]$TimeoutSeconds = 6)
    $options = @{ Uri = "$($BaseUri.TrimEnd('/'))$Path"; Method = $Method;
        NoProxy = $true; TimeoutSec = $TimeoutSeconds; SkipHttpErrorCheck = $true }
    if ($Method -eq 'Post') { $options.Body = $Body }
    $response = Invoke-WebRequest @options
    if ($response.StatusCode -ne $Expected) {
        throw "$Path : HTTP $($response.StatusCode), expected $Expected. $($response.Content)"
    }
    return ($response.Content | ConvertFrom-Json)
}

function Read-State { Invoke-GpioTestRequest '/api/gpio.json' }

function Restore-GpioTestState {
    param([object[]]$Pins, $PrimaryError)
    $errors = [System.Collections.Generic.List[string]]::new()
    foreach ($pin in $Pins) {
        try {
            $number = $pin.pin
            $current = (Read-State).pins[$number]
            # Restore hidden pad categories only if changed. In particular, the
            # ADC tests never introduce temporary output modes on external pins.
            $schmitt = if ($pin.hysteresis) { 'on' } else { 'off' }
            if ($current.hysteresis -ne $pin.hysteresis) {
                Invoke-GpioTestRequest "/api/gpio/$number/input/hysteresis/$schmitt" Post > $null
            }
            if ($current.drive_strength_ma -ne $pin.drive_strength_ma -or $current.slew_rate -ne $pin.slew_rate) {
                Invoke-GpioTestRequest "/api/gpio/$number/off/drive/$($pin.drive_strength_ma)/$($pin.slew_rate)" Post > $null
            }
            $action = switch ($pin.mode) {
                'input' {
                    $inputMode = switch ($pin.pull) { 'up' { 'input-pullup' } 'down' { 'input-pulldown' }
                        'keeper' { 'input-buskeeper' } default { 'input' } }
                    "$inputMode/hysteresis/$schmitt"
                }
                'analog' { 'analog' }
                'pwm' { "pwm/$($pin.frequency_hz)/$($pin.duty_percent)" }
                'open-drain' { if ($pin.output_enabled) { 'open-drain/low' } else { 'open-drain/release' } }
                'output' { if ($pin.value) { 'on' } else { 'off' } }
                default { throw "Unknown original mode: $($pin.mode)" }
            }
            Invoke-GpioTestRequest "/api/gpio/$number/$action" Post > $null
        } catch { $errors.Add("GP$($pin.pin): $($_.Exception.Message)") }
    }
    if ($errors.Count) {
        $primary = if ($PrimaryError) { "Test failure: $($PrimaryError.Exception.Message)`n" } else { '' }
        throw "${primary}Restoration failed:`n$($errors -join "`n")"
    }
}
