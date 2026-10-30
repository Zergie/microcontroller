#Requires -Version 7.0
[CmdletBinding()]
param(
    [ValidateRange(1, 600)][int]$WaitSeconds = 60,
    [switch]$TestLed,
    [string]$BaseUri = 'http://192.168.7.1'
)
$ErrorActionPreference = 'Stop'
. "$PSScriptRoot/tools/Gpio-TestClient.ps1"
$deadline = [DateTime]::UtcNow.AddSeconds($WaitSeconds)
do {
    try {
        $response = Invoke-WebRequest -Uri "$($BaseUri.TrimEnd('/'))/" -NoProxy -TimeoutSec 3
        if ($response.StatusCode -eq 200 -and $response.Content -match 'RP2040 USB Webserver') {
            Write-Host "HTTP 200: RP2040 USB Webserver reachable at $BaseUri/"
            break
        }
    } catch { $lastError = $_.Exception.Message }
    Start-Sleep -Seconds 1
} while ([DateTime]::UtcNow -lt $deadline)
if (!$response -or $response.StatusCode -ne 200 -or $response.Content -notmatch 'RP2040 USB Webserver') {
    throw "HTTP test failed: $lastError"
}
if ($TestLed) {
    $original = (Read-State).pins[25]
    try {
        foreach ($command in @('on', 'off')) {
            $expected = $command -eq 'on'
            $result = Invoke-GpioTestRequest "/api/led/$command" Post -TimeoutSeconds 3
            $state = Invoke-GpioTestRequest "/api/led.json" -TimeoutSeconds 3
            if ($result.on -ne $expected -or $state.on -ne $expected) {
                throw "LED test failed: $command"
            }
        }
        Write-Host 'LED test passed: on/off and GPIO state verified over HTTP.'
    } catch {
        $primaryError = $_
        throw
    } finally {
        Restore-GpioTestState -Pins @($original) -PrimaryError $primaryError
    }
}
