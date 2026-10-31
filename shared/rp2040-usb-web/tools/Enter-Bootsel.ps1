#Requires -Version 7.0
[CmdletBinding()]
param(
    [string]$BaseUri = 'http://192.168.7.1',
    [ValidateRange(2, 120)][int]$WaitSeconds = 20
)
$ErrorActionPreference = 'Stop'
function Find-Rp2040Drives {
    foreach ($drive in Get-PSDrive -PSProvider FileSystem) {
        $info = Join-Path $drive.Root 'INFO_UF2.TXT'
        if ((Test-Path -LiteralPath $info) -and (Get-Content -LiteralPath $info -Raw) -match 'Board-ID:\s*RPI-RP2') {
            $drive.Root
        }
    }
}
$existing = @(Find-Rp2040Drives)
$uri = $BaseUri.TrimEnd('/') + '/api/system/bootsel'
$response = Invoke-WebRequest $uri -Method Post -Body '' -NoProxy -TimeoutSec 5
$accepted = $response.Content | ConvertFrom-Json
if ($response.StatusCode -ne 202 -or $accepted.mode -ne 'bootsel') { throw 'BOOTSEL reboot was not acknowledged.' }
Write-Host "HTTP 202: BOOTSEL reboot in $($accepted.delay_ms) ms acknowledged."
$deadline = [DateTime]::UtcNow.AddSeconds($WaitSeconds)
do {
    $targets = @(Find-Rp2040Drives | Where-Object { $_ -notin $existing })
    if ($targets.Count -gt 1) { throw 'Multiple new BOOTSEL drives detected; target is ambiguous.' }
    if ($targets.Count -eq 1) {
        Write-Host "RP2040 in BOOTSEL mode: $($targets[0])"
        return $targets[0]
    }
    Start-Sleep -Milliseconds 250
} while ([DateTime]::UtcNow -lt $deadline)
throw 'API acknowledged the reboot, but no new RP2040 BOOTSEL drive was found.'
