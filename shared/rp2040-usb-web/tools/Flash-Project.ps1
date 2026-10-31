#Requires -Version 7.0
[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$ProjectRoot,
    [Parameter(Mandatory)][string]$FirmwareTarget,
    [ValidateSet('pico', 'pico_w')][string]$Board = 'pico',
    [string]$Drive,
    [switch]$EnterBootsel,
    [string]$BaseUri = 'http://192.168.7.1',
    [ValidateRange(0, 600)][int]$WaitSeconds = 30
)
$ErrorActionPreference = 'Stop'
$uf2 = Join-Path $ProjectRoot "build/$Board/$FirmwareTarget.uf2"
if (!(Test-Path -LiteralPath $uf2)) { throw 'Run ./Build-Project.ps1 first.' }
if ($EnterBootsel) {
    $detected = & (Join-Path $PSScriptRoot 'Enter-Bootsel.ps1') -BaseUri $BaseUri -WaitSeconds ([Math]::Max(2, [Math]::Min(120, $WaitSeconds)))
    if ($Drive -and [IO.Path]::GetFullPath($Drive) -ne [IO.Path]::GetFullPath($detected)) {
        throw "API board appeared at $detected, not at the specified target $Drive."
    }
    $Drive = $detected
}
function Test-Rp2040Drive([string]$Root) {
    $info = Join-Path $Root 'INFO_UF2.TXT'
    return (Test-Path -LiteralPath $info) -and
        ((Get-Content -LiteralPath $info -Raw) -match 'Board-ID:\s*RPI-RP2')
}
$deadline = [DateTime]::UtcNow.AddSeconds($WaitSeconds)
do {
    $roots = if ($Drive) { @($Drive) } else {
        @(Get-PSDrive -PSProvider FileSystem | ForEach-Object { $_.Root })
    }
    $targets = @($roots | Where-Object { Test-Rp2040Drive $_ })
    if ($targets.Count -gt 1) { throw 'Multiple RP2040 boards found. Use -Drive F:\ to select one target.' }
    if ($targets.Count -eq 1) { break }
    if ([DateTime]::UtcNow -ge $deadline) { throw 'No RP2040 BOOTSEL drive found. Hold BOOTSEL while connecting.' }
    Start-Sleep -Seconds 1
} while ($true)
Write-Host "Writing $uf2 to $($targets[0]); the board will reboot afterward."
Copy-Item -LiteralPath $uf2 -Destination (Join-Path $targets[0] 'firmware.uf2')
Write-Host 'UF2 copy complete. Verify operation on the device.'
