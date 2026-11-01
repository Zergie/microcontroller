#requires -Version 7.0
<#
.SYNOPSIS
Programs and verifies the STM32G431CBU6 using its factory USB DFU bootloader.
.EXAMPLE
./Flash-Project.ps1 -Build
.EXAMPLE
./Flash-Project.ps1 -Mode SWD -SerialNumber 0123456789ABCDEF -UnderReset
.EXAMPLE
./Flash-Project.ps1 -WhatIf
.EXAMPLE
./Flash-Project.ps1 -SkipEnterDfu
#>
[CmdletBinding(SupportsShouldProcess)]
param(
    [string]$Firmware = "$PSScriptRoot/build/stm32g431_usb_webserver.hex",
    [string]$ProgrammerPath,
    [ValidateSet('DFU', 'SWD')][string]$Mode = 'DFU',
    [ValidatePattern('^USB[1-9][0-9]*$')][string]$UsbPort = 'USB1',
    [uri]$BaseUri = 'http://192.168.7.1',
    [ValidateRange(1,120)][int]$DfuTimeoutSeconds = 30,
    [switch]$SkipEnterDfu,
    [string]$SerialNumber,
    [ValidateRange(5, 24000)][int]$FrequencyKHz = 1000,
    [switch]$UnderReset,
    [switch]$Build
)
$ErrorActionPreference = 'Stop'
if ($Mode -eq 'DFU' -and ($SerialNumber -or $UnderReset -or $PSBoundParameters.ContainsKey('FrequencyKHz'))) {
    throw '-SerialNumber, -UnderReset and -FrequencyKHz require -Mode SWD. For DFU select -UsbPort USB1 (or USB2, etc.).'
}
if ($Mode -eq 'SWD' -and $PSBoundParameters.ContainsKey('UsbPort')) { throw '-UsbPort requires -Mode DFU.' }
if ($Mode -eq 'SWD' -and ($SkipEnterDfu -or $PSBoundParameters.ContainsKey('BaseUri') -or $PSBoundParameters.ContainsKey('DfuTimeoutSeconds'))) {
    throw '-SkipEnterDfu, -BaseUri and -DfuTimeoutSeconds require -Mode DFU.'
}
if (!$BaseUri.IsAbsoluteUri -or $BaseUri.Scheme -notin 'http','https' -or $BaseUri.Query -or $BaseUri.Fragment) {
    throw '-BaseUri must be an absolute HTTP(S) URL without query or fragment.'
}
if ($Build) {
    if ($WhatIfPreference) { throw 'Run Build-Project.ps1 first, then use Flash-Project.ps1 -WhatIf without -Build.' }
    & "$PSScriptRoot/Build-Project.ps1"
}
if (!(Test-Path -LiteralPath $Firmware -PathType Leaf)) {
    throw "Firmware not found: $Firmware. Run Build-Project.ps1 first."
}
$firmwarePath = (Resolve-Path -LiteralPath $Firmware).Path
$extension = [IO.Path]::GetExtension($firmwarePath).ToLowerInvariant()
if ($extension -notin '.hex', '.elf', '.bin') { throw 'Firmware must be a HEX, ELF or BIN file.' }
$arguments = if ($Mode -eq 'DFU') { @('-c', "port=$UsbPort") } else { @('-c', 'port=SWD', "freq=$FrequencyKHz") }
if ($Mode -eq 'SWD') {
    if ($SerialNumber) { $arguments += "sn=$SerialNumber" }
    if ($UnderReset) { $arguments += @('mode=UR', 'reset=HWrst') }
}
$arguments += @('-w', $firmwarePath)
if ($extension -eq '.bin') { $arguments += '0x08000000' }
$arguments += '-v'
# CubeProgrammer's -rst is a debug-port operation, not a USB DFU reset.
if ($Mode -eq 'SWD') { $arguments += '-rst' }
$target = if ($Mode -eq 'DFU') { "USB DFU device on $UsbPort" } elseif ($SerialNumber) { "ST-LINK $SerialNumber over SWD" } else { 'the connected ST-LINK target over SWD' }
$action = if ($Mode -eq 'DFU' -and !$SkipEnterDfu) { "Request DFU at $BaseUri, wait for $UsbPort, write '$firmwarePath' and verify flash" } else { "Write '$firmwarePath' and verify flash" }
if (!$PSCmdlet.ShouldProcess($target, $action)) { return }

if ($ProgrammerPath) {
    if (!(Test-Path -LiteralPath $ProgrammerPath -PathType Leaf)) { throw "Programmer not found: $ProgrammerPath" }
    $programmer = (Resolve-Path -LiteralPath $ProgrammerPath).Path
} else {
    $command = Get-Command STM32_Programmer_CLI, STM32_Programmer_CLI.exe -ErrorAction SilentlyContinue | Select-Object -First 1
    $programmer = if ($command) { $command.Source } else { $null }
    if (!$programmer) {
        foreach ($installationRoot in $env:ProgramW6432, $env:ProgramFiles, ${env:ProgramFiles(x86)}) {
            if (!$installationRoot) { continue }
            $candidate = Join-Path $installationRoot 'STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe'
            if (Test-Path -LiteralPath $candidate -PathType Leaf) { $programmer = $candidate; break }
        }
    }
    if (!$programmer) {
        throw 'Install STM32CubeProgrammer, add its bin folder to PATH, or provide -ProgrammerPath to STM32_Programmer_CLI.exe.'
    }
}
if ($Mode -eq 'DFU') {
    # Enumeration is read-only. Never use an unrelated pre-existing DFU device
    # as evidence that the API-requested device has switched successfully.
    function Get-DfuPorts {
        $PSNativeCommandUseErrorActionPreference = $false
        $listing = & $programmer -l usb 2>&1 | Out-String
        if ($LASTEXITCODE) { return }
        [regex]::Matches($listing, '(?im)^\s*(?:USB Port|Device Index)\s*:\s*(USB[1-9][0-9]*)\s*$') |
            ForEach-Object { $_.Groups[1].Value.ToUpperInvariant() } | Sort-Object -Unique
    }
    if (!$SkipEnterDfu) {
        if (@(Get-DfuPorts).Count) { throw 'A USB DFU device is already connected. For an intentionally selected device use -SkipEnterDfu -UsbPort USB1 (or USB2, etc.).' }
        $endpoint = $BaseUri.AbsoluteUri.TrimEnd('/') + '/api/system/dfu'
        try {
            $accepted = Invoke-WebRequest -Uri $endpoint -Method Post -Body '' -ContentType 'application/octet-stream' -NoProxy -TimeoutSec 8
            $payload = $accepted.Content | ConvertFrom-Json
            if ($accepted.StatusCode -ne 202 -or $payload.mode -ne 'dfu' -or $payload.status -ne 'rebooting') {
                throw 'Expected HTTP 202 with mode=dfu and status=rebooting.'
            }
        } catch {
            throw "DFU request failed; no firmware was written. Install the API-capable firmware once using manual BOOT0/reset and -SkipEnterDfu. Details: $($_.Exception.Message)"
        }
        Write-Host 'DFU request accepted; waiting for USB re-enumeration.'
    }
    $deadline = [Diagnostics.Stopwatch]::StartNew()
    while ($true) {
        $ports = @(Get-DfuPorts)
        if ($ports.Count -gt 1 -and !$PSBoundParameters.ContainsKey('UsbPort')) {
            throw 'Multiple DFU devices detected. Select the intended device explicitly with -UsbPort.'
        }
        if ($ports -contains $UsbPort) { break }
        if ($deadline.Elapsed.TotalSeconds -ge $DfuTimeoutSeconds) {
            throw "Timed out waiting for DFU on $UsbPort; no firmware was written. Check the native USB cable, CubeProgrammer DFU driver, bootloader entry and -UsbPort selection."
        }
        Start-Sleep -Milliseconds 500
    }
    Write-Host "Flashing $firmwarePath via USB DFU on $UsbPort"
} else {
    Write-Host "Flashing $firmwarePath via SWD at $FrequencyKHz kHz"
}
& $programmer @arguments
if ($LASTEXITCODE) {
    $hint = if ($Mode -eq 'DFU') { 'Check BOOT0/reset, the native USB data connection, CubeProgrammer DFU driver and -UsbPort selection. The running webserver USB interface is not DFU.' } else { 'Check board power, SWD wiring and probe selection. Use -UnderReset with NRST connected if needed.' }
    throw "STM32CubeProgrammer failed (exit $LASTEXITCODE). $hint"
}
if ($Mode -eq 'DFU') {
    Write-Host 'Flash verified. Return BOOT0 low and reset or power-cycle the board to run the firmware.'
} else {
    Write-Host 'Flash verified; board reset.'
}
Write-Host 'Open http://192.168.7.1/ after USB-NCM connects.'
