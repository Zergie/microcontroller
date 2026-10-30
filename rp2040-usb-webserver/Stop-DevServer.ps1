#Requires -Version 7.0
[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$recordPath = Join-Path $PSScriptRoot 'build/dev-server-process.json'
if (!(Test-Path -LiteralPath $recordPath)) { Write-Host 'No background server registered.'; return }
$record = Get-Content -LiteralPath $recordPath -Raw | ConvertFrom-Json
$process = Get-Process -Id $record.id -ErrorAction SilentlyContinue
if ($process) {
    if ($process.ProcessName -ne 'node' -or $process.StartTime.ToUniversalTime() -ne ([DateTime]$record.startTime).ToUniversalTime()) {
        throw 'PID now belongs to another process; it will not be stopped.'
    }
    Stop-Process -Id $process.Id
}
Remove-Item -LiteralPath $recordPath
Write-Host 'Development server stopped.'
