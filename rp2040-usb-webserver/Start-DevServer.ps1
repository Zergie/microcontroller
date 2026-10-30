#Requires -Version 7.0
[CmdletBinding()]
param(
    [ValidateRange(1, 65535)][int]$Port = 5173,
    [string]$Target = 'http://192.168.7.1',
    [switch]$Background
)
$ErrorActionPreference = 'Stop'
if (!(Get-Command node -ErrorAction SilentlyContinue)) { throw 'Install Node.js 22 or later.' }
$major = [int]((& node --version) -replace '^v(\d+).*', '$1')
if ($major -lt 22) { throw 'Node.js 22 or later is required.' }
if ($Background) {
    $build = Join-Path $PSScriptRoot 'build'
    $recordPath = Join-Path $build 'dev-server-process.json'
    New-Item -ItemType Directory -Force $build > $null
    if (Test-Path -LiteralPath $recordPath) {
        $record = Get-Content -LiteralPath $recordPath -Raw | ConvertFrom-Json
        $existing = Get-Process -Id $record.id -ErrorAction SilentlyContinue
        if ($existing -and $existing.ProcessName -eq 'node' -and
            $existing.StartTime.ToUniversalTime() -eq ([DateTime]$record.startTime).ToUniversalTime()) {
            if ($record.port -eq $Port -and $record.target -eq $Target) {
                Write-Host "Development server already running: http://127.0.0.1:$Port/"
                return
            }
            throw 'A development server is already running. Run ./Stop-DevServer.ps1 first.'
        }
    }
    $serverScript = Join-Path $PSScriptRoot 'tools/dev-server.mjs'
    $stdout = Join-Path $build 'dev-server.log'
    $stderr = Join-Path $build 'dev-server-error.log'
    $process = Start-Process -FilePath (Get-Command node).Source -WindowStyle Hidden -PassThru `
        -ArgumentList @('"' + $serverScript + '"', '--port', $Port, '--target', '"' + $Target + '"') `
        -RedirectStandardOutput $stdout -RedirectStandardError $stderr
    @{ id = $process.Id; startTime = $process.StartTime.ToUniversalTime().ToString('o'); port = $Port; target = $Target } |
        ConvertTo-Json | Set-Content -LiteralPath $recordPath
    for ($attempt = 0; $attempt -lt 30; $attempt++) {
        $process.Refresh()
        if ($process.HasExited) { throw "Development server failed to start: $(Get-Content -LiteralPath $stderr -Raw)" }
        try {
            Invoke-WebRequest "http://127.0.0.1:$Port/__dev/client.js" -NoProxy -TimeoutSec 1 > $null
            Write-Host "Development server started: http://127.0.0.1:$Port/ (PID $($process.Id))"
            return
        } catch { Start-Sleep -Milliseconds 100 }
    }
    throw "Development server is not responding. Logs: $stderr"
}
& node (Join-Path $PSScriptRoot 'tools/dev-server.mjs') --port $Port --target $Target
if ($LASTEXITCODE -ne 0) { throw 'Development server exited: see the error above.' }
