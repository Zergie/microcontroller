#requires -Version 7.0
# Offline orchestration test. No USB, network or real programmer is used.
$ErrorActionPreference='Stop'
$projectRoot=Split-Path $PSScriptRoot
$fixture=Join-Path $projectRoot "build/flash-script-test-$([guid]::NewGuid().ToString('N'))"
New-Item -ItemType Directory -Path $fixture -Force | Out-Null
$previousFixture=$env:STM32_FLASH_TEST_STATE
$env:STM32_FLASH_TEST_STATE=Join-Path $fixture 'state.json'
$fakeProgrammer=Join-Path $fixture 'fake programmer.ps1'
$fakeFirmware=Join-Path $fixture 'firmware with spaces.hex'
Set-Content -LiteralPath $fakeFirmware -Value ':00000001FF'
Set-Content -LiteralPath $fakeProgrammer -Value @'
$s=Get-Content -LiteralPath $env:STM32_FLASH_TEST_STATE -Raw | ConvertFrom-Json
$global:LASTEXITCODE=0
if($args -contains '-l') {
    $s.Lists++
    $show=$s.Scenario -in 'manual','already' -or ($s.Scenario -notin 'timeout','old','invalid' -and $s.ApiCalls -gt 0 -and $s.Lists -ge 3)
    if($show) { "    USB Port : $($s.Port)" }
} else {
    $s.Writes++
    $s.Arguments=@($args)
    if($s.Scenario -eq 'failure') {$global:LASTEXITCODE=5}
}
$s | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $env:STM32_FLASH_TEST_STATE
'@
function Invoke-WebRequest {
    param($Uri,$Method,$Body,$ContentType,[switch]$NoProxy,$TimeoutSec)
    if($Uri -ne 'http://192.168.7.1/api/system/dfu' -or $Method -ne 'Post' -or $Body -ne '' -or !$NoProxy) {throw 'Unexpected API request'}
    $s=Get-Content -LiteralPath $env:STM32_FLASH_TEST_STATE -Raw | ConvertFrom-Json
    $s.ApiCalls++
    $s | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $env:STM32_FLASH_TEST_STATE
    if($s.Scenario -eq 'old') {throw 'HTTP 404'}
    if($s.Scenario -eq 'invalid') {return [pscustomobject]@{StatusCode=200;Content='{}'}}
    [pscustomobject]@{StatusCode=202;Content='{"mode":"dfu","status":"rebooting","delay_ms":1500}'}
}
function Check($condition,$message) {if(!$condition) {throw $message}}
function Run-Case($scenario,$options=@{},[switch]$ShouldFail) {
    @{Scenario=$scenario;Port='USB1';Lists=0;ApiCalls=0;Writes=0;Arguments=@()} |
        ConvertTo-Json | Set-Content -LiteralPath $env:STM32_FLASH_TEST_STATE
    $failed=$false
    try { & "$projectRoot/Flash-Project.ps1" -Firmware $fakeFirmware -ProgrammerPath $fakeProgrammer -DfuTimeoutSeconds 1 @options }
    catch {$failed=$true; if(!$ShouldFail) {throw}}
    Check ($failed -eq $ShouldFail.IsPresent) "Unexpected result for $scenario"
    Get-Content -LiteralPath $env:STM32_FLASH_TEST_STATE -Raw | ConvertFrom-Json
}
try {
    $s=Run-Case auto
    Check ($s.ApiCalls -eq 1 -and $s.Lists -ge 3 -and $s.Writes -eq 1) 'Automatic DFU ordering failed'
    Check (($s.Arguments -contains 'port=USB1') -and ($s.Arguments -contains $fakeFirmware) -and ($s.Arguments -contains '-v') -and ($s.Arguments -notcontains '-rst')) 'Wrong DFU arguments'
    $s=Run-Case manual @{SkipEnterDfu=$true}
    Check ($s.ApiCalls -eq 0 -and $s.Writes -eq 1) 'Manual DFU failed'
    foreach($scenario in 'old','invalid','timeout','already') {
        $s=Run-Case $scenario @{} -ShouldFail
        Check ($s.Writes -eq 0) "Unsafe write after $scenario"
    }
    $s=Run-Case failure @{} -ShouldFail
    Check ($s.Writes -eq 1) 'Programmer failure not tested'
    $s=Run-Case auto @{WhatIf=$true}
    Check ($s.ApiCalls -eq 0 -and $s.Lists -eq 0 -and $s.Writes -eq 0) 'WhatIf had side effects'
    Write-Host 'PASS: auto/manual DFU, delayed enumeration, API errors, timeout, existing device, verification failure, WhatIf, paths with spaces'
} finally {
    $env:STM32_FLASH_TEST_STATE=$previousFixture
    # Only delete the exact unique fixture directory created above.
    $resolved=[IO.Path]::GetFullPath($fixture)
    $expected=[IO.Path]::GetFullPath((Join-Path $projectRoot 'build'))+[IO.Path]::DirectorySeparatorChar
    if(!$resolved.StartsWith($expected,[StringComparison]::OrdinalIgnoreCase)) {throw 'Fixture escaped build directory'}
    Remove-Item -LiteralPath $resolved -Recurse -Force
}
