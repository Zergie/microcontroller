#requires -Version 7.0
param([string]$BaseUri='http://192.168.7.1')
$ErrorActionPreference='Stop'
$page=Invoke-WebRequest "$BaseUri/" -NoProxy -TimeoutSec 10
if($page.Content -notmatch 'STM32G431 USB Webserver') {throw 'STM32 GPIO UI not found; flash the current firmware.'}
foreach($asset in 'app.js','style.css','stm32-pins.js','gpio-model.js','pin-session.js','pin-store.js','gpio-transport.js') {
    $response=Invoke-WebRequest "$BaseUri/$asset" -NoProxy -TimeoutSec 10
    if($response.StatusCode -ne 200) {throw "Cannot load $asset"}
}
$state=Invoke-RestMethod "$BaseUri/api/gpio.json" -NoProxy -TimeoutSec 10
if($state.pins.Count -ne 41) {throw 'Expected 41 GPIO entries'}
$levels=Invoke-RestMethod "$BaseUri/api/gpio/levels.json" -NoProxy -TimeoutSec 10
if($levels.levels.Count -ne 41 -or @($levels.levels | Where-Object {$_ -notin 0,1}).Count) {throw 'Invalid live GPIO levels'}
foreach($pin in 11,12,13,14,24,35,36,37,38) {
    if($state.pins[$pin].available) {throw "Reserved pin $pin was exposed"}
}
Write-Host 'PASS: STM32 UI, all assets, GPIO snapshot and reserved pins. No GPIO was changed.'
