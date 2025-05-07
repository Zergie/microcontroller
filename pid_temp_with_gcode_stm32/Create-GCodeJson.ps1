[CmdletBinding()]
param (
)
$DefaultGcode = @"
[{"code":"G4","description":"Dwell","parameters":[{"name":"P","description":"Time to wait, in milliseconds (In Teacup, P0, wait until all previous moves are finished)"},{"name":"S","description":"Time to wait, in seconds (Only on Repetier, Marlin, Prusa, Smoothieware, and RepRapFirmware 1.16 and later)"}]},{"code":"M1","description":"Sleep or Conditional stop"},{"code":"M104","description":"Set Extruder Temperature","parameters":[{"name":"C","description":"Use fan for cooling (Buddy only)"},{"name":"D","description":"Display temperature (Buddy only)"},{"name":"S","description":"Target temperature"},{"name":"R","description":"Idle temperature (Only MK4duo)"}]},{"code":"M105","description":"Get Extruder Temperature"},{"code":"M108","description":"Cancel Heating"},{"code":"M109","description":"Set Extruder Temperature and Wait","parameters":[{"name":"C","description":"Use fan for cooling (Buddy only)"},{"name":"S","description":"minimum target temperature, waits until heating"},{"name":"R","description":"maximum target temperature, waits until cooling (Sprinter)"},{"name":"R","description":"accurate target temperature, waits until heating and cooling (Marlin and MK4duo)"},{"name":"T","description":"tool number (RepRapFirmware and Klipper), optional"}]},{"code":"M115","description":"Get Firmware Version and Capabilities","parameters":[{"name":"B","description":"(RepRapFirmware 3 only) Expansion board number (typically the CAN address) for which the firmware version is requested, default 0 (i.e. main board)"},{"name":"P","description":"Electronics type"},{"name":"V","description":"Report the Prusa version number"},{"name":"U","description":"Check the firmware version provided"}]},{"code":"M130","description":"Set PID P value","parameters":[{"name":"P","description":"heater number"},{"name":"S","description":"proportional (Kp)"}]},{"code":"M131","description":"Set PID I value","parameters":[{"name":"P","description":"heater number"},{"name":"S","description":"integral (Ki)"}]},{"code":"M132","description":"Set PID D value","parameters":[{"name":"P","description":"heater number"},{"name":"S","description":"derivative (Kd)"}]},{"code":"M155","description":"Automatically send temperatures","parameters":[{"name":"S","description":"enable sending temperatures = 1, disable = 0"},{"name":"S","description":"Interval in seconds between auto-reports. S0 to disable. (Marlin) Prusa has a Maximum: 255"},{"name":"C","description":"Activate auto-report function (bit mask). Default is temperature."}]},{"code":"M2","description":"Program End"},{"code":"M301","description":"Set PID parameters","parameters":[{"name":"H","description":"heater number (Smoothie uses 'S', Redeem uses 'E')"},{"name":"P","description":"proportional (Kp)"},{"name":"I","description":"integral (Ki)"},{"name":"D","description":"derivative (Kd)"}]},{"code":"M303","description":"Run PID tuning"},{"code":"M500","description":"Store parameters in non-volatile storage"},{"code":"M501","description":"Read parameters from EEPROM","parameters":[{"name":"S","description":"Enable auto-save (only RepRapFirmware)"}]},{"code":"M502","description":"Restore Default Settings"},{"code":"M503","description":"Report Current Settings"}]
"@ | ConvertFrom-Json -Depth 99

$Content = Get-Content $PSScriptRoot\main\main.ino -Encoding utf8
$Content |
    Select-String "void\s+processCommand(G|M)" -Context 0,9999 |
    ForEach-Object {
        $end = $_.Context.PostContext |
            Select-String "^  }" |
            Select-Object -First 1
        $c = $_.Matches.Groups[1].Value
        $_.Context.PostContext |
            Select-Object -First $end.LineNumber |
            Select-String "case (\d+):\s*//\s*(.+)" -Context 0,99 |
            ForEach-Object { 
                $gcode = "$c$($_.Matches.Groups[1])"
                $end = $_.Context.PostContext |
                            Select-String "break;" |
                            Select-Object -First 1
                [PSCustomObject]@{
                    code = $gcode
                    description = $_.Matches.Groups[2].Value
                    parameters = $_.Context.PostContext |
                        Select-Object -First $end.LineNumber |
                        Select-String "(HasWord|GetWordValue)\('(\w)'" |
                        Sort-Object LineNumber -Descending |
                        ForEach-Object {
                            $p_name = $_.Matches.Groups[2].Value
                            [PSCustomObject]@{
                                name = $p_name
                                description = @(
                                    $_.Line |
                                        Select-String "//\s*(.+)" |
                                        ForEach-Object { $_.Matches.Groups[1].Value }
                                    $DefaultGcode |
                                        Where-Object code -EQ $gcode |
                                        ForEach-Object parameters |
                                        Where-Object name -EQ $p_name |
                                        ForEach-Object description |
                                        Select-Object -First 1
                                ) |
                                    Select-Object -First 1
                            }
                        } |
                        Sort-Object name -Unique
                } 
            }
    } |
    Sort-Object code |
    ForEach-Object {
        $params = $_.parameters |
            Sort-Object name |
            ForEach-Object {
                "`n      {`"name`": `"$($_.name)`", `"description`": `"$($_.description)`"}"
            } | 
            Join-String -Separator ","
        @(
            "  { `"code`": `"$($_.code)`", `"description`": `"$($_.description)`","
            "    `"parameters`": [$params]"
        ) | 
            Out-String
     } |
     Join-String -Separator "  },`n" |
     ForEach-Object -Begin {"["} -Process {"$_  }"} -End {"]"} |
     Set-Content $PSScriptRoot\gcode.json -Encoding utf8