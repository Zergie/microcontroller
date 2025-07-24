[CmdletBinding()]
param (
    [Parameter()]
    [string]
    $port,

    [Parameter()]
    [switch]
    $DryRun
)

$ampy = "C:\Users\user\AppData\Roaming\Python\Python313\Scripts\ampy.exe"
if ($null -eq (Get-Command $ampy -ErrorAction SilentlyContinue)) {
    Write-Host -ForegroundColor Red "Please install ampy"
    exit 1
}
if (-not $port) {
    $ports = [System.IO.Ports.SerialPort]::GetPortNames()
    if ($ports.Count -eq 1) {
        $port = $ports[0]
        Write-Host "Using port $port"
    } else {
        Write-Host -ForegroundColor Red "Please specify a port or connect only one device to the computer. (available ports: $($ports | Join-String -Separator ", "))"
        exit 1
    }
}


Push-Location $PSScriptRoot
$last_upload = Get-Content -Path .\last_upload.txt -ErrorAction SilentlyContinue |
                ForEach-Object { [datetime]::ParseExact($_, "yyyy-MM-dd HH:mm:ss", $null) } |
                Select-Object -First 1

Write-Host "last upload: $last_upload"
$mcu_files = . $ampy -p $port ls -r
$local_files = Get-ChildItem -Recurse -File -Filter *.py |
                ForEach-Object { 
                    [PSCustomObject]@{
                        FullName = $_.FullName
                        LastWriteTime = $_.LastWriteTime
                        Destination = "/" + $_.FullName.Substring($PSScriptRoot.Length + 1).Replace("\", "/")
                    }
                }

$mcu_files |
    Where-Object { $_ -notin $local_files.Destination } |
    ForEach-Object {
        Write-Host "Deleting $_ from mcu"
        if (!$DryRun) {. $ampy -p $port rm $_}
} 
 
$local_files |
    Where-Object { $_.LastWriteTime -gt $last_upload } |
    ForEach-Object {
        Write-Host "Uploading $($_.Destination) to mcu (changed)"
        if (!$DryRun) {. $ampy -p $port put $_.FullName $_.Destination}
        $mcu_files += @($_)
    }
$local_files |
    Where-Object { $_.Destination -notin $mcu_files } |
    ForEach-Object {
        Write-Host "Uploading $($_.Destination) to mcu (new)"
        if (!$DryRun) {. $ampy -p $port put $_.FullName $_.Destination}
    }

Set-Content -Path .\last_upload.txt -Value (Get-Date).ToString("yyyy-MM-dd HH:mm:ss") -Force
Write-Host "Done!"
write-Host ""

Pop-Location
