$ErrorActionPreference = 'Stop'
$dependencies = @(
    @('tinyusb', 'https://github.com/hathach/tinyusb.git', '72649b6234968813ca3e53420c7ac4526b6a6eb9'),
    @('lwip', 'https://github.com/lwip-tcpip/lwip.git', '77dcd25a72509eb83f72b033d219b1d40cd8eb95'),
    @('cmsis-device-g4', 'https://github.com/STMicroelectronics/cmsis-device-g4.git', '626ee412334a5ed2e5b320af5a8d77d69f03a558'),
    @('cmsis', 'https://github.com/ARM-software/CMSIS_5.git', '2b7495b8535bdcb306dac29b9ded4cfb679d7e5c')
)
New-Item -ItemType Directory -Force "$PSScriptRoot/deps" | Out-Null
foreach ($dependency in $dependencies) {
    $name, $url, $revision = $dependency
    $destination = Join-Path "$PSScriptRoot/deps" $name
    if (!(Test-Path "$destination/.git")) {
        git init $destination
        if ($LASTEXITCODE) { throw "Cannot initialize $name" }
        git -C $destination remote add origin $url
        if ($LASTEXITCODE) { throw "Cannot configure $name" }
    }
    $head = git -C $destination rev-parse --verify --quiet HEAD
    if ($head -eq $revision) { continue }
    if (git -C $destination status --porcelain) { throw "Local changes in $destination; refusing checkout" }
    git -C $destination fetch --depth 1 origin $revision
    if ($LASTEXITCODE) { throw "Cannot fetch $name" }
    git -C $destination checkout --detach $revision
    if ($LASTEXITCODE) { throw "Cannot checkout $name" }
}
