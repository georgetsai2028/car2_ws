[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][ValidateSet('master','throttle','steering','brake')][string]$Role,
    [Alias('Image')][ValidateSet('operational','sensor','throttle-usb')][string]$Firmware = 'operational',
    [string]$Serial = '',
    [switch]$ActuatorPowerOff,
    [switch]$ValidateOnly
)
$ErrorActionPreference = 'Stop'
$requestedRole = $Role.ToLowerInvariant()
$requestedImage = $Firmware
$requestedSerial = $Serial
$powerOffConfirmed = [bool]$ActuatorPowerOff
$validationOnly = [bool]$ValidateOnly
$bundleRoot = Split-Path -Parent $PSScriptRoot
& (Join-Path $PSScriptRoot 'Verify-Package.ps1')
# Import existing identity-aware enumeration and wait functions without opening USB.
. (Join-Path $bundleRoot 'outputs/car2-four-node-can-test/Flash-CAN-Test.ps1') -ValidateOnly
$package = Get-Content -LiteralPath (Join-Path $bundleRoot 'manifest.json') -Raw | ConvertFrom-Json
$selectionKey = if ($requestedImage -eq 'operational') { $requestedRole } else { $requestedImage }
$imageEntry = $package.images.$selectionKey
if (-not $imageEntry -or $imageEntry.role -ne $requestedRole) { throw 'Image does not match this node role' }
$selectedImage = Join-Path $bundleRoot $imageEntry.path
if ((Get-FileHash -LiteralPath $selectedImage -Algorithm SHA256).Hash -ne $imageEntry.sha256) { throw 'Image hash mismatch' }
if (-not $requestedSerial) { $requestedSerial = [string]$package.boards.$requestedRole.serial }
if ($requestedSerial -notmatch '^\d+$') { throw 'Expected a numeric application USB serial' }
if ($validationOnly) { Write-Host "PASS: $requestedRole image and tools verified; target serial $requestedSerial. No USB opened."; return }
if (-not $powerOffConfirmed) { throw 'Switch actuator power OFF and pass -ActuatorPowerOff before flashing' }
$connected = @(Get-TeensyPorts)
$boot = @(Get-Bootloaders)
$target = $null
if ($connected.Count -eq 1 -and [string]$connected[0].serial -eq $requestedSerial -and $boot.Count -eq 0) {
    & $python $usbHelper reboot --port $connected[0].port --serial $requestedSerial
    if ($LASTEXITCODE -ne 0) { Write-Host 'Checking for bootloader after reboot request...' }
    $target = Wait-Bootloader $requestedSerial 12
} elseif ($connected.Count -eq 0 -and $boot.Count -eq 1 -and (Get-ApplicationSerial $boot[0].InstanceId) -eq $requestedSerial) {
    $target = $boot[0]
} else {
    throw "Connect ONLY the physical $requestedRole board, expected serial $requestedSerial. No image written."
}
if (-not $target) { throw 'Bootloader absent. Keep actuator power off; press/release Program once, then rerun.' }
$nowBoot = @(Get-Bootloaders)
if ($nowBoot.Count -ne 1 -or $nowBoot[0].InstanceId -ne $target.InstanceId -or (Get-ApplicationSerial $nowBoot[0].InstanceId) -ne $requestedSerial) { throw 'Bootloader identity changed' }
if ((Get-FileHash -LiteralPath $selectedImage -Algorithm SHA256).Hash -ne $imageEntry.sha256) { throw 'Image changed before upload' }
$runtime = Join-Path $bundleRoot 'runtime/installations'
New-Item -ItemType Directory -Path $runtime -Force | Out-Null
$stamp = [DateTime]::UtcNow.ToString('yyyyMMdd-HHmmssZ')
& $loader '--mcu=TEENSY41' '-v' $selectedImage 2>&1 | Tee-Object -FilePath (Join-Path $runtime "$requestedRole-$stamp.log")
if ($LASTEXITCODE -ne 0) { throw 'Flash failed. Stop and inspect the selected board.' }
$application = Wait-Application $requestedSerial 15
[ordered]@{
    uploadedUtc = [DateTime]::UtcNow.ToString('o'); role = $requestedRole
    serial = $requestedSerial; port = $application.port; image = $imageEntry.path
    sha256 = $imageEntry.sha256; actuatorPowerOffConfirmed = $powerOffConfirmed
    usbApplicationReturned = $true; flashReadbackVerified = $false; motionSent = $false
} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $runtime "$requestedRole-$stamp.json") -Encoding UTF8
Write-Host "UPLOADED $requestedRole; USB returned on $($application.port). No movement commanded. Verify runtime telemetry before powered testing."
