[CmdletBinding()]
param([switch]$RestoreOperational, [switch]$InstallOperationalLimits, [switch]$InstallSensorTest, [switch]$ActuatorPowerOff)
$ErrorActionPreference = 'Stop'
if (-not $ActuatorPowerOff) { throw 'Confirm actuator power OFF with -ActuatorPowerOff before flashing' }
$testDirectory = $PSScriptRoot
$outputDirectory = Split-Path -Parent $testDirectory
if (($RestoreOperational -and $InstallOperationalLimits) -or
    ($InstallSensorTest -and ($RestoreOperational -or $InstallOperationalLimits))) { throw 'Choose only one image' }
. (Join-Path $outputDirectory 'car2-four-node-can-test/Flash-CAN-Test.ps1') -ValidateOnly
$image = Join-Path $testDirectory 'steer-limit-test-v3.hex'
$expectedHash = 'e48890ca035bb5a2defb53d94a8bad020afe267cdf43c64dfba2e08d01b8d623'
if ($RestoreOperational) {
    $image = Join-Path $testDirectory 'steering-operational-limits-r01.hex'
    $expectedHash = '5250a929e681813abb43ed1f825f00ee735a24a9553de68cd74ab8dd7b4e22f2'
}
if ($InstallOperationalLimits) {
    $image = Join-Path $testDirectory 'steering-operational-limits-r01.hex'
    $expectedHash = '5250a929e681813abb43ed1f825f00ee735a24a9553de68cd74ab8dd7b4e22f2'
}
if ($InstallSensorTest) {
    $image = Join-Path $testDirectory 'steer-sensor-test-v1.hex'
    $expectedHash = 'f7ae2a32d04af728e64a8cf594d28eac031bead05a30fac433a5f6b6682d6d7d'
}
if ((Get-FileHash -LiteralPath $image -Algorithm SHA256).Hash -ne $expectedHash) { throw 'Steering image hash mismatch' }
$connected = @(Get-TeensyPorts)
$initialBoot = @(Get-Bootloaders)
if ($connected.Count -eq 1 -and $connected[0].serial -eq '20369970' -and $initialBoot.Count -eq 0) {
    $selectedPort = $connected[0].port
    & $python $usbHelper reboot --port $selectedPort --serial 20369970
    if ($LASTEXITCODE -ne 0) { throw 'Targeted reboot failed' }
    $target = Wait-Bootloader '20369970' 12
} elseif ($connected.Count -eq 0 -and $initialBoot.Count -eq 1 -and
          (Get-ApplicationSerial $initialBoot[0].InstanceId) -eq '20369970') {
    # A manual Program-button press bypasses the application's USB implementation.
    $target = $initialBoot[0]
} else {
    throw 'Expected only steering USB serial 20369970, in application or bootloader mode'
}
if (-not $target) { throw 'Steering bootloader did not appear' }
$boot = @(Get-Bootloaders)
if ($boot.Count -ne 1 -or $boot[0].InstanceId -ne $target.InstanceId -or
    (Get-ApplicationSerial $boot[0].InstanceId) -ne '20369970') { throw 'Steering bootloader identity changed' }
if ((Get-FileHash -LiteralPath $image -Algorithm SHA256).Hash -ne $expectedHash) { throw 'Image changed before upload' }
$runtimeDirectory = Join-Path (Split-Path -Parent $outputDirectory) 'runtime/steering'
New-Item -ItemType Directory -Path $runtimeDirectory -Force | Out-Null
$stamp = [DateTime]::UtcNow.ToString('yyyyMMdd-HHmmssZ')
& $loader '--mcu=TEENSY41' '-v' $image 2>&1 | Tee-Object -FilePath (Join-Path $runtimeDirectory "flash-$stamp.log")
if ($LASTEXITCODE -ne 0) { throw 'Flash failed; stop and inspect target before retrying' }
$application = Wait-Application '20369970' 15
[ordered]@{
    uploadedUtc = [DateTime]::UtcNow.ToString('o')
    serial = '20369970'
    port = $application.port
    image = $image
    sha256 = $expectedHash
    operational = [bool]($RestoreOperational -or $InstallOperationalLimits)
    motionSent = $false
} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $runtimeDirectory "installation-$stamp.json") -Encoding UTF8
Write-Output "UPLOADED steering serial20369970 port=$($application.port) sha256=$expectedHash"
