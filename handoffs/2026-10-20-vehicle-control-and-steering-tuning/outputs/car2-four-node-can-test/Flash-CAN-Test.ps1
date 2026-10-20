<#
.SYNOPSIS
Flash each vehicle-slot Teensy with its CAN-only diagnostic, then read master.
.EXAMPLE
  .\Flash-CAN-Test.ps1
.EXAMPLE
  .\Flash-CAN-Test.ps1 -Role Brake
.EXAMPLE
  .\Flash-CAN-Test.ps1 -MonitorOnly
.EXAMPLE
  .\Flash-CAN-Test.ps1 -ValidateOnly
#>
[CmdletBinding()]
param(
    [ValidateSet('All', 'Throttle', 'Steering', 'Brake', 'Master')]
    [string]$Role = 'All',
    [switch]$MonitorOnly,
    [switch]$ValidateOnly,
    [switch]$ActuatorPowerOff
)

$ErrorActionPreference = 'Stop'
$imageDirectory = $PSScriptRoot
$bundleDirectory = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$python = Join-Path $bundleDirectory '.venv/Scripts/python.exe'
$runtimeDirectory = Join-Path $bundleDirectory 'runtime/can'
New-Item -ItemType Directory -Path $runtimeDirectory -Force | Out-Null
$loader = Join-Path $env:USERPROFILE '.platformio\packages\tool-teensy\teensy_loader_cli.exe'
$usbHelper = Join-Path $imageDirectory 'can_test_usb.py'
$manifestPath = Join-Path $imageDirectory 'manifest.json'
$assignmentPath = Join-Path $runtimeDirectory 'usb-role-assignments.json'

function Get-TeensyPorts {
    $raw = & $python $usbHelper list
    if ($LASTEXITCODE -ne 0) { throw 'Could not enumerate Teensy USB serial ports.' }
    $raw | ConvertFrom-Json
}

function Get-Bootloaders {
    @(Get-PnpDevice -PresentOnly -ErrorAction Stop | Where-Object {
        $_.InstanceId -like 'USB\VID_16C0&PID_0478\*'
    })
}

function Get-ApplicationSerial([string]$BootInstance) {
    # Teensy 4 usb_desc.c appends a zero to MAC-derived serials below 10 million.
    $hex = ($BootInstance -split '\\')[-1]
    if ($hex -notmatch '^[0-9A-Fa-f]{1,8}$') {
        throw "Cannot identify Teensy bootloader serial: $BootInstance"
    }
    [long]$number = [Convert]::ToInt64($hex, 16)
    if ($number -lt 10000000) { $number = $number * 10 }
    [string]$number
}

function Wait-Bootloader([string]$ExpectedSerial, [int]$Seconds = 8) {
    $timer = [Diagnostics.Stopwatch]::StartNew()
    do {
        $found = @(Get-Bootloaders)
        if ($found.Count -gt 1) {
            throw 'More than one Teensy bootloader is connected. Disconnect extra USB cables.'
        }
        if ($found.Count -eq 1) {
            $identity = Get-ApplicationSerial $found[0].InstanceId
            if ($ExpectedSerial -and $identity -ne $ExpectedSerial) {
                throw "Wrong bootloader: expected $ExpectedSerial, found $identity. Nothing flashed."
            }
            return $found[0]
        }
        Start-Sleep -Milliseconds 300
    } while ($timer.Elapsed.TotalSeconds -lt $Seconds)
    return $null
}

function Wait-Application([string]$Identity, [int]$Seconds = 15) {
    $timer = [Diagnostics.Stopwatch]::StartNew()
    do {
        $found = @(Get-TeensyPorts | Where-Object { $_.serial -eq $Identity })
        if ($found.Count -eq 1) { return $found[0] }
        Start-Sleep -Milliseconds 400
    } while ($timer.Elapsed.TotalSeconds -lt $Seconds)
    throw "USB serial $Identity did not return after flashing. Reconnect it and retry that role."
}

foreach ($required in @($python, $loader, $usbHelper, $manifestPath)) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) { throw "Missing file: $required" }
}
$manifest = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
if ($manifest.test -ne 'CAN_TEST_V1' -or $manifest.board -ne 'Teensy 4.1') {
    throw 'Unexpected firmware manifest. No board will be accessed.'
}
foreach ($name in @('master', 'throttle', 'steering', 'brake')) {
    $image = $manifest.images.$name
    if (-not $image -or $image.file -ne ($name + '.hex')) { throw "Invalid $name manifest entry." }
    $imagePath = Join-Path $imageDirectory $image.file
    $actualHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $imagePath).Hash
    if ($actualHash -ne $image.sha256) { throw "Hash mismatch: $imagePath. No board will be accessed." }
}
if ($ValidateOnly) {
    Write-Host 'PASS: Python/helper/loader present; all four CAN-test image hashes match.' -ForegroundColor Green
    Write-Host 'No USB device was opened or flashed.'
    return
}

$assignments = @{}
if (Test-Path -LiteralPath $assignmentPath) {
    $saved = Get-Content -LiteralPath $assignmentPath -Raw | ConvertFrom-Json
    foreach ($property in $saved.PSObject.Properties) { $assignments[$property.Name] = $property.Value }
}

if (-not $MonitorOnly -and -not $ActuatorPowerOff) { throw 'Confirm actuator power OFF with -ActuatorPowerOff before flashing' }
if (-not $MonitorOnly) {
    Write-Host 'Temporary CAN test: this REPLACES the selected boards firmware.' -ForegroundColor Yellow
    Write-Host 'The images contain no actuator-control code. Keep actuator power OFF during uploads.'
    Write-Host 'Use one Teensy USB cable at a time. Close other serial monitors.'
    $roles = if ($Role -eq 'All') { @('throttle', 'steering', 'brake', 'master') } else { @($Role.ToLowerInvariant()) }
    $usedThisRun = @{}
    foreach ($name in $roles) {
        $label = $name.ToUpperInvariant()
        Write-Host "`n--- $label ---" -ForegroundColor Cyan
        $null = Read-Host "Connect USB ONLY to the physical $label slot, then press Enter"
        $connected = @(Get-TeensyPorts)
        $boot = @(Get-Bootloaders)
        if ($connected.Count -gt 1 -or $boot.Count -gt 1 -or ($connected.Count + $boot.Count) -gt 1) {
            throw 'Multiple Teensys are on USB. Keep only the requested slot connected and retry this role.'
        }
        $expectedSerial = $null
        $targetBoot = $null
        if ($connected.Count -eq 1) {
            $target = $connected[0]
            $expectedSerial = [string]$target.serial
            if (-not $expectedSerial) { throw 'Selected Teensy has no readable USB identity.' }
            if ($usedThisRun.ContainsKey($expectedSerial)) {
                throw "You already flashed this board as $($usedThisRun[$expectedSerial]). Move USB to $label."
            }
            Write-Host "Identified $($target.port), USB serial $expectedSerial. Requesting its bootloader."
            try {
                & $python $usbHelper reboot --port $target.port --serial $expectedSerial
                if ($LASTEXITCODE -ne 0) { throw 'Automatic reboot returned an error.' }
            } catch {
                Write-Host 'Automatic reboot did not complete cleanly; checking bootloader.' -ForegroundColor Yellow
            }
            $targetBoot = Wait-Bootloader $expectedSerial
        } elseif ($boot.Count -eq 1) {
            $targetBoot = $boot[0]
            $expectedSerial = Get-ApplicationSerial $targetBoot.InstanceId
        }
        if (-not $targetBoot) {
            $null = Read-Host "Press and release Program on the $label Teensy, then press Enter here"
            $targetBoot = Wait-Bootloader $expectedSerial 15
            if (-not $targetBoot) { throw "No $label bootloader detected. Check USB/data cable and retry this role." }
            $expectedSerial = Get-ApplicationSerial $targetBoot.InstanceId
        }
        if ($usedThisRun.ContainsKey($expectedSerial)) {
            throw "This USB serial was already assigned to $($usedThisRun[$expectedSerial]). No image written."
        }
        # Re-check the uniquely identified bootloader immediately before writing.
        $currentBoot = @(Get-Bootloaders)
        if ($currentBoot.Count -ne 1 -or $currentBoot[0].InstanceId -ne $targetBoot.InstanceId) {
            throw 'Bootloader identity changed. No image written.'
        }
        $imagePath = Join-Path $imageDirectory $manifest.images.$name.file
        if ((Get-FileHash -Algorithm SHA256 -LiteralPath $imagePath).Hash -ne $manifest.images.$name.sha256) {
            throw 'Firmware image changed after validation. No image written.'
        }
        Write-Host "Flashing $label CAN-only test to USB serial $expectedSerial..."
        & $loader '--mcu=TEENSY41' '-v' $imagePath
        if ($LASTEXITCODE -ne 0) { throw "Flash failed. Keep this board selected and retry with -Role $name." }
        $application = Wait-Application $expectedSerial
        $verificationRaw = & $python $usbHelper verify --port $application.port --serial $expectedSerial --role $label
        if ($LASTEXITCODE -ne 0) { throw "$label firmware output did not verify. Stop and inspect that board." }
        $verification = $verificationRaw | ConvertFrom-Json
        $usedThisRun[$expectedSerial] = $label
        $assignments[$name] = [ordered]@{
            serial = $expectedSerial
            portAtFlash = $application.port
            role = $label
            imageSha256 = $manifest.images.$name.sha256
            verifiedUtc = [DateTime]::UtcNow.ToString('o')
        }
        $assignments | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $assignmentPath -Encoding UTF8
        Write-Host "Verified $label on $($application.port), serial $expectedSerial." -ForegroundColor Green
        if ($name -ne 'master') { Write-Host 'You can move the USB cable to the next slot when prompted.' }
    }
    if ($Role -ne 'All' -and $Role -ne 'Master') {
        Write-Host "`n$name finished. Use the script without -Role to install all four, or -MonitorOnly after master is installed."
        return
    }
}

if (-not $assignments.ContainsKey('master')) {
    throw 'No verified master identity saved. Flash master first with -Role Master.'
}
Write-Host "`n--- MASTER CAN MONITOR ---" -ForegroundColor Cyan
Write-Host 'Keep all four boards in their vehicle slots, with CAN modules powered and H/L connected.'
Write-Host 'Power the nodes from the car for this test; keep actuator power off.'
Write-Host 'Ensure VIN-VUSB power isolation when master has external VIN and USB connected.'
$null = Read-Host 'Leave USB on MASTER, power the four CAN nodes, then press Enter to read results'
$master = Wait-Application ([string]$assignments['master'].serial)
$roleCheck = & $python $usbHelper verify --port $master.port --serial $master.serial --role MASTER
if ($LASTEXITCODE -ne 0) { throw 'Connected board did not verify as CAN-test master.' }
$logName = 'master-can-' + [DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss') + 'Z.log'
$logPath = Join-Path $runtimeDirectory $logName
Write-Host 'Expected: THROTTLE=OK, STEERING=OK, BRAKE=OK; reply counts increasing; TEC/REC/fault zero.'
Write-Host "Raw log: $logPath"
& $python $usbHelper monitor --port $master.port --serial $master.serial --log $logPath
if ($LASTEXITCODE -ne 0) { throw 'CAN monitoring stopped with an error. Reconnect master and use -MonitorOnly.' }
