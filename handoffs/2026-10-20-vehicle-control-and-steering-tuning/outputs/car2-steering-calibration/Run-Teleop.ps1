[CmdletBinding()]
param([string]$Port = '', [switch]$DisableManualReference)
$ErrorActionPreference = 'Stop'
$workspaceDirectory = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$python = Join-Path $workspaceDirectory '.venv/Scripts/python.exe'
$portHelper = Join-Path $workspaceDirectory 'outputs/car2-four-node-can-test/can_test_usb.py'
$teleopScript = Join-Path $workspaceDirectory 'car2_ws/teleop/keyboard_teleop_wasd_guarded.py'
$portJson = & $python $portHelper list
if ($LASTEXITCODE -ne 0) { throw 'Could not enumerate Teensy USB ports' }
$masterPorts = @($portJson | ConvertFrom-Json | Where-Object {
    $_.serial -eq '18637540' -and (-not $Port -or $_.port -eq $Port)
})
if ($masterPorts.Count -ne 1) { throw 'Connect the master Teensy, USB serial 18637540' }
$arguments = @($teleopScript, '--port', $masterPorts[0].port)
if ($DisableManualReference) { $arguments += '--disable-manual-reference' }
Write-Output 'Starts in E-stop. Physically straight wheels required before Z. F2 toggles manual Z in E-stop.'
& $python @arguments
exit $LASTEXITCODE
