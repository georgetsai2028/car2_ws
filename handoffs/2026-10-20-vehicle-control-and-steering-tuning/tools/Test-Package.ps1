[CmdletBinding()]
param([string]$Python = '', [string]$Compiler = 'g++', [switch]$BuildFirmware)
$ErrorActionPreference = 'Stop'
$bundle = Split-Path -Parent $PSScriptRoot
if (-not $Python) { $Python = Join-Path $bundle '.venv/Scripts/python.exe' }
$compilerCommand = Get-Command $Compiler -ErrorAction Stop
& $Python -m unittest discover -s (Join-Path $bundle 'car2_ws/tests') -p 'test_*.py' -v
if ($LASTEXITCODE -ne 0) { throw 'Python regression failed' }
& $Python (Join-Path $bundle 'car2_ws/tests/run_firmware_tests.py') --compiler $compilerCommand.Source --build-dir (Join-Path $bundle '.test-build')
if ($LASTEXITCODE -ne 0) { throw 'Firmware mock regression failed' }
if ($BuildFirmware) {
    & $Python -m platformio run -d (Join-Path $bundle 'car2_ws/firmware/teensys')
    if ($LASTEXITCODE -ne 0) { throw 'Teensy build failed' }
}
Write-Host 'PASS: offline regression checks completed. No USB device was accessed.'
