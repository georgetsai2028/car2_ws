[CmdletBinding()]
param([string]$Python = 'py', [switch]$Build)
$ErrorActionPreference = 'Stop'
$bundle = Split-Path -Parent $PSScriptRoot
$venvPython = Join-Path $bundle '.venv/Scripts/python.exe'
if (-not (Test-Path -LiteralPath $venvPython)) {
    if ($Python -eq 'py') { & $Python -3 -m venv (Join-Path $bundle '.venv') }
    else { & $Python -m venv (Join-Path $bundle '.venv') }
    if ($LASTEXITCODE -ne 0) { throw 'Python 3 venv creation failed' }
}
& $venvPython -m pip install -r (Join-Path $bundle 'requirements.txt')
if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed' }
if ($Build) {
    & $venvPython -m platformio run -d (Join-Path $bundle 'car2_ws/firmware/teensys')
    if ($LASTEXITCODE -ne 0) { throw 'Firmware build failed' }
}
Write-Host 'Setup complete. No USB device was accessed. Use -Build to install the Teensy build/upload tools.'
