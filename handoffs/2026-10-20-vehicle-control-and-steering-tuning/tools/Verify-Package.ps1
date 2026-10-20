[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$bundle = Split-Path -Parent $PSScriptRoot
$manifest = Get-Content -LiteralPath (Join-Path $bundle 'manifest.json') -Raw | ConvertFrom-Json
$count = 0
foreach ($entry in $manifest.files) {
    $path = Join-Path $bundle $entry.path
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Missing: $($entry.path)" }
    if ((Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash -ne $entry.sha256) {
        throw "Hash mismatch: $($entry.path)"
    }
    $count++
}
Write-Host "PASS: $count packaged file hashes match. No USB device was accessed."
