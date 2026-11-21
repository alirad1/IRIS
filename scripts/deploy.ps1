# Copy the code to the Pi into ~/IRIS. Usage (from the repo root):
#   powershell -ExecutionPolicy Bypass -File scripts\deploy.ps1
#   powershell -ExecutionPolicy Bypass -File scripts\deploy.ps1 -WithModel
# Uses the `iris-pi` alias from ~/.ssh/config. Does not start anything on the Pi.
param(
    [string]$PiHost = "iris-pi",
    [switch]$WithModel
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$tgz = Join-Path $env:TEMP "iris-deploy.tgz"

$items = @("iris", "config", "tests", "README.md")
if ($WithModel) { $items += "models" }

Push-Location $root
try {
    # Build a file first: PowerShell 5.1 corrupts binary data piped between programs.
    tar --format ustar --exclude "__pycache__" --exclude "*.pyc" -czf $tgz @items
    if ($LASTEXITCODE -ne 0) { throw "tar failed" }

    scp -q $tgz "${PiHost}:iris-deploy.tgz"
    if ($LASTEXITCODE -ne 0) { throw "scp failed" }

    $remote = "mkdir -p ~/IRIS && cd ~/IRIS && rm -rf iris config tests && " +
              "tar -xzf ~/iris-deploy.tgz -C ~/IRIS && rm ~/iris-deploy.tgz && echo deployed to ~/IRIS"
    ssh $PiHost $remote
    if ($LASTEXITCODE -ne 0) { throw "remote extract failed" }
}
finally {
    Pop-Location
    Remove-Item $tgz -ErrorAction SilentlyContinue
}
