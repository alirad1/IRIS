# Copy the code to the Pi into ~/IRIS. Usage (from the repo root):
#   powershell -ExecutionPolicy Bypass -File scripts\deploy.ps1
#   powershell -ExecutionPolicy Bypass -File scripts\deploy.ps1 -WithModel
# Uses the `iris-pi` alias from ~/.ssh/config. Does not start anything on the Pi.
# Refuses to run if the Pi's config/iris.json differs from this one (for example
# after calibrating on the Pi): run scripts\pull_config.ps1 first, or pass -Force.
# -KeepPiConfig updates code only and leaves the Pi's config/iris.json as it is.
param(
    [string]$PiHost = "iris-pi",
    [switch]$WithModel,
    [switch]$Force,
    [switch]$KeepPiConfig
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$tgz = Join-Path $env:TEMP "iris-deploy.tgz"

if (-not $Force -and -not $KeepPiConfig) {
    $local = (Get-FileHash (Join-Path $root "config\iris.json") -Algorithm SHA256).Hash.ToLower()
    $remote = (ssh -n $PiHost "sha256sum ~/IRIS/config/iris.json 2>/dev/null | cut -d' ' -f1")
    if ($LASTEXITCODE -ne 0) { throw "could not reach $PiHost" }
    if ($remote -and ($remote.Trim() -ne $local)) {
        throw ("The Pi's config/iris.json differs from this one (calibrated on the Pi?). " +
               "Run scripts\pull_config.ps1 first, or -Force to overwrite the Pi's copy.")
    }
}

$items = @("iris", "tests", "README.md")
$wipe = "iris tests"
# config/ is never wiped: only iris.json is overwritten, so the calibration
# session file and backups next to it survive.
if (-not $KeepPiConfig) { $items += "config/iris.json" }
if ($WithModel) { $items += "models" }

Push-Location $root
try {
    # Build a file first: PowerShell 5.1 corrupts binary data piped between programs.
    & "$env:SystemRoot\System32\tar.exe" --format ustar --exclude "__pycache__" --exclude "*.pyc" -czf $tgz @items
    if ($LASTEXITCODE -ne 0) { throw "tar failed" }

    scp -q $tgz "${PiHost}:iris-deploy.tgz"
    if ($LASTEXITCODE -ne 0) { throw "scp failed" }

    $remote = "mkdir -p ~/IRIS && cd ~/IRIS && rm -rf $wipe && " +
              "tar -xzmf ~/iris-deploy.tgz -C ~/IRIS && rm ~/iris-deploy.tgz && echo deployed to ~/IRIS"
    ssh $PiHost $remote
    if ($LASTEXITCODE -ne 0) { throw "remote extract failed" }
}
finally {
    Pop-Location
    Remove-Item $tgz -ErrorAction SilentlyContinue
}
