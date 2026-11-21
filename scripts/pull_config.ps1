# Copy config/iris.json from the Pi (e.g. after calibrating there) into this repo.
#   powershell -ExecutionPolicy Bypass -File scripts\pull_config.ps1
param([string]$PiHost = "iris-pi")
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$dest = Join-Path $root "config\iris.json"
$tmp = Join-Path $env:TEMP "iris-pulled.json"

scp -q "${PiHost}:IRIS/config/iris.json" $tmp
if ($LASTEXITCODE -ne 0) { throw "scp failed" }

# Only accept it if it passes the same validation the Pi uses.
& (Join-Path $root ".venv\Scripts\python.exe") -c "import json,sys; from iris import config as c; c.from_dict(json.load(open(sys.argv[1]))); print('valid')" $tmp
if ($LASTEXITCODE -ne 0) { throw "pulled config is not valid; not copied" }

Copy-Item $tmp $dest -Force
Remove-Item $tmp
Write-Output "pulled into config\iris.json. Review with: git diff config/iris.json"
