# Windows-first task runner. No make.
#
# Usage:
#   .\tasks.ps1              # defaults to "test"
#   .\tasks.ps1 test         # python -m pytest -q
#   .\tasks.ps1 guarded      # the composition test: claim (A), all 15 attacks
#   .\tasks.ps1 timing       # the pre-registered timing run, writes results/
#   .\tasks.ps1 docker       # report whether URSim work can run here, and why not
#
# The interpreter path is repository-relative ($PSScriptRoot\.venv), never a
# path hardcoded to any one machine, so this runs the same way from any clone
# that has created .venv in the repository root per the README's quick start.

param([Parameter(Position = 0)][string]$Task = "test")

$ErrorActionPreference = "Stop"
$py = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"

switch ($Task) {
    "test"    { & $py -m pytest -q }
    "guarded" { & $py -m pytest tests\test_guarded.py -q }
    "timing"  { & $py -m keystone.timing }
    "docker"  { & $py -c "from keystone.ursim import docker_status; s = docker_status(); print(('available: ' if s.available else 'unavailable: ') + s.reason)" }
    default   { Write-Host "usage: .\tasks.ps1 [test|guarded|timing|docker]"; exit 2 }
}

exit $LASTEXITCODE
