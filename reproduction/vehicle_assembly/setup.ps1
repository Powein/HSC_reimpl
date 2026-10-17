param([string]$Python = '')
$ErrorActionPreference = 'Stop'
if (-not $Python) {
    $bundledPython = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
    if (Test-Path -LiteralPath $bundledPython) { $Python = $bundledPython }
    else { $Python = (Get-Command python -ErrorAction Stop).Source }
}
$reproRoot = Split-Path -Parent $PSScriptRoot
$researchRoot = Join-Path $reproRoot 'vehicle_research'
if (-not (Test-Path -LiteralPath (Join-Path $reproRoot '_deps\numpy'))) {
    & (Join-Path $reproRoot 'setup.ps1') -Python $Python
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
& $Python -m pip install --disable-pip-version-check --no-warn-script-location --no-deps --target (Join-Path $researchRoot '_geometry') -r (Join-Path $PSScriptRoot 'requirements-geometry.txt')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $Python -m pip install --disable-pip-version-check --no-warn-script-location --no-deps --target (Join-Path $researchRoot '_runtime') 'pyuipc==0.0.28'
exit $LASTEXITCODE
