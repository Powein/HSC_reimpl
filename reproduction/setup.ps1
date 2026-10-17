param([string]$Python = '')
$ErrorActionPreference = 'Stop'
if (-not $Python) {
    $bundledPython = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
    if (Test-Path -LiteralPath $bundledPython) { $Python = $bundledPython }
    else { $Python = (Get-Command python -ErrorAction Stop).Source }
}
& $Python -m pip install --disable-pip-version-check --no-warn-script-location --target (Join-Path $PSScriptRoot '_deps') -r (Join-Path $PSScriptRoot 'requirements-lock.txt')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
