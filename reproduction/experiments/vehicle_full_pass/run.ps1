param([string]$Case, [switch]$Resume)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$runtime = Get-Content -LiteralPath (Join-Path $projectRoot 'reproduction/hsc_backend/runtime.json') -Raw | ConvertFrom-Json
$arguments = @('-u', (Join-Path $PSScriptRoot 'run.py'))
if ($Case) { $arguments += @('--case', $Case) }
if ($Resume) { $arguments += '--resume' }
& $runtime.python @arguments
exit $LASTEXITCODE
