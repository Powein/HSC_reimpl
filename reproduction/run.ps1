param([string]$Python = '', [switch]$SkipGpu, [int]$TimeoutSeconds = 600)
$ErrorActionPreference = 'Stop'
if (-not $Python) {
    $bundledPython = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
    if (Test-Path -LiteralPath $bundledPython) { $Python = $bundledPython }
    else { $Python = (Get-Command python -ErrorAction Stop).Source }
}
$scriptFile = Join-Path $PSScriptRoot 'run.py'
$resultDir = Join-Path $PSScriptRoot 'results'
New-Item -ItemType Directory -Path $resultDir -Force | Out-Null
$arguments = @('-X', 'utf8', ('"' + $scriptFile + '"'))
if ($SkipGpu) { $arguments += '--skip-gpu' }
$startUtc = [DateTime]::UtcNow.ToString('o')
$process = Start-Process -FilePath $Python -ArgumentList $arguments -WorkingDirectory $PSScriptRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $resultDir 'run.log') -RedirectStandardError (Join-Path $resultDir 'stderr.log')
Write-Output "Running PID $($process.Id); log: $resultDir\run.log"
if (-not $process.WaitForExit($TimeoutSeconds * 1000)) {
    Write-Warning "The explicit $TimeoutSeconds second hard timeout was reached; terminating this experiment process."
    Stop-Process -Id $process.Id
    exit 124
}
$process.WaitForExit()
$executionRecord = [ordered]@{
    command = "$Python $($arguments -join ' ')"
    working_directory = $PSScriptRoot
    monitored_pid = $process.Id
    hard_timeout_seconds = $TimeoutSeconds
    observed_exit_code = $process.ExitCode
    started_utc = $startUtc
    completed_utc = [DateTime]::UtcNow.ToString('o')
    stdout = 'run.log'
    stderr = 'stderr.log'
}
$executionRecord | ConvertTo-Json | Out-File -LiteralPath (Join-Path $resultDir 'execution_monitoring.json') -Encoding utf8
Get-Content -LiteralPath (Join-Path $resultDir 'run.log') -Encoding utf8
if ($process.ExitCode -ne 0) {
    Get-Content -LiteralPath (Join-Path $resultDir 'stderr.log') -Encoding utf8
}
exit $process.ExitCode
