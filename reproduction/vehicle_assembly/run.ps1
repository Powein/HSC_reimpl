param(
    [string]$Python = '',
    [ValidateSet('bead','suspension','vehicle')][string]$Case = 'suspension',
    [ValidateSet('assembly','paper_stiffness')][string]$Profile = 'assembly',
    [ValidateRange(1,100000)][int]$Steps = 120,
    [ValidateRange(0,12)][int]$Obstacles = 0,
    [double]$MotorSpeed = 0.8,
    [double]$StartX = -3.0,
    [ValidateRange(1,86400)][int]$TimeoutSeconds = 300,
    [string]$Label = ''
)
$ErrorActionPreference = 'Stop'
if (-not $Python) {
    $bundledPython = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
    if (Test-Path -LiteralPath $bundledPython) { $Python = $bundledPython }
    else { $Python = (Get-Command python -ErrorAction Stop).Source }
}
if (-not $Label) { $Label = $Case + '_' + (Get-Date -Format 'yyyyMMdd_HHmmss') }
if ($Label -notmatch '^[A-Za-z0-9_-]+$') { throw 'Label must contain only letters, digits, _ or -.' }
$invariant = [System.Globalization.CultureInfo]::InvariantCulture
& $Python (Join-Path $PSScriptRoot 'run_job.py') --name $Label --timeout $TimeoutSeconds validate_assembly.py --case $Case --steps $Steps --obstacles $Obstacles --profile $Profile --motor-speed $MotorSpeed.ToString($invariant) --start-x $StartX.ToString($invariant) --label $Label
exit $LASTEXITCODE
