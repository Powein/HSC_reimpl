param(
    [int]$Steps = 40,
    [ValidateRange(0,12)][int]$Obstacles = 3,
    [ValidateSet('assembly','paper_stiffness')][string]$Profile = 'paper_stiffness',
    [double]$MotorSpeed = 2.0,
    [double]$StartX = -2.6,
    [int]$Rank = 128,
    [switch]$FixedNeumann,
    [string]$Label = ''
)
$ErrorActionPreference = 'Stop'
$runtime = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'runtime.json') | ConvertFrom-Json
$python = $runtime.python
if (-not $Label) { $Label = 'hsc_' + (Get-Date -Format 'yyyyMMdd_HHmmss') }
$culture = [System.Globalization.CultureInfo]::InvariantCulture
$hscArgs = @('--steps',$Steps,'--obstacles',$Obstacles,'--profile',$Profile,'--motor-speed',$MotorSpeed.ToString($culture),'--start-x',$StartX.ToString($culture),'--rank',$Rank,'--label',$Label)
if ($FixedNeumann) { $hscArgs += '--fixed-neumann' }
& $python (Join-Path $PSScriptRoot 'run_vehicle.py') @hscArgs
exit $LASTEXITCODE
