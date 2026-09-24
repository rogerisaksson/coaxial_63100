<#
.SYNOPSIS
    Runs the test suites and reports what passed, what did not, and how
    long.
.DESCRIPTION
    Coverage tiers, as a percentage of every check there is.
.EXAMPLE
    .\run_tests.ps1                      # ~25 %, the default
#>
[CmdletBinding(DefaultParameterSetName = 'AutomaticMinimal')]
param(
    [Parameter(ParameterSetName = 'All')][switch]$All,
    [Parameter(ParameterSetName = 'AutomaticHigh')][switch]$AutomaticHigh,
    [Parameter(ParameterSetName = 'AutomaticMedium')][switch]$AutomaticMedium,
    [Parameter(ParameterSetName = 'AutomaticMinimal')][switch]$AutomaticMinimal,
    [Parameter(ParameterSetName = 'Only', Mandatory = $true)][string[]]$Only,
    [Parameter(ParameterSetName = 'Structure')][switch]$Structure,
    [Parameter(ParameterSetName = 'Depth', Mandatory = $true)]
    [ValidateScript({ $_ -ge 5 -and $_ -le 100 -and $_ % 5 -eq 0 })]
    [int]$Depth,
    [string[]]$Tags,
    [string[]]$Scope,
    [string]$Model = 'auto',
    [switch]$DryRun
)

# Continue, not Stop: a native exe writing to stderr becomes a
# NativeCommandError in PowerShell 5.1, and the suites do write there.
$ErrorActionPreference = 'Continue'
$hostDir = $PSScriptRoot

function Write-Rule([string]$Text) {
    Write-Host ''
    Write-Host ("-- $Text " + ('-' * [Math]::Max(0, 66 - $Text.Length))) -ForegroundColor DarkGray
}

switch ($PSCmdlet.ParameterSetName) {
    'All'             { $mode = 'All (100 %)';    $runArgs = @('--conformance', '--live') }
    'AutomaticHigh'   { $mode = 'Automatic 75 %'; $runArgs = @('--coverage', '75') }
    'AutomaticMedium' { $mode = 'Automatic 50 %'; $runArgs = @('--coverage', '50') }
    'Only'            { $named = $Only -join ','
                        $mode = "Only $named"; $runArgs = @('--only', $named) }
    'Structure'       { $mode = 'Structure';      $runArgs = @('--structure') }
    'Depth'           { $mode = "Depth $Depth %";  $runArgs = @('--coverage', [string]$Depth) }
    default           { $mode = 'Automatic 25 %'; $runArgs = @('--coverage', '25') }
}
# Scope wins the label as well as the plan: --file makes the runner skip the
# smart pick entirely, so a header still reading "Automatic 25 %" would be
# describing a tier that had no say in what ran.
if ($Scope) {
    # -File hands 'a,b' over as ONE string, not a [string[]] of two - measured:
    # -Scope test_conformance.py,test_mcp.py ran neither, and the only trace
    # was a MISSING line the filter below did not surface.
    $Scope = @($Scope -split ',' | ForEach-Object { $_.Trim() } |
               Where-Object { $_ })
    foreach ($f in $Scope) { $runArgs += @('--file', $f) }
    $mode = 'Scope ' + ($Scope -join ', ')
}
if ($Tags)   { $runArgs += @('--tags', ($Tags -join ',')) }
if ($DryRun) { $runArgs += '--dry-run' }
$runArgs += @('--model', $Model)

Write-Rule $mode

# One invocation, not a --dry-run and then the real thing.
Push-Location $hostDir
$started = Get-Date
$output = & python tools/dev/run_tests.py @runArgs
$code = $LASTEXITCODE
$elapsed = (Get-Date) - $started
Pop-Location

$rows = @()
$failures = @()
$crashes = @()
$total = $null

foreach ($line in $output) {
    $text = [string]$line
    if ($text -match '^(?<name>\S+\.py)\s+(?<pass>\d+) passed, (?<fail>\d+) failed\s+(?<secs>[\d.]+)s$') {
        $rows += [pscustomobject]@{
            Suite   = $Matches.name
            Passed  = [int]$Matches.pass
            Failed  = [int]$Matches.fail
            Seconds = [double]$Matches.secs
        }
    }
    elseif ($text -match '^(?<name>\S+\.py)\s+CRASHED exit=(?<code>\S+)\s+(?<secs>[\d.]+)s$') {
        $crashes += [pscustomobject]@{ Suite = $Matches.name; Exit = $Matches.code; Seconds = [double]$Matches.secs }
    }
    elseif ($text -match '^Total:') { $total = $text }
    elseif ($text -match '^\s{2}(?<suite>\S+\.py): (?<what>.+)$') {
        $failures += [pscustomobject]@{ Suite = $Matches.suite; Check = $Matches.what }
    }
    elseif ($text -match '^\s*(suites|subjects):|% tier:') {
        Write-Host $text -ForegroundColor Cyan
    }
    elseif ($text -match 'MISSING') {
        Write-Host $text -ForegroundColor Red
    }
    elseif ($text -match '^--|^\s*(ran \d+ of \d+ groups|holding |released |no such|have:|   )') {
        Write-Host $text -ForegroundColor DarkGray
    }
}

Write-Rule 'results'

if ($rows) {
    $width = ($rows.Suite | Measure-Object -Property Length -Maximum).Maximum
    foreach ($row in $rows) {
        $colour = if ($row.Failed -gt 0) { 'Red' } else { 'Green' }
        $mark = if ($row.Failed -gt 0) { 'FAIL' } else { 'ok  ' }
        Write-Host ('{0}  {1}  {2,5} passed  {3,3} failed  {4,7:N1}s' -f
            $mark, $row.Suite.PadRight($width), $row.Passed, $row.Failed, $row.Seconds) -ForegroundColor $colour
    }
}

foreach ($crash in $crashes) {
    Write-Host ('CRASH {0} exited {1} after {2:N1}s - it never printed a tally' -f
        $crash.Suite, $crash.Exit, $crash.Seconds) -ForegroundColor Magenta
}

if ($failures) {
    Write-Rule "$($failures.Count) failing check(s)"
    foreach ($failure in $failures) {
        Write-Host ('  {0}' -f $failure.Suite) -NoNewline -ForegroundColor DarkGray
        Write-Host ('  {0}' -f $failure.Check) -ForegroundColor Red
    }
}

Write-Host ''
if ($total) { Write-Host $total -ForegroundColor White }
Write-Host ('{0} in {1:N1}s' -f $mode, $elapsed.TotalSeconds) -ForegroundColor DarkGray

# 130 is run_tests.py's STOPPED: Ctrl+C, not a suite saying no.
if ($code -eq 130) {
    Write-Host 'STOPPED' -ForegroundColor Yellow
} elseif ($code -ne 0 -or $failures -or $crashes) {
    Write-Host 'FAILED' -ForegroundColor Red
} else {
    Write-Host 'PASSED' -ForegroundColor Green
}
exit $code
