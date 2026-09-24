<#
.SYNOPSIS
    Thermal picture of the board: where the heat is, not just how hot.
.DESCRIPTION
    The board is an annulus, 100 mm across with a 10 mm bore.
.PARAMETER Simulated
    No cable.
.PARAMETER Switch
    Duty 0-1.
.PARAMETER Frames
    Stop after this many frames.
.EXAMPLE
    .\terminal\thermal_observer.ps1
#>
[CmdletBinding()]
param(
    [string] $Port = 'COM4',
    [switch] $Simulated,
    [double] $Hz = 2.0,
    [int]    $Frames = 0,
    [double] $Switch = -1,
    [string] $Phases = 'U,V,W'
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot

$argv = @('terminal/views/show_thermal_observer.py', '--port', $Port)
if ($Simulated) { $argv += '--simulated' }
$argv += @('--hz', $Hz)
if ($Frames -gt 0) { $argv += @('--frames', $Frames) }
if ($Switch -ge 0) { $argv += @('--switch', $Switch, '-P', $Phases) }

Push-Location (Join-Path $root 'host')
try {
    & python @argv
    $code = $LASTEXITCODE
} finally {
    Pop-Location
}

# 64 is show_thermal_observer.py's TO_MENU.
exit $code
