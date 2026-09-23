<#
.SYNOPSIS
    The gate drivers: the six signals, the currents, and a timed burst.
.DESCRIPTION
    What the gate drivers are doing, from three sources that mislead
    separately.
.PARAMETER Port
    The board's VCP.
.PARAMETER Afe
    AFE_ON on: currents are real and the drivers have no supply.
.PARAMETER Simulated
    No cable.
.PARAMETER Hz
    Screen refreshes per second.
.PARAMETER Frames
    Stop after this many, for checking the view without a terminal to close.
.EXAMPLE
    .\terminal\gate_drivers.ps1
#>
param(
    [string]$Port = 'COM4',
    [switch]$Afe,
    [switch]$Simulated,
    [double]$Hz = 8.0,
    [int]$Frames = 0
)

# Continue, not Stop: a native exe writing to stderr becomes a
# NativeCommandError in PowerShell 5.1, and python does write there.
$ErrorActionPreference = 'Continue'

$Root = Split-Path -Parent $PSScriptRoot
. (Join-Path $Root 'env.ps1') -Quiet

Push-Location (Join-Path $Root 'host')
try {
    $call = @('tools/show_gate_drivers.py', '--hz', [string]$Hz, '--port', $Port)
    if ($Afe)       { $call += '--afe' }
    if ($Simulated) { $call += '--simulated' }
    if ($Frames -gt 0) { $call += @('--frames', [string]$Frames) }

    & python @call
    # The view's own exit code: 64 is ESC asking coaxial_tty.ps1 for the menu.
    $code = $LASTEXITCODE
} finally {
    Pop-Location
}

exit $code
