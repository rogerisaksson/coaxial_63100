<#
.SYNOPSIS
    The gate drivers: the six signals, the currents, and a timed burst.

.DESCRIPTION
    What the gate drivers are doing, from three sources that mislead separately.
    The gate snapshot is one IDR read on the board, so the six signals are
    the same instant - six asks at 50 kHz can straddle an edge and show a
    leg with both FETs on, which is the one state the dead time prevents.
    The currents and the DC link come from the acquisition task's live
    accumulator, which carries a count, a lowest and a highest per channel,
    so ripple is measured rather than inferred from one sample.

    Arming is arming a power stage. The 2EDL8034's inputs are independent
    and it has no interlock of its own, so TIM1's 80 ns dead time is all
    there is between the two FETs of a leg; the view re-reads BDTR DTG and
    refuses to arm a gate driver stage reporting zero.

    On this bench board AFE_ON is inverted: the drivers have supply while it
    is OFF, which is how this runs by default - and with it off the board
    refuses to convert at all, so there are no currents. -Afe runs it the
    other way: real currents, unpowered drivers. Switching and measuring are
    mutually exclusive here until the patch.

.PARAMETER Port
    The board's VCP. Ignored with -Simulated.

.PARAMETER Afe
    AFE_ON on: currents are real and the drivers have no supply.

.PARAMETER Simulated
    No cable. Every value invented, and the view says so.

.PARAMETER Hz
    Screen refreshes per second.

.PARAMETER Frames
    Stop after this many, for checking the view without a terminal to close.

.EXAMPLE
    .\demos\gate_drivers.ps1
    .\demos\gate_drivers.ps1 -Afe
    .\demos\gate_drivers.ps1 -Simulated
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
    # The view's own exit code: 64 is ESC asking demo.ps1 for the menu.
    $code = $LASTEXITCODE
} finally {
    Pop-Location
}

exit $code
