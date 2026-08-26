<#
.SYNOPSIS
    The shaft angle, drawn live from the A1335 the board says it has.

.DESCRIPTION
    Reads the board's own parts list first (command 0x6D kind 4) and looks
    for an angle sensor in it. Nothing here decides the board has one: a
    board without the part says so itself, and a board that grows one needs
    no change here.

    AFE_ON powers that part as well as the analog front end. If it is already
    on, this leaves it on. If it is off, this switches it on for the run and
    switches it off again on the way out, Ctrl+C included. What the board
    looked like when this started is what it looks like when it finishes.

    The picture is the magnet on the end of the shaft, seen from the front,
    with the sensor below the axis looking up at its face - which is where it
    sits on the PCB. The field strength is read once at the start: below a
    few tens of gauss there is no magnet in front of the sensor, and the
    angle is noise. The picture says so instead of drawing a confident
    pointer.

    Nothing here judges an angle. It shows the counts the part reported and
    the degrees that follow from them - invariant 10 applies to a shaft angle
    exactly as it applies to a voltage.

.PARAMETER Port
    The board's VCP. Ignored with -Simulated.

.PARAMETER Simulated
    No cable: the stand-in turns once every twelve seconds so the picture has
    something to show. Every value invented.

.PARAMETER Hz
    Screen refreshes per second.

.PARAMETER Frames
    Stop after this many rather than running until closed.

.EXAMPLE
    .\angle_test.ps1
    .\angle_test.ps1 -Simulated
    .\angle_test.ps1 -Frames 40
#>
param(
    [string]$Port = 'COM4',
    [switch]$Simulated,
    [double]$Hz = 20.0,
    [int]$Frames = 0
)

# Continue, not Stop: a native exe writing to stderr becomes a
# NativeCommandError in PowerShell 5.1, and python does write there.
$ErrorActionPreference = 'Continue'

$Root = $PSScriptRoot
. (Join-Path $Root 'env.ps1') -Quiet

Push-Location (Join-Path $Root 'host')
try {
    $call = @('tools/show_angle.py', '--hz', [string]$Hz, '--port', $Port)
    if ($Simulated) { $call += '--simulated' }
    if ($Frames -gt 0) { $call += @('--frames', [string]$Frames) }

    & python @call
} finally {
    Pop-Location
}
