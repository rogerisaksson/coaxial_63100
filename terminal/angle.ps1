<#
.SYNOPSIS
    The shaft angle, drawn live from the A1335 the board says it has.
.DESCRIPTION
    Reads the board's own parts list first (command 0x6D kind 4) and looks
    for an angle sensor in it.
.PARAMETER Port
    The board's VCP.
.PARAMETER Simulated
    No cable: the stand-in turns once every twelve seconds so the picture
    has something to show.
.PARAMETER Hz
    Screen refreshes per second.
.PARAMETER Frames
    Stop after this many rather than running until closed.
.EXAMPLE
    .\terminal\angle.ps1
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

$Root = Split-Path -Parent $PSScriptRoot
. (Join-Path $Root 'env.ps1') -Quiet

Push-Location (Join-Path $Root 'host')
try {
    $call = @('terminal/views/show_angle.py', '--hz', [string]$Hz, '--port', $Port)
    if ($Simulated) { $call += '--simulated' }
    if ($Frames -gt 0) { $call += @('--frames', [string]$Frames) }

    & python @call
    # The view's own exit code, not this wrapper's: 64 is ESC asking
    # coaxial_tty.ps1 for the menu, and a script that does not pass it on exits
    # 0 and the menu never comes back.
    $code = $LASTEXITCODE
} finally {
    Pop-Location
}

exit $code
