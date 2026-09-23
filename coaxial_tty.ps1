<#
.SYNOPSIS
    The terminal: the front page and the board's live views.

.DESCRIPTION
    One process, `python -m terminal` (host/terminal/): the loader reads
    the pages under host/terminal/pages/, lists them on the front page,
    preloads the model into memory and runs the picked page there - so a
    second page opens on what the first already loaded. This script is
    the shortcut in front of it; the wrappers in terminal/ still start
    one view on its own.

    In a view: Q closes the terminal, ESC comes back to the front page -
    on the second question the view came from, with it lit. Both put the
    board back the way the view found it, and so does Ctrl+C. On the way
    out the session is opened once more and whatever a page left running
    is stopped, so "nothing was left running" is measured.

    Every view reads the board over the same session. With -Simulated
    none of them touch a port: the stand-in sweeps three phases 120
    degrees apart like a machine turning, so the meters move, and every
    view says SIMULATED across the top for as long as it runs.

.PARAMETER Name
    Skip the front page and go straight to a page or an item by name:
    session, imu, angle, adc, gate_drivers, rotor_observer,
    thermal_observer, chat, claude.

.PARAMETER Port
    The board's VCP. Ignored with -Simulated.

.PARAMETER Simulated
    No cable. Every value invented, and the view says so.

.PARAMETER Frames
    Stop after this many rather than running until closed - the smoke.

.EXAMPLE
    .\coaxial_tty.ps1
    .\coaxial_tty.ps1 adc
    .\coaxial_tty.ps1 imu -Simulated
    .\coaxial_tty.ps1 -Simulated -Frames 3
#>
param(
    [ValidateSet('session', 'imu', 'angle', 'adc', 'gate_drivers',
                 'rotor_observer', 'thermal_observer', 'chat', 'claude')]
    [string]$Name,
    [string]$Port = 'COM4',
    [switch]$Simulated,
    [int]$Frames = 0
)

# Continue, not Stop: a native exe writing to stderr becomes a
# NativeCommandError in PowerShell 5.1, and python does write there.
$ErrorActionPreference = 'Continue'

# The frames are box drawing; a console left on the OEM codepage prints
# them as mojibake. Same line board_chat.ps1 runs, for the same reason.
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

. (Join-Path $PSScriptRoot 'env.ps1') -Quiet

$argv = @('-X', 'utf8', '-m', 'terminal')
if ($Name) { $argv += $Name }
$argv += @('--port', $Port)
if ($Simulated) { $argv += '--simulated' }
if ($Frames -gt 0) { $argv += @('--frames', [string]$Frames) }

Push-Location (Join-Path $PSScriptRoot 'host')
try {
    & python @argv
    $code = $LASTEXITCODE
} finally {
    Pop-Location
}
exit $code
