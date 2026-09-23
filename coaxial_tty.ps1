<#
.SYNOPSIS
    The terminal: the front page and the board's live views.
.DESCRIPTION
    One process, `python -m terminal` (host/terminal/): the loader reads the
    pages under host/terminal/pages/, lists them on the front page, preloads
    the model into memory and runs the picked page there - so a second page
    opens on what the first already loaded.
.PARAMETER Name
    Skip the front page and go straight to a page or an item by name:
    session, imu, angle, adc, gate_drivers, rotor_observer,
    thermal_observer, chat, claude.
.PARAMETER Port
    The board's VCP.
.PARAMETER Simulated
    No cable.
.PARAMETER Frames
    Stop after this many rather than running until closed - the smoke.
.EXAMPLE
    .\coaxial_tty.ps1
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

# The frames are box drawing; a console left on the OEM codepage prints them as
# mojibake.
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
