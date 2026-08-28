<#
.SYNOPSIS
    One session: run the gate drivers, then look at whatever you like.

.DESCRIPTION
    Every other view here opens its own rig and owns the serial port, so
    switching the drivers and watching the heat meant two processes and one
    port. This holds one rig and lets panels come and go over it.

    The activity outlives the panel. Start the stage switching, move to the
    thermal panel, and it keeps switching - the session owns it, the view
    only draws.

        1 2 3   the panels
        s       gate drivers, three legs at 50 %
        d       DAQ running
        q       quit, and everything the session started is stopped

    WHAT THE BOARD CANNOT DO, and the session says so on the sensors panel:
    AFE_ON is inverted, so the drivers have supply only while the analog
    front end does not - and AFE_ON is what powers the IMU, the angle sensor
    and the ADC reference. There is no watching the IMU react to switching;
    it is unpowered for the duration. The thermal observer is what keeps
    answering, on power and time between samples.

.PARAMETER Simulated
    No cable. Every value is invented.

.EXAMPLE
    .\demos\workbench.ps1
    .\demos\workbench.ps1 -Simulated
#>
[CmdletBinding()]
param(
    [string] $Port = 'COM4',
    [switch] $Simulated,
    [double] $Hz = 2.0,
    [int]    $Frames = 0
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot

$argv = @('tools/workbench.py', '--port', $Port, '--hz', $Hz)
if ($Simulated) { $argv += '--simulated' }
if ($Frames -gt 0) { $argv += @('--frames', $Frames) }

Push-Location (Join-Path $root 'host')
try {
    & python @argv
    $code = $LASTEXITCODE
} finally {
    Pop-Location
}

# 64 is TO_MENU: ESC asking demo.ps1 to draw its menu again rather than close.
exit $code
