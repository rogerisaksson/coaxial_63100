<#
.SYNOPSIS
    Thermal picture of the board: where the heat is, not just how hot.

.DESCRIPTION
    The board is an annulus, 100 mm across with a 10 mm bore. Zones are drawn
    where they sit - switches in a line across the top, the supply out to the
    left, the AFE along the bottom, the DC link upper right.

    The field is diffuse on purpose: heat in a laminate spreads, and sharp
    zones would be a lie about the physics. Each source is a Gaussian blob.

    The zone temperatures are the observer running in the firmware at 10 Hz,
    read over 0x6E device 8 - not recomputed here.

    AFE_ON is left as found. The gate is inverted, so switching it on would
    take the gate drivers' supply away, and with it the load worth watching.
    While it is off there is no NTC either and the model runs open on power
    and time.

.PARAMETER Simulated
    No cable. Every value is invented.

.PARAMETER Switch
    Duty 0-1. Arms the gate drivers and holds it while drawing, so the zones
    have a load to follow. Without it the view only watches.

.PARAMETER Frames
    Stop after this many frames. Without it the view runs until Q, ESC or
    Ctrl+C.

.EXAMPLE
    .\demos\thermal.ps1
    .\demos\thermal.ps1 -Simulated
    .\demos\thermal.ps1 -Hz 4
    .\demos\thermal.ps1 -Switch 0.5
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

$argv = @('tools/show_thermal.py', '--port', $Port)
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

# 64 is show_thermal.py's TO_MENU. Swallowing it here is what stopped ESC
# from going back to demo.ps1's menu - the view returned it and the wrapper
# dropped it on the floor.
exit $code
