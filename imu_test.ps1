<#
.SYNOPSIS
    The board's attitude, drawn live from the IMU.

.DESCRIPTION
    Enables the rotation vector and redraws the disc until you close it. The
    drawing is coaxial.orientation, which is pure and tested; this is the
    cable and the screen.

    Nothing here judges an orientation. It shows the quaternion the part
    reported and the angles that follow from it - invariant 10 applies to
    attitude exactly as it applies to a voltage.

.PARAMETER Port
    The board's VCP. Ignored with -Simulated.

.PARAMETER Simulated
    No cable: the stand-in turns slowly about Z so the picture has something
    to show. Every value invented, and the caption says which board it is.

.PARAMETER Hz
    Screen refreshes per second.

.PARAMETER Once
    One frame and exit, for checking the renderer without a terminal to
    close. Uses the stand-in unless -Port finds a board.

.EXAMPLE
    .\imu_test.ps1
    .\imu_test.ps1 -Simulated
    .\imu_test.ps1 -Once
#>
param(
    [string]$Port = 'COM4',
    [switch]$Simulated,
    [double]$Hz = 20.0,
    [int]$Frames = 0,
    [switch]$Once
)

# Continue, not Stop: a native exe writing to stderr becomes a
# NativeCommandError in PowerShell 5.1, and python does write there.
$ErrorActionPreference = 'Continue'

$Root = $PSScriptRoot
. (Join-Path $Root 'env.ps1') -Quiet

Push-Location (Join-Path $Root 'host')
try {
    if ($Once) {
        # No loop and no port: the renderer, once, from the stand-in. This is
        # the check that survives having no board and no patience.
        $code = @'
import sys
sys.path.insert(0, '.')
from coaxial.orientation import picture
from coaxial.simulated import SimulatedSession
part = SimulatedSession().board.imu
part.feature(0x05, 10000)
for report in part.read()['reports']:
    q = report.get('quaternion')
    if q:
        print(picture((q['i'], q['j'], q['k'], q['real']), frame=1))
        break
else:
    print('the stand-in sent no rotation vector')
'@
        & python -c $code
        return
    }

    $call = @('tools/show_orientation.py', '--hz', [string]$Hz,
              '--port', $Port)
    if ($Simulated) { $call += '--simulated' }
    if ($Frames -gt 0) { $call += @('--frames', [string]$Frames) }

    & python @call
} finally {
    Pop-Location
}
