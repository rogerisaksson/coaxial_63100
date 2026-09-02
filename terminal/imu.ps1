<#
.SYNOPSIS
    The board's attitude, drawn live from the IMU it says it has.

.DESCRIPTION
    Reads the board's own parts list first (command 0x6D kind 4) and looks for
    an IMU in it, then prints a preflight line per step the way
    board_chat.ps1 does - ok, warn or fail, with what it found beside it.
    Nothing here decides the board has an IMU: a board without the part says
    so itself, and a board that grows one needs no change here.

    AFE_ON powers that part as well as the analog front end. If it is already
    on, this leaves it on. If it is off, this switches it on for the run and
    switches it off again on the way out - Ctrl+C included, which also
    disables the rotation vector it enabled. What the board looked like when
    this started is what it looks like when it finishes.

    The drawing is coaxial.orientation, which is pure and tested; this is the
    cable and the screen. Nothing here judges an orientation: it shows the
    quaternion the part reported and the angles that follow from it, and
    invariant 10 applies to attitude exactly as it applies to a voltage.

.PARAMETER Port
    The board's VCP. Ignored with -Simulated.

.PARAMETER Simulated
    No cable: the stand-in turns slowly about Z so the picture has something
    to show. Every value invented, and the caption says which board it is.

.PARAMETER Hz
    Screen refreshes per second.

.PARAMETER Frames
    Stop after this many rather than running until closed.

.PARAMETER Once
    One frame from the stand-in and exit, for checking the renderer without a
    board and without a terminal to close.

.EXAMPLE
    .\terminal\imu.ps1
    .\terminal\imu.ps1 -Simulated
    .\terminal\imu.ps1 -Once
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

$Root = Split-Path -Parent $PSScriptRoot
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

    # No size: the view fills the window. A board is mostly flat, so what
    # shows its components is each one covering several cells - at 34x15 none
    # of them does and it draws as a featureless disc. --width/--height pin
    # it if a fixed size is wanted.
    $call = @('tools/show_orientation.py', '--hz', [string]$Hz,
              '--port', $Port)
    if ($Simulated) { $call += '--simulated' }
    if ($Frames -gt 0) { $call += @('--frames', [string]$Frames) }

    & python @call
    # The view's own exit code, not this wrapper's: 64 is ESC asking
    # coaxial_tty.ps1 for the menu, and a script that does not pass it on
    # exits 0 and the menu never comes back.
    $code = $LASTEXITCODE
} finally {
    Pop-Location
}

exit $code
