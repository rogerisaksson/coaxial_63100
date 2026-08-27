<#
.SYNOPSIS
    Buffered capture: the AFE, the digital pins and both SPI parts at once.

.DESCRIPTION
    Nothing here is polled per value. Two buffers on the board feed it: the
    acquisition task (0x6E device 6) samples every analog channel and the
    digital word into a ring, and the event ring (device 5) takes what the
    angle and IMU loops produce. This drains both.

    Every name comes off the wire. The analog fields, their units and the
    digital bits are named by the task's own layout, so a channel added to
    Board/Src/board_adc.c appears here with nothing told.

    The rate is the board's, not this script's. Asked for nothing, the board
    works out what the link can carry from its own record stride and the
    baud of whichever port is answering - 105 records a second for seven
    channels plus the digital word at 115200, and more on a faster segment.

    When the board still reports drops the view raises accumulation, in
    doublings, and marks it with an asterisk. Accumulation and not
    decimation: summing keeps every sample's contribution where subsampling
    throws it away. Measured, seven channels and the digital word drop 3851
    records at accumulate 1 and none at 16. It happens on the target, before
    a byte is sent, which is the only place it saves anything - the payload
    ceiling is about 3.8 kB/s whatever the record size.

    AFE_ON has to be on for any of it to mean anything: it powers the ADC
    reference and both SPI parts, not just the signal path (invariant 9).
    If it is already on this leaves it on; if it is off it goes on for the
    run and off again on the way out, Ctrl+C included.

    Nothing here judges a reading. Raw codes and the board's own units.

.PARAMETER Port
    The board's VCP. Ignored with -Simulated.

.PARAMETER Simulated
    No cable. Every value invented, and the view says so.

.PARAMETER Hz
    Screen refreshes per second.

.PARAMETER Rate
    The software clock, in hertz. 0 lets the board pick from the link.

.PARAMETER Accumulate
    Samples summed per record to start with. Raised on its own when the
    board reports drops.

.PARAMETER SampleTime
    The converter's own sampling window, 0..7, shortest first.

.PARAMETER Frames
    Stop after this many, for checking the view without a terminal to close.

.EXAMPLE
    .\demos\capture.ps1
    .\demos\capture.ps1 -Simulated
    .\demos\capture.ps1 -Rate 0 -SampleTime 5
#>
param(
    [string]$Port = 'COM4',
    [switch]$Simulated,
    [double]$Hz = 8.0,
    [double]$Rate = 2000.0,
    [int]$Accumulate = 1,
    [int]$SampleTime = 0,
    [int]$Frames = 0
)

# Continue, not Stop: a native exe writing to stderr becomes a
# NativeCommandError in PowerShell 5.1, and python does write there.
$ErrorActionPreference = 'Continue'

$Root = Split-Path -Parent $PSScriptRoot
. (Join-Path $Root 'env.ps1') -Quiet

Push-Location (Join-Path $Root 'host')
try {
    $call = @('tools/show_capture.py', '--hz', [string]$Hz, '--port', $Port,
              '--rate', [string]$Rate, '--accumulate', [string]$Accumulate,
              '--sample-time', [string]$SampleTime)
    if ($Simulated) { $call += '--simulated' }
    if ($Frames -gt 0) { $call += @('--frames', [string]$Frames) }

    & python @call
    # The view's own exit code, not this wrapper's: 64 is ESC asking
    # demo.ps1 for the menu, and a script that does not pass it on
    # exits 0 and the menu never comes back.
    $code = $LASTEXITCODE
} finally {
    Pop-Location
}

exit $code
