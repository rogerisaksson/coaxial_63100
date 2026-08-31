<#
.SYNOPSIS
    Every analog channel live, on a meter bridge.

.DESCRIPTION
    Seven strips, one per ADC channel, read off the board's own channel table
    - a board that grows a channel grows a strip and nothing here needs
    telling.

    The scale is dBFS against the CONVERTER'S full scale: a differential
    channel's +/-32768, a single-ended one's 65536. Logarithmic because
    linear is unreadable - nine amperes against a converter that runs to 207
    is four percent of the face, one segment, at the bottom, indistinguishable
    from nothing. Consoles have been logarithmic since the 1930s for the same
    reason.

    Three marks per strip, and they are three different measurements. The lit
    segments are the burst's mean. The ticks are the burst's own min and max,
    straight off the wire - what the channel did during the sample window,
    not something inferred here. The caret is peak hold, decaying, and it is
    the host's memory of the windows before.

    Ink stands in for colour: '=' low, '*' above -10 dBFS, '@' for OVER.
    Unlit segments are still drawn, so the ladder is visible when a channel
    is quiet.

    AFE_ON has to be on for any of it to mean anything - it powers the ADC
    reference, not just the signal path (invariant 9). If it is already on,
    this leaves it on. If it is off, this switches it on for the run and off
    again on the way out, Ctrl+C included.

    Nothing here judges a reading. There is no limit, no expected value and
    no mark that means bad; a channel sitting at OVER is drawn sitting there
    and left for the reader. Invariant 10 applies to a meter exactly as it
    applies to a table.

.PARAMETER Port
    The board's VCP. Ignored with -Simulated.

.PARAMETER Simulated
    No cable. Every value invented.

.PARAMETER Hz
    Screen refreshes per second.

.PARAMETER Rate
    Burst length per refresh. The min and max ticks are taken over this
    window, so a longer one catches more and responds slower.

.PARAMETER Frames
    Stop after this many rather than running until closed.

.EXAMPLE
    .\demos\adc.ps1
    .\demos\adc.ps1 -Simulated
    .\demos\adc.ps1 -Hz 4 -Rate 500
#>
param(
    [string]$Port = 'COM4',
    [switch]$Simulated,
    [double]$Hz = 8.0,
    [double]$Rate = 200.0,
    [int]$Frames = 0
)

# Continue, not Stop: a native exe writing to stderr becomes a
# NativeCommandError in PowerShell 5.1, and python does write there.
$ErrorActionPreference = 'Continue'

$Root = Split-Path -Parent $PSScriptRoot
. (Join-Path $Root 'env.ps1') -Quiet

Push-Location (Join-Path $Root 'host')
try {
    $call = @('tools/show_desk.py', '--hz', [string]$Hz, '--port', $Port,
              '--rate', [string]$Rate)
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
