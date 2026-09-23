<#
.SYNOPSIS
    Every analog channel live, on a meter bridge.
.DESCRIPTION
    Seven strips, one per ADC channel, read off the board's own channel
    table - a board that grows a channel grows a strip and nothing here
    needs telling.
.PARAMETER Port
    The board's VCP.
.PARAMETER Simulated
    No cable.
.PARAMETER Hz
    Screen refreshes per second.
.PARAMETER Rate
    Records a second the board produces.
.PARAMETER Frames
    Stop after this many rather than running until closed.
.EXAMPLE
    .\terminal\adc.ps1
#>
param(
    [string]$Port = 'COM4',
    [switch]$Simulated,
    [double]$Hz = 8.0,
    [double]$Rate = 0,
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
    # coaxial_tty.ps1 for the menu, and a script that does not pass it on exits
    # 0 and the menu never comes back.
    $code = $LASTEXITCODE
} finally {
    Pop-Location
}

exit $code
