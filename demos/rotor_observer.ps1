<#
.SYNOPSIS
    The rotor observer: the drive watched live, on the model or the converters.

.DESCRIPTION
    The drive - 0x6E device 10 - runs on the board at the PWM rate. This
    view sets what it is asked to do and draws what it did: the estimated
    rotor angle on the dial, the model's own rotor beside it when the model
    is the source, the dq currents, the innovation, the interrupt's cost.

    With -Source model the samples come from the PMSM the board integrates
    itself, so the observer can be watched working with the AFE off and no
    motor - which is what this bench board allows. With -Source adc the
    real converters feed it, and AFE_ON has to be on.

    Nothing switches unless -Switch is given, and then only when A is
    pressed: arming is arming a power stage, and on this bench board the
    drivers have supply exactly when the AFE is off.

.PARAMETER Port
    The board's VCP. Ignored with -Simulated.

.PARAMETER Source
    model (default) or adc.

.PARAMETER Motor
    A profile under host\motors\, e.g. outrunner_14p.json, written to the
    board before the run.

.PARAMETER Switch
    Let A arm the stage: gates.arm(bypass_sto, ignore_interlock).

.PARAMETER Simulated
    No cable. Every value invented, and the view says so.

.PARAMETER Hz
    Screen refreshes per second.

.PARAMETER Frames
    Stop after this many, for checking the view without a terminal to close.

.PARAMETER Extra
    Anything else, passed straight to show_rotor_observer.py - every parameter of
    the drive is a switch there: --iq, --id, --omega, --v-inj, --kp, --ki,
    --l1, --l2, --i-max, --i-trip, --vdc, --load, --noise, --theta0 ...

.EXAMPLE
    .\demos\rotor_observer.ps1 -Simulated
    .\demos\rotor_observer.ps1 -Source model -Motor outrunner_14p.json
    .\demos\rotor_observer.ps1 -Source model -Switch -Extra '--iq','0.3'
#>
param(
    [string]$Port = 'COM4',
    [ValidateSet('model', 'adc')][string]$Source = 'model',
    [string]$Motor,
    [switch]$Switch,
    [switch]$Simulated,
    [double]$Hz = 8.0,
    [int]$Frames = 0,
    [string[]]$Extra = @()
)

# Continue, not Stop: a native exe writing to stderr becomes a
# NativeCommandError in PowerShell 5.1, and python does write there.
$ErrorActionPreference = 'Continue'

$Root = Split-Path -Parent $PSScriptRoot
. (Join-Path $Root 'env.ps1') -Quiet

Push-Location (Join-Path $Root 'host')
try {
    $call = @('tools/show_rotor_observer.py', '--hz', [string]$Hz, '--port', $Port,
              '--source', $Source)
    if ($Motor)     { $call += @('--motor', $Motor) }
    if ($Switch)    { $call += '--switch' }
    if ($Simulated) { $call += '--simulated' }
    if ($Frames -gt 0) { $call += @('--frames', [string]$Frames) }
    $call += $Extra

    & python @call
    # The view's own exit code: 64 is ESC asking coaxial_tty.ps1 for the menu.
    $code = $LASTEXITCODE
} finally {
    Pop-Location
}

exit $code
