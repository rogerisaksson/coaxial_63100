<#
.SYNOPSIS
    Pick one of the board's live views.

.DESCRIPTION
    The views live in .\demos and each one runs on its own. This is the menu
    in front of them, for when you want to look at the board rather than
    remember a filename.

    In a view: Q closes it, ESC comes back here. ESC rather than a keystroke
    somewhere else because picking the wrong view is the common mistake, and
    retyping the command to fix it is the annoying part. Both put the front
    end back the way the view found it, and so does Ctrl+C.

    Every view reads the board over the same session. With -Simulated none of
    them touch a port at all: the stand-in sweeps three phases 120 degrees
    apart like a machine turning, so the meters move, and every view says
    SIMULATED across the top for as long as it runs. Those numbers are
    invented and the banner is there so nobody has to remember that.

.PARAMETER Name
    Skip the chooser and go straight to one of them: session, imu, angle,
    adc, capture, gate_drivers, thermal.

    `session` is the first entry and carries every panel over one port. The
    other six are standalone and own the port on their own - which is what
    the session exists to avoid, and still the way to the rich rendering its
    panels do not carry.

.PARAMETER Port
    The board's VCP. Ignored with -Simulated.

.PARAMETER Simulated
    No cable. Every value invented, and the view says so.

.PARAMETER Frames
    Stop after this many rather than running until closed. Every view takes
    it, and it is how a view gets checked without someone holding Ctrl+C.

.EXAMPLE
    .\demo.ps1
    .\demo.ps1 adc
    .\demo.ps1 imu -Simulated
    .\demo.ps1 adc -Simulated -Frames 3
#>
param(
    [ValidateSet('session', 'imu', 'angle', 'adc', 'capture', 'gate_drivers',
                 'thermal')]
    [string]$Name,
    [string]$Port = 'COM4',
    [switch]$Simulated,
    [int]$Frames = 0
)

$ErrorActionPreference = 'Continue'

# The session first, because it is what most runs want and what the number
# keys land on. Script $null marks it: it is not one of demos/, it IS demos.
$Views = [ordered]@{
    'session' = @{ Script = $null
                 What   = 'one dash: analog, thermals, bridges, DIO, IMU, angle' }
    'imu'   = @{ Script = 'imu.ps1'
                 What   = 'board attitude, drawn from the STL the IMU turns' }
    'angle' = @{ Script = 'angle.ps1'
                 What   = 'shaft angle, the magnet and the air gap' }
    'adc'   = @{ Script = 'adc.ps1'
                 What   = 'every analog channel, on a meter bridge' }
    'capture' = @{ Script = 'capture.ps1'
                 What   = 'buffered: the AFE, the pins and both SPI parts' }
    'gate_drivers' = @{ Script = 'gate_drivers.ps1'
                 What   = 'the gate drivers: six signals, current, a burst' }
    'thermal' = @{ Script = 'thermal.ps1'
                 What   = 'where the heat sits, drawn on the board itself' }
}

# 64 is show_*.py's TO_MENU: ESC asking to come back here rather than close.
# Anything else - Q, Ctrl+C, an error - ends the run.
$TO_MENU = 64

function Read-Choice {
    <#
        One keystroke, no Enter. A menu of four things does not need a line
        editor, and having to press Return to look at a board is the kind of
        friction that stops anyone looking.

        ReadKey throws when input is redirected, so a run with no console -
        a smoke test, a pipe - falls back to reading a line.

        BOTH READS CAN THROW, and a throw has to end the menu rather than be
        retried. Read-Host throws outright in NonInteractive mode; the loop
        below then called ReadKey, which throws too, and kept calling it -
        thousands of identical exceptions and no way out. A console that
        cannot be read is quit, not a reason to ask again.
    #>
    if ([Console]::IsInputRedirected) {
        try { return Read-Host } catch { return $null }
    }

    while ($true) {
        try { $key = [Console]::ReadKey($true) } catch { return $null }
        if ($key.Key -eq 'Escape') { Write-Host 'q'; return 'q' }
        $char = $key.KeyChar
        if ($char -match '^[0-9a-zA-Z]$') {
            Write-Host $char
            return [string]$char
        }
    }
}

function Read-View($Views) {
    Write-Host ''
    Write-Host '  coaxial_63100 - live views' -ForegroundColor White
    Write-Host ''

    # The name column is as wide as the longest name, not a number typed in
    # once: 'capture' is exactly 7 and 'gate_drivers' is 12, so a fixed 7 ran
    # both of them straight into their description.
    $keys = @($Views.Keys)
    $width = ($keys | Measure-Object -Property Length -Maximum).Maximum + 3
    $name = '{0,-' + $width + '}'

    for ($i = 0; $i -lt $keys.Count; $i++) {
        $key = $keys[$i]
        Write-Host ('    {0}   ' -f ($i + 1)) -NoNewline -ForegroundColor Cyan
        Write-Host ($name -f $key) -NoNewline -ForegroundColor White
        Write-Host $Views[$key].What -ForegroundColor DarkGray
    }

    Write-Host ''
    Write-Host '    q   ' -NoNewline -ForegroundColor Cyan
    Write-Host 'quit' -ForegroundColor DarkGray
    Write-Host ''
    Write-Host '  which ' -NoNewline

    $answer = Read-Choice

    # Empty first: with no console behind it Read-Choice falls back to
    # Read-Host, which returns $null, and $Views.Contains($null) throws
    # rather than answering false.
    if (-not $answer -or $answer -match '^(q|quit|exit)$') { return $null }
    if ($answer -match '^\d+$' -and
        [int]$answer -ge 1 -and [int]$answer -le $keys.Count) {
        return $keys[[int]$answer - 1]
    }
    if ($Views.Contains($answer)) { return $answer }

    Write-Host ('  no view called {0}' -f $answer) -ForegroundColor Yellow
    return ''
}

# THE SESSION IS AN ENTRY, not the whole of it. It was made the no-argument
# case once, which left Read-View unreachable from either branch - the chooser
# was dead for as long as it took someone to miss it. It is first on the list
# instead, so one key still gets there and the other six are visible again.
$asked = $Name
$code = 0

do {
    $view = $asked
    $asked = $null

    while (-not $view) {
        $view = Read-View $Views
        if ($null -eq $view) { exit 0 }
    }

    # INLINE, and not behind a function returning the code. Both ways of
    # getting the code out of a function redirect the child: assigning its
    # output captures the frames, and piping to Out-Host makes stdout a pipe
    # - so `sys.stdout.isatty()` went false, the session stopped clearing the
    # screen and repainted whole frames instead of the rows that changed.
    if (-not $Views[$view].Script) {
        Push-Location (Join-Path $PSScriptRoot 'host')
        $argv = @('tools/demos.py', '--port', $Port)
        if ($Simulated) { $argv += '--simulated' }
        if ($Frames -gt 0) { $argv += @('--frames', $Frames) }
        & python @argv
        $code = $LASTEXITCODE
        Pop-Location
        continue
    }

    $call = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File',
              (Join-Path $PSScriptRoot (Join-Path 'demos' $Views[$view].Script)),
              '-Port', $Port)
    if ($Simulated) { $call += '-Simulated' }
    if ($Frames -gt 0) { $call += @('-Frames', [string]$Frames) }

    & powershell @call
    $code = $LASTEXITCODE
} while ($code -eq $TO_MENU)

exit $code
