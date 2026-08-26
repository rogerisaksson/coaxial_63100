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
    Skip the menu: imu, angle or adc. ESC still comes back to it.

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
    [ValidateSet('imu', 'angle', 'adc')]
    [string]$Name,
    [string]$Port = 'COM4',
    [switch]$Simulated,
    [int]$Frames = 0
)

$ErrorActionPreference = 'Continue'

$Views = [ordered]@{
    'imu'   = @{ Script = 'imu.ps1'
                 What   = 'board attitude, drawn from the STL the IMU turns' }
    'angle' = @{ Script = 'angle.ps1'
                 What   = 'shaft angle, the magnet and the air gap' }
    'adc'   = @{ Script = 'adc.ps1'
                 What   = 'every analog channel, on a meter bridge' }
}

# 64 is show_*.py's TO_MENU: ESC asking to come back here rather than close.
# Anything else - Q, Ctrl+C, an error - ends the run.
$TO_MENU = 64

function Read-View($Views) {
    Write-Host ''
    Write-Host '  coaxial_63100 - live views' -ForegroundColor White
    Write-Host ''

    $keys = @($Views.Keys)
    for ($i = 0; $i -lt $keys.Count; $i++) {
        $key = $keys[$i]
        Write-Host ('    {0}  ' -f ($i + 1)) -NoNewline -ForegroundColor Cyan
        Write-Host ('{0,-7}' -f $key) -NoNewline -ForegroundColor White
        Write-Host $Views[$key].What -ForegroundColor DarkGray
    }

    Write-Host ''
    Write-Host '    q  ' -NoNewline -ForegroundColor Cyan
    Write-Host 'quit' -ForegroundColor DarkGray
    Write-Host ''

    $answer = Read-Host '  which'

    # Empty first: Read-Host returns $null with no console behind it, and
    # $Views.Contains($null) throws rather than answering false.
    if (-not $answer -or $answer -match '^(q|quit|exit)$') { return $null }
    if ($answer -match '^\d+$' -and
        [int]$answer -ge 1 -and [int]$answer -le $keys.Count) {
        return $keys[[int]$answer - 1]
    }
    if ($Views.Contains($answer)) { return $answer }

    Write-Host ('  no view called {0}' -f $answer) -ForegroundColor Yellow
    return ''
}

# The name given on the command line is used once; after that the menu asks,
# so ESC out of `demo.ps1 adc` lands somewhere you can choose again rather
# than reopening the view just left.
$asked = $Name
$code = 0

do {
    $view = $asked
    $asked = $null

    while (-not $view) {
        $view = Read-View $Views
        if ($null -eq $view) { exit 0 }
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
