<#
.SYNOPSIS
    Pick one of the board's live views.

.DESCRIPTION
    The views live in .	erminal and each one runs on its own. This is the menu
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
    adc, gate_drivers, rotor_observer, thermal_observer. On the front page
    gate_drivers and rotor_observer sit under MOTOR CONTROLLER, which asks which half, and
    chat and claude under BOARD CHAT, which asks who answers; ESC from a
    view under either comes back to that question. The old capture view is a box in the session.

    `session` is the first entry and carries every panel over one port. The
    other seven are standalone and own the port on their own - which is what
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
    .\coaxial_tty.ps1
    .\coaxial_tty.ps1 adc
    .\coaxial_tty.ps1 imu -Simulated
    .\coaxial_tty.ps1 adc -Simulated -Frames 3
#>
param(
    [ValidateSet('session', 'imu', 'angle', 'adc',
                 'gate_drivers', 'rotor_observer', 'thermal_observer')]
    [string]$Name,
    [string]$Port = 'COM4',
    [switch]$Simulated,
    [int]$Frames = 0
)

$ErrorActionPreference = 'Continue'

# The frames are box drawing; a console left on the OEM codepage prints
# them as mojibake. Same line board_chat.ps1 runs, for the same reason.
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

# The session first, because it is what most runs want and what the number
# keys land on. Script $null marks it: it is not one of terminal/, it IS
# the session itself.
$Views = [ordered]@{
    'session' = @{ Script = $null
                 What   = 'one dash: analog, thermals, bridges, DIO, IMU, angle' }
    'imu'   = @{ Script = 'imu.ps1'
                 What   = 'board attitude, drawn from the STL the IMU turns' }
    'angle' = @{ Script = 'angle.ps1'
                 What   = 'shaft angle, the magnet and the air gap' }
    'adc'   = @{ Script = 'adc.ps1'
                 What   = 'every analog channel, on a meter bridge' }
    'gate_drivers' = @{ Script = 'gate_drivers.ps1'
                 What   = 'the gate drivers: six signals, current, a burst' }
    'thermal_observer' = @{ Script = 'thermal_observer.ps1'
                 What   = 'where the heat sits, drawn on the board itself' }
    'chat'  = @{ Script = $null; Chat = 'local'
                 What   = 'CCC - the coaxial 63100 chat client' }
    'claude' = @{ Script = $null; Chat = 'anthropic'
                 What   = 'claude with the board over MCP' }
    # Past the list on the front page like claude: MOTOR CONTROLLER's
    # second answer. Its own code is gate_drivers', by position.
    'rotor_observer' = @{ Script = 'rotor_observer.ps1'
                 What   = 'the rotor observer: the drive on the model or the converters' }
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
    # The reference screens: a solid title bar, every entry in one outlined
    # box, the keys spelled out on the bottom line. The console's 16
    # colours stand in for the motif - Cyan for neon, DarkGray for ash.
    Write-Host ''
    Write-Host ' COAXIAL 63100 // LIVE VIEWS                                               ' `
        -ForegroundColor Black -BackgroundColor DarkCyan
    Write-Host ''

    # The name column is as wide as the longest name, not a number typed in
    # once: 'capture' is exactly 7 and 'gate_drivers' is 12, so a fixed 7 ran
    # both of them straight into their description.
    $keys = @($Views.Keys)
    $width = ($keys | Measure-Object -Property Length -Maximum).Maximum + 3
    $name = '{0,-' + $width + '}'
    $inner = 74

    Write-Host (' ┌' + ('─' * $inner) + '┐') -ForegroundColor DarkGray
    for ($i = 0; $i -lt $keys.Count; $i++) {
        $key = $keys[$i]
        Write-Host ' │' -NoNewline -ForegroundColor DarkGray
        Write-Host ('  {0}  ' -f ($i + 1)) -NoNewline -ForegroundColor Cyan
        Write-Host ($name -f $key) -NoNewline -ForegroundColor White
        $what = $Views[$key].What
        $lead = 5 + $width
        if ($lead + $what.Length -gt $inner) {
            $what = $what.Substring(0, $inner - $lead)
        }
        Write-Host $what -NoNewline -ForegroundColor DarkGray
        Write-Host ((' ' * [Math]::Max(0, $inner - $lead - $what.Length)) + '│') `
            -ForegroundColor DarkGray
    }
    Write-Host (' └' + ('─' * $inner) + '┘') -ForegroundColor DarkGray

    Write-Host ''
    Write-Host (' 1-{0}' -f $keys.Count) -NoNewline -ForegroundColor Cyan
    Write-Host ': SELECT  |  ' -NoNewline -ForegroundColor DarkGray
    Write-Host 'Q' -NoNewline -ForegroundColor Cyan
    Write-Host ': EXIT' -ForegroundColor DarkGray
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
# LEAVING THE MENU IS AN EXIT PATH TOO. A view cleans up after itself and
# says so, but quitting the chooser used to be a bare `exit 0` - and a view
# that was killed, or a session that ended badly, leaves a stage armed with
# nothing on screen to say it. One port open on the way out buys the right
# to say `nothing was left running` and mean it.
function Close-Session {
    Push-Location (Join-Path $PSScriptRoot 'host')
    $argv = @('tools/show_session.py', '--leave', '--port', $Port)
    if ($Simulated) { $argv += '--simulated' }
    & python @argv
    Pop-Location
}

$asked = $Name
$code = 0

# The views a second question answers with. ESC from one of them reopens
# the front page ON that question, with the view lit - not at the top of
# the list, three keys away from where the reader was. Not $Asked:
# PowerShell names are case-insensitive, and $Asked clobbered $asked,
# the -Name parameter - $view became this list, the front page was
# skipped and the chat branch opened on every start.
$SubViews = @('gate_drivers', 'rotor_observer', 'chat', 'claude')
$view = $null
$from = $null

do {
    $from = $view
    $view = $asked
    $asked = $null

    while (-not $view) {
        # The front page is tools/menu.py - the rotating board and the
        # access list - and the choice comes back in the EXIT CODE, because
        # capturing stdout would turn the page's console into a pipe.
        Push-Location (Join-Path $PSScriptRoot 'host')
        $page = @('-X', 'utf8', 'tools/menu.py', '--port', $Port)
        # The front page wears the chip a view will wear. Without this it
        # probed for a board, found one, and said LIVE over a menu whose
        # every view was about to open the stand-in.
        if ($Simulated) { $page += '--simulated' }
        if ($from -and $SubViews -contains $from) { $page += @('--open', $from) }
        $from = $null
        & python @page
        $picked = $LASTEXITCODE
        Pop-Location
        if ($picked -lt 101) {
            Close-Session
            exit 0
        }
        $names = @($Views.Keys)
        $view = $names[$picked - 101]
        if ($null -eq $view) {
            Close-Session
            exit 0
        }
    }

    # INLINE, and not behind a function returning the code. Both ways of
    # getting the code out of a function redirect the child: assigning its
    # output captures the frames, and piping to Out-Host makes stdout a pipe
    # - so `sys.stdout.isatty()` went false, the session stopped clearing the
    # screen and repainted whole frames instead of the rows that changed.
    if ($Views[$view].Chat) {
        # A chat is its own process like every demo, and coming back from
        # it is ALWAYS the menu: quitting a chat must not quit the
        # terminal it was picked from.
        Push-Location $PSScriptRoot
        if ($Views[$view].Chat -eq 'anthropic') {
            # The same page as CCC, claude -p answering each turn - the
            # view runs it from the repo root, where .mcp.json wires the
            # coaxial MCP server to it.
            Push-Location (Join-Path $PSScriptRoot 'host')
            & python -X utf8 tools/show_chat.py --claude --port $Port
            Pop-Location
        } else {
            # The CCC page: the same Chat the bench prompt drives,
            # drawn inside the stage - a terminal in the terminal.
            Push-Location (Join-Path $PSScriptRoot 'host')
            $argv = @('-X', 'utf8', 'tools/show_chat.py', '--port', $Port)
            if ($Simulated) { $argv += '--simulated' }
            & python @argv
            Pop-Location
        }
        $said = $LASTEXITCODE
        Pop-Location
        if ($said -ne 0 -and $said -ne $TO_MENU) {
            Write-Host ''
            Write-Host ('  {0} exited {1} - its last lines above say why' `
                -f $view, $said) -ForegroundColor DarkYellow
            Write-Host '  any key for the menu ' -NoNewline -ForegroundColor DarkGray
            try { [void][Console]::ReadKey($true) } catch { }
            Write-Host ''
        }
        $code = $TO_MENU
        continue
    }

    if (-not $Views[$view].Script) {
        Push-Location (Join-Path $PSScriptRoot 'host')
        $argv = @('tools/show_session.py', '--port', $Port)
        if ($Simulated) { $argv += '--simulated' }
        if ($Frames -gt 0) { $argv += @('--frames', $Frames) }
        & python @argv
        $code = $LASTEXITCODE
        Pop-Location
        if ($code -ne 0 -and $code -ne $TO_MENU) {
            Write-Host ''
            Write-Host ('  {0} exited {1} - its last lines above say why' `
                -f $view, $code) -ForegroundColor DarkYellow
            Write-Host '  any key for the menu ' -NoNewline -ForegroundColor DarkGray
            try { [void][Console]::ReadKey($true) } catch { }
            Write-Host ''
            $code = $TO_MENU
        }
        continue
    }

    $call = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File',
              (Join-Path $PSScriptRoot (Join-Path 'terminal' $Views[$view].Script)),
              '-Port', $Port)
    if ($Simulated) { $call += '-Simulated' }
    if ($Frames -gt 0) { $call += @('-Frames', [string]$Frames) }

    & powershell @call
    $code = $LASTEXITCODE

    # A view that FAILED comes back here, with its refusal still on screen -
    # ending the whole chooser over one view's preflight turned 'AFE was
    # off' into 'the menu quit on me'. Only Q (0) and a dead console leave.
    if ($code -ne 0 -and $code -ne $TO_MENU) {
        Write-Host ''
        Write-Host ('  {0} exited {1} - its last lines above say why' `
            -f $view, $code) -ForegroundColor DarkYellow
        Write-Host '  any key for the menu ' -NoNewline -ForegroundColor DarkGray
        try { [void][Console]::ReadKey($true) } catch { }
        Write-Host ''
        $code = $TO_MENU
    }
} while ($code -eq $TO_MENU)

# The other way out: a view closed with Q rather than coming back here. It
# put its own things back and said so - this checks nothing was missed, and
# says that too, so both exits read the same.
Close-Session
exit $code
