<#
.SYNOPSIS
    One window with the model and the board in it.

        . .\env.ps1 ; .\board_prompt.ps1
        .\board_prompt.ps1 -NewWindow          the same, in its own window
        .\board_prompt.ps1 -Plain              plain ollama chat, no board, no tools
        .\board_prompt.ps1 -Simulated          board tools work, against invented data
        .\board_prompt.ps1 -AutodetectComport  tries -Port, then every other COM port

.DESCRIPTION
    You asked for a terminal that talks to gemma. `ollama run gemma4:12b` is
    that terminal and it is the wrong one: it is a model with no board, so every
    answer about this hardware is a guess dressed as a fact. The prompt worth
    opening is host/dbg.py --repl - the same local model, with the eleven board
    tools, /py against a live Session and /sh for a build, and a token meter on
    every turn. This script is the preflight that stands in front of it.

    Four things it does that typing `python dbg.py --repl` does not:

      * env.ps1, quietly, so cube-cmake, the programmer and ollama itself are on
        PATH in this shell. The ollama installer only reaches shells opened
        after it ran, which is never the one you are standing in.
      * Starts the daemon if nothing answers on 11434, instead of letting the
        first question fail thirty seconds in.
      * Loads the model before the prompt opens. gemma4:12b is 8 GB into VRAM
        and that wait belongs here, visibly, not inside your first question. It
        is pinned there for -KeepAlive afterwards, so question two is not
        another cold start.
      * Says which COM port answered. A prompt that silently has no board is the
        one way this tool wastes a real afternoon: you cannot tell a model that
        cannot reach the hardware from a model that is confidently wrong.

    Nothing here judges a measurement, and nothing here is a limit. It is a
    launcher: the board stays the dumb slave it is, and every number you see
    still comes from the board itself.

.PARAMETER Model
    Ollama tag. Left alone, this machine decides: cores, RAM and the size of the
    graphics card are measured and the largest tools-capable model that fits the
    card whole is chosen, then pulled if it is not here yet. See
    host/coaxial_ollama/capability.py, or ask it directly with

        python -m coaxial_ollama.capability

    Local only either way - the Python refuses a :cloud tag and a daemon on
    another machine unless you pass --allow-remote to it yourself.

.PARAMETER Prefer
    What the automatic choice optimises for: 'speed' takes the largest model
    that fits the card whole, 'capability' allows a bigger one to spill onto the
    CPU, which measured about five times slower per token.

.PARAMETER Port
    The board's VCP. COM4 on this bench. Ignored if -AutodetectComport finds
    one, and if -Simulated is given, never even looked at.

.PARAMETER AutodetectComport
    Try -Port first, then every other COM port Windows reports, oldest-
    enumerated first, opening each just long enough to see whether this board
    answers on it - a few seconds per port it does not. For "which port did I
    just plug the programmer into" instead of checking Device Manager. Silent
    when it finds nothing: the ordinary -Port warning below still fires, and
    -Port is whatever it was, unchanged.

.PARAMETER Simulated
    Run against coaxial.simulated instead of a real port - no cable, no COM
    port, every board tool still answers, with invented values instead of
    measured ones. -Port and -AutodetectComport are pointless with this and
    are skipped rather than acted on. For a machine with nothing plugged in
    that still wants to see the tools actually work, not just be told they
    would fail - which is what -NoBoard gives you instead.

.PARAMETER Tools
    Which tool subset the model gets: read, code, pins, build, all or none. The
    list is re-sent every turn, so it is the cost that scales with the
    conversation. `build` is board_info, docs and run_command - the model's
    only path to host/tools/build_and_flash.py, which is in turn the only path
    to cube-cmake and STM32_Programmer_CLI. Pair it with -Confirm.

.PARAMETER Ask
    One question, printed, then exit. No prompt loop.

.PARAMETER Confirm
    Ask before every state change - a pin write, run_python, run_command. Off
    by default. Matters most with -Tools build: without it, the model builds
    and flashes the real board the moment it decides to, no human in the loop.

.PARAMETER NoBoard
    Open the prompt with the board tools stubbed out - for a machine with
    nothing plugged in.

.PARAMETER Plain
    `ollama run` instead: a bare chat with the model, no tools and no board.
    Warmed and checked the same way.

.PARAMETER NewWindow
    Relaunch in a new PowerShell window and return. Off by default, because
    output you can scroll back through beside your build log is usually what you
    actually wanted.

.PARAMETER KeepAlive
    How long ollama holds the model in memory after the last turn.

.PARAMETER KeepOthers
    Leave models that are already resident where they are. By default this
    script unloads them before loading its own: a session that was killed, one
    that exited with -Hold, or a plain `ollama run` in another window all leave
    weights on the card, and the second model is what turns a 16 GB card into a
    500 reading `cudaMalloc failed`. The model this run wants is never unloaded
    when it is already there - that is warm, not stale.

.PARAMETER Hold
    Leave the model resident when this script exits. By default the weights are
    handed back the moment the prompt closes: measured here, a finished session
    left 9.69 GB on a 16 GB card for another 27 minutes at 1 % utilisation,
    which is a cache nobody is going to hit and a desktop with 3.8 GB to work
    in. Use -Hold when the next question really is imminent.

.PARAMETER Reserve
    VRAM in GB to hold back for the desktop, overriding what capability.py works
    out on its own. Raise it if the screens stutter while the model answers: on
    a 16 GB card, 8 steps the choice down from a 14B to a 12B and leaves about
    5.5 GB free with the desktop's own 2.6 GB already counted. It is also
    settable once per machine, for every entry point, with the environment
    variable COAXIAL_VRAM_RESERVE_GB.

.PARAMETER Normal
    Leave ollama at normal process priority. By default this script drops the
    daemon to BelowNormal while it is being driven from here, because a bench PC
    is also the machine you are reading the schematic on.

.PARAMETER NoTune
    Leave the daemon's own settings alone. By default this script makes sure
    ollama is running with llama-server's prompt cache off and its context
    checkpoints capped, restarting the daemon once if it has to - see
    $DaemonTuning in board_prompt/Tuning.ps1 for the measurements. Without
    that, a bench session of eight or ten questions reliably kills the model
    runner with std::bad_alloc partway through and reloads eight gigabytes
    mid-answer. Use this to reproduce that, or when something else on the
    machine owns the daemon.

.PARAMETER NumCtx
    Context window. It is passed to the preload as well as to the prompt, and
    that is not a detail: load the model at one context size and question it at
    another and the daemon reloads it, so the wait this script exists to make
    visible happens again inside your first question.
#>
[CmdletBinding()]
param(
    [string]$Model,
    [ValidateSet('speed', 'capability')]
    [string]$Prefer = 'speed',
    [string]$Port = 'COM4',
    [switch]$AutodetectComport,
    [switch]$Simulated,
    [ValidateSet('read', 'code', 'pins', 'build', 'all', 'none')]
    [string]$Tools = 'code',
    [string]$Ask,
    [switch]$Confirm,
    [switch]$NoBoard,
    [switch]$Plain,
    [switch]$NewWindow,
    [string]$KeepAlive = '30m',
    [int]$NumCtx = 8192,
    [double]$Reserve = 0,
    [switch]$Normal,
    [switch]$Hold,
    [switch]$KeepOthers,
    [switch]$NoTune
)

$ErrorActionPreference = 'Continue'
$Root = $PSScriptRoot
$Api = 'http://localhost:11434'

# board_prompt/*.ps1 - Say first by convention, though load order does not
# actually matter: nothing in any of these files runs until well after all
# five are dot-sourced, and PowerShell resolves a function call by name at
# call time, not at definition time.
foreach ($part in 'Say', 'Tuning', 'ComPort', 'Ollama', 'ModelChoice', 'Relaunch') {
    . (Join-Path (Join-Path $Root 'board_prompt') "$part.ps1")
}

# UTF-8, console and all: dbg.py's prompt has a robot and a pager either side
# of its spinner, and Python auto-detects its own stdout encoding from this
# console's codepage at startup, not from anything Python-side. Left as the
# legacy default (cp1252 on this bench), forcing Python's side to UTF-8
# without also changing this would turn every multi-byte character it writes
# into mojibake instead of the plain UnicodeEncodeError it started as - which
# is why dbg.py's own _printable() deliberately never forces this. Doing it
# here instead is safe precisely because it is scoped to this one console:
# the task that launches this script passes -NoProfile, so nothing here
# reaches a shell the user already had open, and a bare `python dbg.py` run
# from an unrelated console is untouched and keeps falling back to ASCII.
try {
    [Console]::OutputEncoding = [System.Text.Encoding]::UTF8
} catch {
    # Not fatal - the robot and pager fall back to ASCII either way, and
    # _printable()'s errors='replace' still covers whatever text can't be
    # helped by this.
}

if ($NoBoard -and $Simulated) {
    Write-Host '-NoBoard and -Simulated contradict each other: one stubs the' -ForegroundColor Red
    Write-Host 'board tools out, the other makes them work against invented' -ForegroundColor Red
    Write-Host 'data. Pick one.' -ForegroundColor Red
    exit 1
}

# ---- the new window, if that is what was asked -----------------------------

if ($NewWindow) {
    # Rebuild the call rather than forwarding $args: a switch is '-Name' and a
    # value is two elements, and getting that wrong silently drops a parameter.
    $forward = @('-NoExit', '-ExecutionPolicy', 'Bypass',
                 '-File', (Format-Argument $PSCommandPath))
    foreach ($name in $PSBoundParameters.Keys) {
        if ($name -eq 'NewWindow') { continue }
        $value = $PSBoundParameters[$name]
        if ($value -is [switch]) {
            if ($value.IsPresent) { $forward += ('-' + $name) }
        } else {
            $forward += @(('-' + $name), (Format-Argument ([string]$value)))
        }
    }
    Start-Process -FilePath 'powershell.exe' -ArgumentList ($forward -join ' ') `
                  -WorkingDirectory $Root
    return
}

# ---- preflight -------------------------------------------------------------

Write-Host ''
Write-Host 'coaxial_63100 bench prompt' -ForegroundColor White

. (Join-Path $Root 'env.ps1') -Quiet
$ollama = Get-Command 'ollama' -ErrorAction SilentlyContinue
if ($null -eq $ollama) {
    Say 'fail' 'ollama' 'not installed - run .\setup.ps1'
    exit 1
}
Say 'ok' 'ollama' $ollama.Source

$tags = Get-Tags
$startedHere = $false
if ($null -eq $tags) {
    $startedHere = $true
    Say 'wait' 'ollama serve' 'nothing on 11434 - starting the daemon'
    # The daemon inherits this shell's environment, so the tuning has to be in
    # it before the process starts - see $DaemonTuning in board_prompt/Tuning.ps1
    # for what each variable is worth and what was measured without it.
    Set-DaemonEnvironment | Out-Null
    Start-Process -FilePath $ollama.Source -ArgumentList 'serve' -WindowStyle Hidden `
                  -ErrorAction SilentlyContinue
    $tags = Get-Tags -Tries 10
}
if ($null -eq $tags) {
    Say 'fail' 'ollama serve' 'no answer on 11434'
    exit 1
}

# A daemon that was already up is the ordinary case, and it kept whatever
# environment it was started with at login - which is the untuned one. This is
# the only thing that can fix that, and it costs one restart, once.
Initialize-Daemon -Exe $ollama.Source -JustStarted:$startedHere

# Stem matching, as everywhere else here: `gemma4` should find gemma4:12b. Cloud
# tags are excluded on purpose - the Python refuses them, so a launcher that
# accepted one would only move the error later.
$names = @()
if ($null -ne $tags.models) {
    $names = $tags.models | ForEach-Object { $_.name } |
             Where-Object { ($_ -split ':')[-1] -ne 'cloud' }
}
# Nobody said which model, so the machine decides - and the same measurement
# decides how many layers go on the card.
$layers = $null
if (-not $Model) {
    $choice = Get-Choice
    $Model = $choice.model
    $layers = $choice.num_gpu
    if ($choice.machine) {
        Say 'ok' 'this machine' ('{0} cores, {1:n0} GB RAM, {2:n0} GB VRAM' `
            -f $choice.machine.cores, $choice.machine.ram_gb, $choice.machine.vram_gb)
    }
    Say 'ok' 'model choice' ('{0}  ({1})' -f $Model, $choice.why)
}

$stem = ($Model -split ':')[0]
$resolved = $names | Where-Object { $_ -eq $Model } | Select-Object -First 1
if ($null -eq $resolved) {
    $resolved = $names | Where-Object { ($_ -split ':')[0] -eq $stem } | Select-Object -First 1
}
if ($null -eq $resolved) {
    # Pull it rather than printing the command and quitting. This script exists
    # so a question can be asked without a detour, and "run this and come back"
    # is a detour. Several GB the first time on a given machine, once.
    Say 'wait' 'model' ("$Model is not here yet - pulling it")
    Write-Host ''
    & $ollama.Source pull $Model
    Write-Host ''
    if ($LASTEXITCODE -ne 0) {
        Say 'fail' 'model' ("ollama pull $Model exited $LASTEXITCODE")
        exit 1
    }
    $tags = Get-Tags -Tries 5
    $names = @()
    if ($null -ne $tags -and $null -ne $tags.models) {
        $names = $tags.models | ForEach-Object { $_.name } |
                 Where-Object { ($_ -split ':')[-1] -ne 'cloud' }
    }
    $resolved = $names | Where-Object { ($_ -split ':')[0] -eq $stem } | Select-Object -First 1
    if ($null -eq $resolved) {
        Say 'fail' 'model' ("pulled, but $Model is still not in the list")
        exit 1
    }
    Say 'ok' 'model' ("pulled $resolved")
}
$Model = $resolved

# Anything left on the card from a killed window, a -Hold, or somebody's
# `ollama run` goes now - before this run adds its own.
Clear-Resident -Except $Model

# Before the load, not after: warming the file cache only helps if it
# happens ahead of the read that actually needs it.
Invoke-Warm -Tag $Model

# An empty prompt loads the weights and generates nothing. Doing it here means
# the 8 GB wait is visible and timed, instead of hiding inside question one.
$clock = [Diagnostics.Stopwatch]::StartNew()
try {
    # options, and not just the tag. Without num_ctx the daemon loads the
    # model's own default context - 32k on qwen2.5, 128k on llama3.1 - and
    # answers 500 trying to allocate the KV cache for it. Measured here, twice:
    # once in Python (see Ollama.preload) and once from this script.
    $options = @{ num_ctx = $NumCtx; temperature = 0.0 }
    if ($null -ne $layers) { $options['num_gpu'] = $layers }
    $body = @{ model = $Model; prompt = ''; stream = $false;
               keep_alive = $KeepAlive; options = $options } |
            ConvertTo-Json
    Invoke-RestMethod -Uri ($Api + '/api/generate') -Method Post -Body $body `
                      -ContentType 'application/json' -TimeoutSec 600 | Out-Null
    Say 'ok' 'model' ('{0}  loaded in {1:n1} s, held {2}' -f $Model, $clock.Elapsed.TotalSeconds, $KeepAlive)
    try {
        $free = (nvidia-smi --query-gpu=memory.total,memory.used --format=csv,noheader,nounits) -split ','
        Say 'ok' 'card' ('{0:n1} GB free of {1:n1}' `
            -f (([double]$free[0] - [double]$free[1]) / 1024), ([double]$free[0] / 1024))
    } catch {
        # No nvidia-smi is not a problem worth a line of its own.
    }
} catch {
    Say 'warn' 'model' ("could not preload: " + $_.Exception.Message)
}

if ($NoBoard) {
    Say 'warn' 'board' '--no-board: tools are stubbed, nothing is measured'
} elseif ($Simulated) {
    Say 'warn' 'board' '--simulated: no port opened, every reading is invented'
} else {
    if ($AutodetectComport) {
        $found = Find-BoardPort -PreferredPort $Port -HostDir (Join-Path $Root 'host')
        if ($found) { $Port = $found }
    }
    $ports = @()
    try { $ports = [System.IO.Ports.SerialPort]::GetPortNames() } catch { $ports = @() }
    # A port name in the list is not a board answering on it - measured, COM4
    # enumerated while the board stayed silent. The session probes for real and
    # falls back to a simulated board; its prompt tag is what actually says
    # which one you got, so neither line here claims more than it knows.
    if ($ports -contains $Port) {
        Say 'ok' 'board' ("$Port listed (ports: " + ($ports -join ', ') +
                          '). The prompt tag says which board answered.')
    } else {
        Say 'warn' 'board' ("no $Port. Present: " + (($ports -join ', ') -replace '^$', 'none') +
                            '. The session falls back to a simulated board - the prompt says (Simulated).')
    }
}

# ---- who gets the machine --------------------------------------------------

if (-not $Normal) {
    # BelowNormal on the daemon, and be honest about what that buys: it governs
    # CPU scheduling, not the GPU's. With the model wholly on the card the work
    # that this deprioritises is tokenisation, sampling and the serial I/O, not
    # the matrix multiplies - so it helps the desktop stay responsive while a
    # question is being prepared and answered, and does nothing about the card
    # being busy. The other half of that problem is VRAM headroom: -Reserve.
    $lowered = @()
    foreach ($proc in (Get-Process -Name 'ollama*' -ErrorAction SilentlyContinue)) {
        try {
            $proc.PriorityClass = [Diagnostics.ProcessPriorityClass]::BelowNormal
            $lowered += $proc.Name
        } catch {
            # A process started by another user, or already gone. Not worth a
            # failure: the prompt below works either way.
        }
    }
    if ($lowered.Count -gt 0) {
        Say 'ok' 'priority' (($lowered | Sort-Object -Unique) -join ', ') 
    }
}

# ---- the prompt itself -----------------------------------------------------

# Set when a prompt loop was opened, so the exit path knows whether the card is
# being left behind by somebody who has finished, or by a one-shot question that
# may well be followed by another in ten seconds.
$script:Interactive = $false

Push-Location (Join-Path $Root 'host')
try {
    if ($Plain) {
        Write-Host ''
        Write-Host '  plain chat: no tools, no board. /bye leaves.' -ForegroundColor DarkGray
        Write-Host ''
        $script:Interactive = $true
        & $ollama.Source run $Model
        return
    }

    $call = @('dbg.py', '-m', $Model, '-t', $Tools,
              '--num-ctx', [string]$NumCtx, '--keep-alive', $KeepAlive)
    if ($Confirm) { $call += '--confirm' }
    if ($Simulated) { $call += '--simulated' }
    elseif ($NoBoard) { $call += '--no-board' }
    else { $call += @('--port', $Port) }
    # A split model needs its layer count on every call, or the daemon reloads
    # it whole on the first question and the preflight above bought nothing.
    if ($null -ne $layers) { $call += @('--num-gpu', [string]$layers) }

    if ($Ask) {
        & python @call $Ask
        return
    }

    Write-Host ''
    Write-Host ('  tools: ' + $Tools + '   /py CODE runs against the board, /sh runs a program,') -ForegroundColor DarkGray
    Write-Host '  both cost no tokens. /tools NAME repriced, /ctx, /clear, /q to leave.' -ForegroundColor DarkGray
    Write-Host ''
    $script:Interactive = $true
    & python @call --repl
} finally {
    Pop-Location

    # Leaving the prompt hands the card back, at once. The keep_alive that made
    # turn nine quick has no further job once the window is closed, and until
    # this existed it parked the whole model on the GPU for the rest of its half
    # hour - measured at 9.69 GB of 16, at 1 % utilisation, with the desktop
    # left 3.8 GB to work in. A reload costs about seven seconds next time, and
    # only if there is a next time.
    #
    # A one-shot -Ask is deliberately not unloaded here: it is usually one of
    # several, and dbg.py already holds it for two minutes rather than thirty.
    if ($script:Interactive -and -not $Hold) {
        try {
            $body = @{ model = $Model; prompt = ''; keep_alive = 0 } | ConvertTo-Json
            Invoke-RestMethod -Uri ($Api + '/api/generate') -Method Post -Body $body `
                              -ContentType 'application/json' -TimeoutSec 30 | Out-Null
            Say 'ok' 'released' ($Model + ' unloaded - -Hold keeps it resident')
        } catch {
            Say 'warn' 'released' ('could not unload: ' + $_.Exception.Message)
        }
    }
}
