<#
.SYNOPSIS
    One window with the model and the board in it.

        . .\env.ps1 ; .\board_prompt.ps1
        .\board_prompt.ps1 -NewWindow          the same, in its own window
        .\board_prompt.ps1 -Plain              plain ollama chat, no board, no tools

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
    The board's VCP. COM4 on this bench.

.PARAMETER Tools
    Which tool subset the model gets: read, code, pins, all or none. The list is
    re-sent every turn, so it is the cost that scales with the conversation.

.PARAMETER Ask
    One question, printed, then exit. No prompt loop.

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
    [ValidateSet('read', 'code', 'pins', 'all', 'none')]
    [string]$Tools = 'code',
    [string]$Ask,
    [switch]$NoBoard,
    [switch]$Plain,
    [switch]$NewWindow,
    [string]$KeepAlive = '30m',
    [int]$NumCtx = 8192,
    [double]$Reserve = 0,
    [switch]$Normal
)

$ErrorActionPreference = 'Continue'
$Root = $PSScriptRoot
$Api = 'http://localhost:11434'

function Say {
    param([string]$State, [string]$Text, [string]$Detail = '')
    $colour = 'Gray'
    if ($State -eq 'ok')   { $colour = 'Green' }
    if ($State -eq 'wait') { $colour = 'Cyan' }
    if ($State -eq 'warn') { $colour = 'Yellow' }
    if ($State -eq 'fail') { $colour = 'Red' }
    Write-Host ('  {0,-6}' -f $State) -ForegroundColor $colour -NoNewline
    Write-Host ('{0,-22} ' -f $Text) -NoNewline
    Write-Host $Detail -ForegroundColor DarkGray
}

function Get-Tags {
    param([int]$Tries = 1)
    for ($i = 0; $i -lt $Tries; $i++) {
        try {
            return (Invoke-RestMethod -Uri ($Api + '/api/tags') -TimeoutSec 5)
        } catch {
            if ($i -lt ($Tries - 1)) { Start-Sleep -Seconds 2 }
        }
    }
    return $null
}

function Get-Choice {
    <#  Which model this machine should run, and how much of it fits the card.

        capability.py owns the answer - the same answer setup.ps1 and
        `dbg -m auto` get - so a bench does not end up with three opinions
        about which tag is right. A machine where the probe fails is not a
        reason to stop: fall back to the tag this bench was built on and say
        so.  #>

    Push-Location (Join-Path $Root 'host')
    try {
        $argv = @('-m', 'coaxial_ollama.capability', '--json', '--prefer', $Prefer)
        if ($Reserve -gt 0) { $argv += @('--reserve-gb', [string]$Reserve) }
        $json = (& python @argv) -join ''
    } catch {
        $json = ''
    } finally {
        Pop-Location
    }
    if ([string]::IsNullOrWhiteSpace($json)) {
        Say 'warn' 'model choice' 'could not measure this machine - falling back'
        return @{ model = 'gemma4:12b'; num_gpu = $null; why = 'fallback' }
    }
    try {
        $picked = $json | ConvertFrom-Json
    } catch {
        Say 'warn' 'model choice' 'capability.py said something unreadable'
        return @{ model = 'gemma4:12b'; num_gpu = $null; why = 'fallback' }
    }
    return @{ model = $picked.model
              num_gpu = $picked.options.num_gpu
              why = $picked.why
              machine = $picked.machine }
}

# ---- the new window, if that is what was asked -----------------------------

if ($NewWindow) {
    # Rebuild the call rather than forwarding $args: a switch is '-Name' and a
    # value is two elements, and getting that wrong silently drops a parameter.
    $forward = @('-NoExit', '-ExecutionPolicy', 'Bypass', '-File', $PSCommandPath)
    foreach ($name in $PSBoundParameters.Keys) {
        if ($name -eq 'NewWindow') { continue }
        $value = $PSBoundParameters[$name]
        if ($value -is [switch]) {
            if ($value.IsPresent) { $forward += ('-' + $name) }
        } else {
            $forward += @(('-' + $name), [string]$value)
        }
    }
    Start-Process -FilePath 'powershell.exe' -ArgumentList $forward `
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
if ($null -eq $tags) {
    Say 'wait' 'ollama serve' 'nothing on 11434 - starting the daemon'
    # One model, one context. Two of either is how a 16 GB card ends up asked
    # for two copies of the weights at once - measured here as a 500 from the
    # daemon, 'cudaMalloc failed', with nothing obviously wrong on either side.
    # Only reachable when this script starts the daemon; an already-running one
    # keeps the environment it was started with.
    $env:OLLAMA_MAX_LOADED_MODELS = '1'
    $env:OLLAMA_NUM_PARALLEL = '1'
    Start-Process -FilePath $ollama.Source -ArgumentList 'serve' -WindowStyle Hidden `
                  -ErrorAction SilentlyContinue
    $tags = Get-Tags -Tries 10
}
if ($null -eq $tags) {
    Say 'fail' 'ollama serve' 'no answer on 11434'
    exit 1
}

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
} catch {
    Say 'warn' 'model' ("could not preload: " + $_.Exception.Message)
}

if ($NoBoard) {
    Say 'warn' 'board' '--no-board: tools are stubbed, nothing is measured'
} else {
    $ports = @()
    try { $ports = [System.IO.Ports.SerialPort]::GetPortNames() } catch { $ports = @() }
    if ($ports -contains $Port) {
        Say 'ok' 'board' ("$Port  (ports: " + ($ports -join ', ') + ')')
    } else {
        Say 'warn' 'board' ("no $Port. Present: " + (($ports -join ', ') -replace '^$', 'none') +
                            '. Questions needing the board will fail.')
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

Push-Location (Join-Path $Root 'host')
try {
    if ($Plain) {
        Write-Host ''
        Write-Host '  plain chat: no tools, no board. /bye leaves.' -ForegroundColor DarkGray
        Write-Host ''
        & $ollama.Source run $Model
        return
    }

    $call = @('dbg.py', '-m', $Model, '-t', $Tools, '--port', $Port,
              '--num-ctx', [string]$NumCtx, '--keep-alive', $KeepAlive)
    # A split model needs its layer count on every call, or the daemon reloads
    # it whole on the first question and the preflight above bought nothing.
    if ($null -ne $layers) { $call += @('--num-gpu', [string]$layers) }
    if ($NoBoard) { $call += '--no-board' }

    if ($Ask) {
        & python @call $Ask
        return
    }

    Write-Host ''
    Write-Host ('  tools: ' + $Tools + '   /py CODE runs against the board, /sh runs a program,') -ForegroundColor DarkGray
    Write-Host '  both cost no tokens. /tools NAME repriced, /ctx, /clear, /q to leave.' -ForegroundColor DarkGray
    Write-Host ''
    & python @call --repl
} finally {
    Pop-Location
}
