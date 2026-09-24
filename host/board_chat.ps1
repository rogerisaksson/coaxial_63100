<#
.SYNOPSIS
    One window with the model and the board in it.
.DESCRIPTION
    You asked for a terminal that talks to gemma.
.PARAMETER Model
    Ollama tag.
.PARAMETER Prefer
    What the automatic choice optimises for: 'speed' takes the largest model
    that fits the card whole, 'capability' allows a bigger one to spill onto
    the CPU, which measured about five times slower per token.
.PARAMETER Port
    The board's VCP.
.PARAMETER AutodetectComport
    Try -Port first, then every other COM port Windows reports, oldest-
    enumerated first, opening each just long enough to see whether this
    board answers on it - a few seconds per port it does not.
.PARAMETER Simulated
    Run against coaxial.simulated instead of a real port - no cable, no COM
    port, every board tool still answers, with invented values instead of
    measured ones.
.PARAMETER Tools
    Which tool subset the model gets: read, code, pins, build, all or none.
.PARAMETER Ask
    One question, printed, then exit.
.PARAMETER Confirm
    Ask before every state change - a pin write, run_python, run_command.
.PARAMETER NoBoard
    Open the prompt with the board tools stubbed out - for a machine with
    nothing plugged in.
.PARAMETER Plain
    `ollama run` instead: a bare chat with the model, no tools and no board.
.PARAMETER NewWindow
    Relaunch in a new PowerShell window and return.
.PARAMETER KeepAlive
    How long ollama holds the model in memory after the last turn.
.PARAMETER KeepOthers
    Leave models that are already resident where they are.
.PARAMETER Hold
    Leave the model resident when this script exits.
.PARAMETER Reserve
    VRAM in GB to hold back for the desktop, overriding what capability.py
    works out on its own.
.PARAMETER Normal
    Leave ollama at normal process priority.
.PARAMETER NoTune
    Leave the daemon's own settings alone.
.PARAMETER NumCtx
    Context window.
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
    [string[]]$Ask,
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

# board_chat/*.ps1 - Say first by convention, though load order does not
# actually matter: nothing in any of these files runs until well after all five
# are dot-sourced, and PowerShell resolves a function call by name at call
# time, not at definition time.
foreach ($part in 'Say', 'Tuning', 'ComPort', 'Ollama', 'ModelChoice', 'Relaunch') {
    . (Join-Path (Join-Path $Root 'board_chat') "$part.ps1")
}

# UTF-8, console and all: dbg.py's prompt has a robot and a pager either side
# of its spinner, and Python auto-detects its own stdout encoding from this
# console's codepage at startup, not from anything Python-side.
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

# env.ps1 stays at the repo root, one level above this script's host/.
. (Join-Path (Split-Path $Root) 'env.ps1') -Quiet
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
    # it before the process starts - see $DaemonTuning in board_chat/Tuning.ps1
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
# environment it was started with at login - which is the untuned one.
Initialize-Daemon -Exe $ollama.Source -JustStarted:$startedHere

# Stem matching, as everywhere else here: `gemma4` should find gemma4:12b.
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
    if ($choice.host) {
        Say 'ok' 'this machine' ('{0} cores, {1:n0} GB RAM, {2:n0} GB VRAM' `
            -f $choice.host.cores, $choice.host.ram_gb, $choice.host.vram_gb)
    }
    Say 'ok' 'model choice' ('{0}  ({1})' -f $Model, $choice.why)
}

$stem = ($Model -split ':')[0]
$resolved = $names | Where-Object { $_ -eq $Model } | Select-Object -First 1
if ($null -eq $resolved) {
    $resolved = $names | Where-Object { ($_ -split ':')[0] -eq $stem } | Select-Object -First 1
}
if ($null -eq $resolved) {
    # Pull it rather than printing the command and quitting.
    Say 'wait' 'model' ("$Model is not here yet - pulling it")
    Push-Location $Root
    try {
        & python -m coaxial_ollama.pull $Model
    } finally {
        Pop-Location
    }
    if ($LASTEXITCODE -ne 0) {
        Say 'fail' 'model' ("could not pull $Model - the daemon's words are above")
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

# Before the load, not after: warming the file cache only helps if it happens
# ahead of the read that actually needs it.
Invoke-Warm -Tag $Model

# An empty prompt loads the weights and generates nothing.
$clock = [Diagnostics.Stopwatch]::StartNew()
try {
    # options, and not just the tag.
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
    # THE DAEMON'S WORDS, not PowerShell's.
    $words = Get-DaemonWords $_
    if ($words -match 'llama-server binary not found') {
        # THE RUNNER, NOT THE MODEL.
        Say 'fail' 'ollama' ('its runner is missing from the install: ' +
                             ($words -split '\(checked')[0].Trim())
        Say 'fail' 'ollama' ('reinstall it - .\setup.ps1, or  irm https://ollama.com/install.ps1 | iex' +
                             '  - then run this again')
        exit 1
    }
    Say 'warn' 'model' ('could not preload: ' + $words)
}

if ($NoBoard) {
    Say 'warn' 'board' '--no-board: tools are stubbed, nothing is measured'
} elseif ($Simulated) {
    Say 'warn' 'board' '--simulated: no port opened, every reading is invented'
} else {
    if ($AutodetectComport) {
        $found = Find-BoardPort -PreferredPort $Port -HostDir $Root
        if ($found) { $Port = $found }
    }
    $ports = @()
    try { $ports = [System.IO.Ports.SerialPort]::GetPortNames() } catch { $ports = @() }
    # A port name in the list is not a board answering on it - measured, COM4
    # enumerated while the board stayed silent.
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
    # CPU scheduling, not the GPU's.
    $lowered = @()
    foreach ($proc in (Get-Process -Name 'ollama*' -ErrorAction SilentlyContinue)) {
        try {
            $proc.PriorityClass = [Diagnostics.ProcessPriorityClass]::BelowNormal
            $lowered += $proc.Name
        } catch {
            # A process started by another user, or already gone.
        }
    }
    if ($lowered.Count -gt 0) {
        Say 'ok' 'priority' (($lowered | Sort-Object -Unique) -join ', ') 
    }
}

# ---- the prompt itself -----------------------------------------------------

Push-Location $Root
try {
    if ($Plain) {
        if ($Ask) {
            # `ollama run TAG "question"` answers once and exits.
            foreach ($question in $Ask) {
                if ($Ask.Count -gt 1) { Say 'ok' 'ask' $question }
                & $ollama.Source run $Model $question
            }
            return
        }
        Write-Host ''
        Write-Host '  plain chat: no tools, no board. /bye leaves.' -ForegroundColor DarkGray
        Write-Host ''
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
        # One load for the lot.
        foreach ($question in $Ask) {
            if ($Ask.Count -gt 1) { Say 'ok' 'ask' $question }
            & python @call $question
        }
        return
    }

    Write-Host ''
    Write-Host ('  tools: ' + $Tools + '   /py CODE runs against the board, /sh runs a program,') -ForegroundColor DarkGray
    Write-Host '  both cost no tokens. /tools NAME repriced, /ctx, /clear, /q to leave.' -ForegroundColor DarkGray
    Write-Host ''
    & python @call --repl
} finally {
    Pop-Location

    # Leaving the prompt hands the card back, at once.
    if (-not $Hold) {
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
