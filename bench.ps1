<#
.SYNOPSIS
    One window with the model and the board in it.

        . .\env.ps1 ; .\bench.ps1
        .\bench.ps1 -NewWindow          the same, in its own window
        .\bench.ps1 -Plain              plain ollama chat, no board, no tools

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
    Ollama tag. Local only - the Python refuses a :cloud tag and a daemon on
    another machine unless you pass --allow-remote to it yourself.

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
#>
[CmdletBinding()]
param(
    [string]$Model = 'gemma4:12b',
    [string]$Port = 'COM4',
    [ValidateSet('read', 'code', 'pins', 'all', 'none')]
    [string]$Tools = 'code',
    [string]$Ask,
    [switch]$NoBoard,
    [switch]$Plain,
    [switch]$NewWindow,
    [string]$KeepAlive = '30m'
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
$stem = ($Model -split ':')[0]
$resolved = $names | Where-Object { $_ -eq $Model } | Select-Object -First 1
if ($null -eq $resolved) {
    $resolved = $names | Where-Object { ($_ -split ':')[0] -eq $stem } | Select-Object -First 1
}
if ($null -eq $resolved) {
    Say 'fail' 'model' ("$Model is not pulled. Have: " + (($names -join ', ')))
    Write-Host ''
    Write-Host "    ollama pull $Model" -ForegroundColor Yellow
    Write-Host ''
    exit 1
}
$Model = $resolved

# An empty prompt loads the weights and generates nothing. Doing it here means
# the 8 GB wait is visible and timed, instead of hiding inside question one.
$clock = [Diagnostics.Stopwatch]::StartNew()
try {
    $body = @{ model = $Model; prompt = ''; stream = $false; keep_alive = $KeepAlive } |
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

    $call = @('dbg.py', '-m', $Model, '-t', $Tools, '--port', $Port)
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
