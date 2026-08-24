<#
    Daemon state: what tags exist, what is resident on the card, and
    unloading whatever should not be there before this run adds its own.
    Needs Say (Say.ps1) and $Api (set in board_prompt.ps1 itself) already in
    scope - Clear-Resident also reads $KeepOthers, board_prompt.ps1's own
    switch, straight from the caller's scope.
#>

# What the daemon has to be started with for a long bench session to survive
# one. Every entry is measured on this machine, not copied from a forum:
#
#   LLAMA_ARG_CACHE_RAM = 0
#       llama-server keeps a prompt cache of up to 8 GiB in host memory, and
#       writes the whole slot state into it whenever a new question's prompt
#       diverges early from the cached one - which is every question here,
#       since each one starts a fresh conversation. Measured: `prompt_save:
#       saving prompt with length 1446, total state size = 342.623 MiB`
#       followed immediately by `libc++abi: terminating due to uncaught
#       exception of type std::bad_alloc`, the runner dying with 0xc0000409
#       and ollama reloading 8 GB underneath the question. Nothing on this
#       bench ever re-asks a prompt it has already asked, so the cache this
#       crashes to maintain has nothing to hit.
#
#   LLAMA_ARG_CTX_CHECKPOINTS = 2
#       The other allocator in the same failure. Context checkpoints are
#       320.013 MiB each here - a fixed size set by the context window, not
#       by the prompt - and the default ceiling is 32 of them, which is
#       10 GB the card does not have beside 8 GB of weights. Two is enough
#       to keep the reuse the mechanism exists for: measured, checkpoint 1
#       was restored on the very next turn.
#
#   OLLAMA_MAX_LOADED_MODELS / OLLAMA_NUM_PARALLEL = 1
#       One model, one context. Two of either is how a 16 GB card ends up
#       asked for two copies of the weights at once - measured here as a 500
#       from the daemon, 'cudaMalloc failed', with nothing obviously wrong on
#       either side.
#
# Measured end to end: ten questions, twenty-seven model calls, zero
# std::bad_alloc and one model load. The same ten against an untuned daemon
# crashed the runner and reloaded the model mid-session.
$script:DaemonTuning = [ordered]@{
    'LLAMA_ARG_CACHE_RAM'       = '0'
    'LLAMA_ARG_CTX_CHECKPOINTS' = '2'
    'OLLAMA_MAX_LOADED_MODELS'  = '1'
    'OLLAMA_NUM_PARALLEL'       = '1'
}

function Get-TuningMarker {
    <#  When this machine's environment was last known to carry the tuning.

        A file rather than a log line, and a timestamp rather than a flag,
        because of what has to be decided: not "is the environment right" -
        that is readable directly - but "did the daemon now running start
        after it became right". A process cannot be asked what environment it
        inherited, and the daemon's own log is no help either: ollama's tray
        app writes server.log, a daemon this script starts writes to its own
        hidden console, so the newest line in that file can belong to a
        daemon that exited hours ago. A start time compared against this
        stamp is true for every way the daemon can have been launched.  #>
    return (Join-Path $env:LOCALAPPDATA 'Ollama\coaxial-tuning.txt')
}

function Get-DaemonProcess {
    <#  The process actually answering on 11434, not every ollama.exe on the
        machine - a `ollama list` from another window is one of those too. #>
    try {
        $owner = (Get-NetTCPConnection -LocalPort 11434 -State Listen `
                                       -ErrorAction Stop).OwningProcess
        if ($owner) {
            return (Get-Process -Id ($owner | Select-Object -First 1) -ErrorAction Stop)
        }
    } catch {
        # No Get-NetTCPConnection, or nothing listening. Fall through.
    }
    return (Get-Process -Name 'ollama' -ErrorAction SilentlyContinue |
            Sort-Object StartTime | Select-Object -Last 1)
}

function Test-DaemonTuned {
    <#  Whether the daemon now running actually has the tuning.

        Two sources, because neither covers both ways a daemon gets started:

          * `srv load_model: prompt cache is disabled` in server.log is what
            the daemon itself says, and ollama rotates that file on every
            daemon start - so when the file has been written to since this
            process started, everything in it belongs to this process and the
            newest such line is the answer. That is the tray app's case, and
            the ordinary one.
          * A daemon this script started writes to its own hidden console and
            never touches server.log, so the newest line there can belong to
            a daemon that exited hours ago. Reading it would restart a tuned
            daemon on every single run - measured, twice. For that case the
            stamp below is the evidence: the environment was complete at that
            time, and this process started after it.

        $false where the environment itself is not right yet, $null where
        there is nothing to ask - and a caller reading $null should tune,
        since not knowing costs one restart and being wrong costs the
        session.  #>
    foreach ($name in $script:DaemonTuning.Keys) {
        if ([Environment]::GetEnvironmentVariable($name, 'User') -ne
            $script:DaemonTuning[$name]) {
            return $false
        }
    }
    $daemon = Get-DaemonProcess
    if ($null -eq $daemon) { return $null }

    $log = Join-Path $env:LOCALAPPDATA 'Ollama\server.log'
    if (Test-Path $log) {
        $written = (Get-Item $log).LastWriteTime
        if ($written -gt $daemon.StartTime) {
            $lines = @(Select-String -Path $log -ErrorAction SilentlyContinue `
                                     -Pattern 'prompt cache is (enabled|disabled)')
            if ($lines.Count -gt 0) {
                return ($lines[-1].Matches[0].Groups[1].Value -eq 'disabled')
            }
            # This daemon's own log, and it has not loaded a model yet. It has
            # not said either way, and a restart with nothing resident is the
            # cheapest this decision ever gets.
            return $false
        }
    }

    $marker = Get-TuningMarker
    if (-not (Test-Path $marker)) { return $null }
    try {
        $stamped = [datetime](Get-Content $marker -First 1)
    } catch {
        return $null
    }
    return ($daemon.StartTime -gt $stamped)
}

function Set-DaemonEnvironment {
    <#  The tuning into this process and into the user's environment.

        Both, and for different reasons: the process copy is what a daemon
        started by this script inherits, and the User copy is what the tray
        app inherits at the next login - which is how the machine stays tuned
        without this script having to run first. Returns the names it
        actually had to change, so a session that was already correct says
        nothing about it.

        The stamp is written the first time the environment is complete, not
        only when something changed: a machine tuned by hand with setx is
        still a machine whose running daemon predates it.  #>
    $changed = @()
    foreach ($name in $script:DaemonTuning.Keys) {
        $want = $script:DaemonTuning[$name]
        Set-Item -Path ("env:" + $name) -Value $want
        if ([Environment]::GetEnvironmentVariable($name, 'User') -ne $want) {
            try {
                [Environment]::SetEnvironmentVariable($name, $want, 'User')
                $changed += $name
            } catch {
                Say 'warn' 'daemon' ('could not persist ' + $name + ': ' + $_.Exception.Message)
            }
        }
    }
    $marker = Get-TuningMarker
    if (($changed.Count -gt 0) -or (-not (Test-Path $marker))) {
        try {
            $folder = Split-Path $marker -Parent
            if (-not (Test-Path $folder)) {
                New-Item -ItemType Directory -Path $folder -Force | Out-Null
            }
            Set-Content -Path $marker -Encoding utf8 -Value @(
                (Get-Date).ToString('o'),
                '# When this machine last had every LLAMA_ARG_/OLLAMA_ setting',
                '# board_prompt.ps1 wants. A daemon started before this line',
                '# did not inherit them - see board_prompt/Ollama.ps1.')
        } catch {
            Say 'warn' 'daemon' ('could not write ' + $marker + ': ' + $_.Exception.Message)
        }
    }
    return $changed
}

function Restart-Daemon {
    <#  Stop ollama and start it again, so it picks the environment up.

        An already-running daemon keeps the environment it was started with -
        there is no API to change it, and no amount of setting variables in
        this shell reaches a process that started at login. So the tuning
        costs one restart, once, and the tray app is put back if it was the
        thing running: on a workstation that icon is how the user expects to
        find ollama, and replacing it with a bare `serve` would be this
        script quietly redecorating somebody's desktop.  #>
    param([string]$Exe)

    # The path as a string, read now: a Process object whose process has
    # exited no longer answers .Path, so reading it after the Stop-Process
    # below returns $null and the restart puts nothing back. Measured exactly
    # that way, once, with the daemon left down afterwards.
    $trayPath = ''
    $tray = @(Get-Process -Name 'ollama app' -ErrorAction SilentlyContinue)
    if ($tray.Count -gt 0) {
        try { $trayPath = [string]$tray[0].Path } catch { $trayPath = '' }
    }

    foreach ($name in 'ollama app', 'ollama') {
        foreach ($proc in @(Get-Process -Name $name -ErrorAction SilentlyContinue)) {
            try {
                Stop-Process -Id $proc.Id -Force -ErrorAction Stop
            } catch {
                Say 'warn' 'daemon' ('could not stop ' + $name + ': ' + $_.Exception.Message)
            }
        }
    }
    Start-Sleep -Milliseconds 700

    if ($trayPath -and (Test-Path $trayPath)) {
        Start-Process -FilePath $trayPath -ErrorAction SilentlyContinue
    } else {
        Start-Process -FilePath $Exe -ArgumentList 'serve' -WindowStyle Hidden `
                      -ErrorAction SilentlyContinue
    }
    return (Get-Tags -Tries 10)
}

function Initialize-Daemon {
    <#  Leave the daemon configured to run as stably as this bench can make
        it, restarting it once if that is what it takes.

        Skipped by -NoTune, and skipped when -KeepOthers says another session
        is using this daemon: a restart there would take somebody else's
        loaded model with it, which is a worse outcome than the crash this
        prevents.  #>
    param([string]$Exe, [switch]$JustStarted)

    if ($NoTune) {
        Say 'ok' 'daemon' 'not tuned (-NoTune) - the prompt cache stays on'
        return
    }

    $persisted = Set-DaemonEnvironment
    if ($JustStarted) {
        # This shell started it, three lines ago, with the environment above
        # already in it. Nothing to read and nothing to restart - and reading
        # the log here would get the wrong answer: a daemon started by this
        # script writes to its own hidden console, not to server.log, so the
        # newest line in that file still belongs to whichever daemon ran
        # before it.
        Say 'ok' 'daemon' 'started with the prompt cache off and checkpoints capped'
        return
    }

    if ((Test-DaemonTuned) -eq $true) {
        Say 'ok' 'daemon' 'prompt cache off, checkpoints capped - already tuned'
        return
    }
    if ($KeepOthers) {
        Say 'warn' 'daemon' ('needs a restart to pick the tuning up, and ' +
                             '-KeepOthers says something else is using it - skipped')
        return
    }

    $why = 'restarting it to take effect'
    if ($persisted.Count -gt 0) {
        $why = ('set ' + ($persisted -join ', ') + ' - ' + $why)
    }
    Say 'wait' 'daemon' $why
    if ($null -eq (Restart-Daemon -Exe $Exe)) {
        Say 'warn' 'daemon' 'did not come back on 11434 - carrying on untuned'
        return
    }
    Say 'ok' 'daemon' 'prompt cache off, checkpoints capped, one model at a time'
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

function Get-Resident {
    <#  What ollama is holding on the card, as returned by /api/ps. #>
    try {
        $ps = Invoke-RestMethod -Uri ($Api + '/api/ps') -TimeoutSec 10
    } catch {
        return @()
    }
    if ($null -eq $ps.models) { return @() }
    return @($ps.models)
}

function Clear-Resident {
    <#  Unload anything still on the card, except the model about to be used.

        A prompt that exits cleanly hands its weights back, but not every exit
        is clean: a killed window, a -Hold from last time, an `ollama run` in
        another terminal. Those sit there until their keep_alive runs out, and
        the next load then asks a card that is already full - which on this
        bench was a 500 from the daemon reading 'cudaMalloc failed', with
        nothing obviously wrong at either end.

        Keeping a matching model is not an exception to the rule, it is the
        rule: what is being cleared is VRAM nobody is going to use, and weights
        this run is about to load are not that.  #>
    param([string]$Except)

    if ($KeepOthers) { return }

    foreach ($entry in (Get-Resident)) {
        if ($entry.name -eq $Except) {
            Say 'ok' 'resident' ('{0} already loaded, {1:n1} GB - kept' `
                -f $entry.name, ($entry.size_vram / 1GB))
            continue
        }
        try {
            $body = @{ model = $entry.name; prompt = ''; keep_alive = 0 } | ConvertTo-Json
            Invoke-RestMethod -Uri ($Api + '/api/generate') -Method Post -Body $body `
                              -ContentType 'application/json' -TimeoutSec 60 | Out-Null
            Say 'ok' 'unloaded' ('{0} was still resident, {1:n1} GB freed' `
                -f $entry.name, ($entry.size_vram / 1GB))
        } catch {
            Say 'warn' 'unloaded' ('could not unload ' + $entry.name + ': ' + $_.Exception.Message)
        }
    }
}
