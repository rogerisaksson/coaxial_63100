<#
    What the ollama daemon has to be started with, and whether the one running
    now was. No output and no dependencies: dot-sourced by board_prompt.ps1,
    which reports through Say, and by setup.ps1, which reports through
    Write-Item. Both need the same four settings and the same answer to "is it
    already tuned".
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
                # Silent by contract: this file prints nothing. A variable
                # that could not be persisted is still set for this process
                # and for whatever this process starts, which is the case
                # that matters most.
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
            # Best-effort: without the stamp the worst case is one extra
            # restart, next run.
        }
    }
    return $changed
}

