<#
    Daemon state: what tags exist, what is resident on the card, and
    unloading whatever should not be there before this run adds its own.
    Needs Say (Say.ps1) and $Api (set in board_chat.ps1 itself) already in
    scope - Clear-Resident also reads $KeepOthers, board_chat.ps1's own
    switch, straight from the caller's scope.
#>

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

    # llama-server too: it is ollama's own child, and stopping only the
    # parent leaves the runner orphaned, holding memory and handles. Measured
    # here - two orphans left behind by a stopped daemon, after which every
    # model load failed with `clip_init: ... std::bad_alloc`.
    foreach ($name in 'ollama app', 'ollama', 'llama-server') {
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
