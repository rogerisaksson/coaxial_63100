# ollama.ps1 - ollama, the daemon tuning, the model this machine can carry, and the
# editor extension.

# The daemon settings, shared with board_chat.ps1 rather than written twice.
. (Join-Path $Root 'host\board_chat\Tuning.ps1')

function Find-Ollama {
    <#
  Both installers put ollama.exe under LOCALAPPDATA, and neither reaches
        the PATH of a shell that was already open.
        before concluding it is absent, and splice whatever we find into
        this
        process's PATH so the model pull below works without a new shell.
#>
    $found = Get-Tool 'ollama'
    if ($null -ne $found) { return $found }

    $candidates = @(
        (Join-Path $env:LOCALAPPDATA 'Programs\Ollama\ollama.exe'),
        (Join-Path $env:LOCALAPPDATA 'Microsoft\WinGet\Links\ollama.exe')
    )
    foreach ($c in $candidates) {
        if (Test-Path $c) {
            $env:PATH = (Split-Path $c) + ';' + $env:PATH
            return $c
        }
    }
    return $null
}

function Install-OllamaBinary {
    <#
  Two ways in, and the order is deliberate.
        winget is the tidy one: a known package, an uninstall entry, no code
        off
        the internet through iex.
        source index, and the winget install wants elevation.
        Ollama's own installer script is the fallback.
        under %LOCALAPPDATA%\Programs\Ollama and needs no administrator -
        which
        is also why it is worth having when winget declines.
        remote script, so it is behind its own confirmation rather than
        folded
        into the first one.
#>

    if ($null -ne (Get-Tool 'winget')) {
        if (Confirm-Step 'winget install Ollama.Ollama ?') {
            winget install --id Ollama.Ollama --exact --accept-package-agreements --accept-source-agreements
            $found = Find-Ollama
            if ($null -ne $found) { return $found }
            Write-Item 'ollama' 'note' 'winget did not produce an ollama on PATH'
        }
    } else {
        Write-Item 'winget' 'missing' 'falling back to the ollama installer script'
    }

    if (Confirm-Step 'irm https://ollama.com/install.ps1 | iex   ?  (per-user, no admin)') {
        try {
            Invoke-RestMethod -Uri 'https://ollama.com/install.ps1' | Invoke-Expression
        } catch {
            Write-Item 'ollama' 'failed' $_.Exception.Message
            return $null
        }
        return (Find-Ollama)
    }

    return $null
}

function Get-OllamaTags {
    <#
  The API answering matters more than the binary existing: everything in
        host/coaxial_ollama talks to the HTTP endpoint, not to the CLI.
        Retried, because a daemon started a second ago has not bound 11434
        yet
        and a single failed probe would send a working machine away with a
        manual step it does not need.
#>
    param([int]$Tries = 1)

    for ($i = 0; $i -lt $Tries; $i++) {
        try {
            return (Invoke-RestMethod -Uri 'http://localhost:11434/api/tags' -TimeoutSec 5)
        } catch {
            if ($i -lt ($Tries - 1)) { Start-Sleep -Seconds 2 }
        }
    }
    return $null
}

function Install-OllamaExtension {
    <#
  The editor half of the same daemon.
        extension: it registers the locally pulled models with VS Code's
        chat
        model picker, so the model answering in the editor is the one on
        11434
        that dbg.py and the runner already talk to - no second copy of the
        weights, no second configuration to keep in step.
        Nothing in this repository needs it.
        server all speak HTTP, so a machine with no VS Code gets a note here
        and not a failure.
#>

    # Find-Code, not Get-Tool: an editor installed a minute ago by the step
    # above is on disk but not on this shell's PATH, and reporting it missing
    # here would be this script disagreeing with itself.
    $code = Find-Code
    if ($null -eq $code) {
        Write-Item 'vs code' 'missing' 'no editor to install Ollama.ollama into'
        Add-Todo 'code --install-extension Ollama.ollama   (once VS Code is installed)'
        return
    }

    $id = 'Ollama.ollama'
    if ((& $code --list-extensions 2>$null) -contains $id) {
        Write-Item $id 'ok' ''
        return
    }
    Write-Item $id 'missing' ''
    if (Confirm-Step "code --install-extension $id ?") {
        & $code --install-extension $id --force
        if ($LASTEXITCODE -eq 0) {
            Write-Item $id 'done' 'installed'
        } else {
            Write-Item $id 'failed' "code exit $LASTEXITCODE"
            Add-Todo "code --install-extension $id"
        }
    } else {
        Add-Todo "code --install-extension $id"
    }
}

function Resolve-Model {
    <#
  Which tag this machine should run, if nobody said.
        The answer comes from capability.py rather than from a constant
        here,
        because it is the same question the runner and dbg.py ask and there
        should be one answer to it.
        that fails, falls back to the tag this bench was built on rather
        than
        stopping the install.
#>
    param([string]$Python)

    if ($Model) { return $Model }
    if ($null -eq $Python) { return 'gemma4:12b' }

    Push-Location $Host_
    try {
        $json = (& $Python '-m' 'coaxial_ollama.capability' '--json' '--prefer' $Prefer) -join ''
    } catch {
        $json = ''
    } finally {
        Pop-Location
    }
    if ([string]::IsNullOrWhiteSpace($json)) {
        Write-Item 'model choice' 'missing' 'could not measure this machine - falling back'
        return 'gemma4:12b'
    }
    try {
        $picked = $json | ConvertFrom-Json
    } catch {
        Write-Item 'model choice' 'missing' 'capability.py said something unreadable - falling back'
        return 'gemma4:12b'
    }
    $machine = $picked.host
    Write-Item 'this machine' 'ok' ('{0} cores / {1} threads, {2:n0} GB RAM, {3:n0} GB VRAM' `
        -f $machine.cores, $machine.threads, $machine.ram_gb, $machine.vram_gb)
    Write-Item 'model choice' 'ok' ('{0}  ({1})' -f $picked.model, $picked.why)
    foreach ($warning in $picked.warnings) {
        Write-Item '' 'note' $warning
    }
    if ($null -ne $picked.options.num_gpu) {
        # A split model needs the layer count on every call, and only dbg.py
        # and the runner can pass it.
        Add-Todo ('this machine runs ' + $picked.model + ' split across GPU and CPU: ' +
                  'use `dbg -m auto`, which passes num_gpu=' + $picked.options.num_gpu +
                  ', rather than a bare `ollama run`')
    }
    return $picked.model
}

function Install-Ollama {
    param([string]$Python)
    Write-Head 'ollama'
    $Model = Resolve-Model -Python $Python
    $ollama = Find-Ollama
    if ($null -eq $ollama) {
        Write-Item 'ollama' 'missing' 'Ollama.Ollama, or ollama.com/install.ps1'
        $ollama = Install-OllamaBinary
        if ($null -eq $ollama) {
            Add-Todo 'winget install --id Ollama.Ollama --exact   (or: irm https://ollama.com/install.ps1 | iex)'
            Add-Todo "ollama pull $Model"
            Install-OllamaExtension
            return
        }
        Write-Item 'ollama' 'done' $ollama
    } else {
        Write-Item 'ollama' 'ok' $ollama
    }

    # Before the daemon is started or asked anything: the settings that keep
    # llama-server from dying of std::bad_alloc partway through a bench
    # session.
    $tuned = @()
    if (-not $Check) { $tuned = @(Set-DaemonEnvironment) }
    if ($tuned.Count -gt 0) {
        Write-Item 'daemon tuning' 'done' (($tuned -join ', ') + ' - prompt cache off, checkpoints capped')
    } elseif ((Test-DaemonTuned) -eq $false) {
        Write-Item 'daemon tuning' 'missing' 'the running daemon predates it - board_chat.ps1 restarts it once'
    } else {
        Write-Item 'daemon tuning' 'ok' 'prompt cache off, checkpoints capped'
    }

    $tags = Get-OllamaTags
    if (($null -eq $tags) -and (-not $Check)) {
        # A fresh install has not started its daemon, and the install that just
        # ran did not put one in this session.
        Write-Item 'ollama serve' 'missing' 'nothing on 11434 - starting the daemon'
        Start-Process -FilePath $ollama -ArgumentList 'serve' -WindowStyle Hidden `
                      -ErrorAction SilentlyContinue
        $tags = Get-OllamaTags -Tries 10
    }

    if ($null -eq $tags) {
        Write-Item 'ollama serve' 'missing' 'nothing answering on 11434'
        Add-Todo ('start ollama - it normally runs as a service after install - then: ollama pull ' + $Model)
        Install-OllamaExtension
        return
    }

    $names = @()
    if ($null -ne $tags.models) { $names = $tags.models | ForEach-Object { $_.name } }
    Write-Item 'ollama serve' 'ok' ("$($names.Count) model(s): " + ($names -join ', '))

    # Stem matching, not tag matching, and deliberately so: it is the same
    # looseness Ollama.have() applies in host/coaxial_ollama/client.py, so a
    # machine this script calls ready is one the runner will also accept.
    $stem = ($Model -split ':')[0]
    $have = $names | Where-Object { ($_ -split ':')[0] -eq $stem }
    if ($null -ne $have) {
        Write-Item 'model' 'ok' ($have -join ', ')
    } else {
        Write-Item 'model' 'missing' $Model
        if (Confirm-Step "ollama pull $Model ?  (several GB - see the model choice above)") {
            & $ollama pull $Model
            if ($LASTEXITCODE -eq 0) {
                Write-Item 'model' 'done' $Model
            } else {
                Write-Item 'model' 'failed' "ollama exit $LASTEXITCODE"
                Add-Todo "ollama pull $Model"
            }
        } else {
            Add-Todo "ollama pull $Model"
        }
    }

    Install-OllamaExtension
}
