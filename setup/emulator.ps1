# emulator.ps1 - Renode, and the electronic_simulations the emulated front end is
# fitted from.

#: Renode, the emulator tools/emu runs the image on; the version CI's firmware job fetches too.
$RenodeVersion = '1.17.0'

function Find-Renode {
    <#
  Where host/tools/emu/emulator.py looks, in its order: RENODE, PATH, then the
        portable builds under %LOCALAPPDATA%\renode.
#>
    if ($env:RENODE -and (Test-Path $env:RENODE)) { return $env:RENODE }
    $onPath = Get-Tool 'renode'
    if ($null -ne $onPath) { return $onPath }
    $local = Get-ChildItem (Join-Path $env:LOCALAPPDATA 'renode') -Directory -Filter 'renode_*' `
                          -ErrorAction SilentlyContinue |
             Sort-Object Name -Descending | Select-Object -First 1
    if ($null -ne $local) {
        $exe = Join-Path $local.FullName 'renode.exe'
        if (Test-Path $exe) { return $exe }
    }
    return $null
}

function Install-Renode {
    Write-Head 'emulator'
    $renode = Find-Renode
    if ($null -ne $renode) {
        Write-Item 'Renode' 'ok' $renode
        return
    }
    if ($SkipRenode) {
        Write-Item 'Renode' 'missing' '-SkipRenode: emulator:// and tests/test_emulator.py skip'
        return
    }
    $zip = "renode-$RenodeVersion.windows-portable.zip"
    if (-not (Confirm-Step "download Renode $RenodeVersion ($zip, 106 MB) into %LOCALAPPDATA%\renode ?")) {
        Write-Item 'Renode' 'missing' "github.com/renode/renode/releases v$RenodeVersion"
        Add-Todo "Renode $RenodeVersion portable under %LOCALAPPDATA%\renode (setup.ps1 again, or RENODE=path)"
        return
    }
    $into = Join-Path $env:LOCALAPPDATA 'renode'
    New-Item -ItemType Directory -Force $into | Out-Null
    $file = Join-Path $into $zip
    try {
        Invoke-WebRequest -UseBasicParsing -OutFile $file `
            "https://github.com/renode/renode/releases/download/v$RenodeVersion/$zip"
        Expand-Archive -Force $file $into
        Remove-Item $file
    } catch {
        Write-Item 'Renode' 'failed' $_.Exception.Message
        Add-Todo "Renode $RenodeVersion portable under %LOCALAPPDATA%\renode"
        return
    }
    $renode = Find-Renode
    if ($null -eq $renode) {
        Write-Item 'Renode' 'failed' "no renode.exe under $into"
        Add-Todo "Renode $RenodeVersion portable under %LOCALAPPDATA%\renode"
    } else {
        Write-Item 'Renode' 'done' $renode
    }
}

function Install-Simulations {
    <#
  The electronic_simulations submodule: the LTspice runs the emulator's front end is fitted
        from (host/tools/emu/afe_spice.py). Over HTTPS - .gitmodules names it by SSH, and a
        machine without a GitLab key cannot clone that; HTTPS asks for a GitLab login. LTspice itself is only needed to
        refit; the fit is checked in.
#>
    $sims = Join-Path $Root 'electronic_simulations'
    if (Test-Path (Join-Path $sims 'afe\amplifiers.asc')) {
        Write-Item 'electronic_simulations' 'ok' $sims
    } elseif (Confirm-Step -Attended 'git submodule update --init electronic_simulations (over HTTPS) ?') {
        & git -C $Root -c 'url.https://gitlab.com/.insteadOf=git@gitlab.com:' `
            submodule update --init electronic_simulations 2>&1 | Out-Null
        if (Test-Path (Join-Path $sims 'afe\amplifiers.asc')) {
            Write-Item 'electronic_simulations' 'done' $sims
        } else {
            Write-Item 'electronic_simulations' 'failed' 'the clone did not land'
            Add-Todo 'git submodule update --init electronic_simulations' -Optional
        }
    } else {
        Write-Item 'electronic_simulations' 'missing' 'only afe_spice.py; the traced constants are in coaxial/model/inverter.py'
        Add-Todo 'git submodule update --init electronic_simulations' -Optional
    }

    $ltspice = Join-Path $env:ProgramFiles 'ADI\LTspice\LTspice.exe'
    if (Test-Path $ltspice) {
        Write-Item 'LTspice' 'ok' $ltspice
    } else {
        Write-Item 'LTspice' 'missing' 'only to refit the front end (analog.com, LTspice)'
        Add-Todo 'LTspice, to refit the emulator''s front end from the simulations' -Optional
    }
}
