<#
    Finding which COM port this board is actually on - board_prompt.ps1's
    -AutodetectComport path. Needs Say (Say.ps1) to be dot-sourced already.

    Both functions here call into host/tools/find_board.py rather than
    probing a port themselves - that script is also what
    coaxial_ollama/tools.py's link_diagnose tool imports directly, mid-
    session, so "does this port answer" is one implementation, not two that
    can drift apart. This side reaches it as a subprocess, once per
    candidate port, because PowerShell cannot import Python.
#>

function Test-BoardPort {
    <#  Does this board answer on $CandidatePort? find_board.py --probe
        does the actual work: open the port the same way coaxial.connect()
        would, the same Modbus round trip a real session makes, so a wrong
        port fails for the same reason it would fail a moment later inside
        dbg.py itself - not a different, weaker check that passes here.  #>
    param([string]$CandidatePort, [string]$HostDir, [int]$TimeoutSec = 6)

    try {
        $psi = New-Object System.Diagnostics.ProcessStartInfo
        $psi.FileName = 'python'
        $psi.Arguments = 'tools' + [System.IO.Path]::DirectorySeparatorChar +
                         'find_board.py --probe ' + $CandidatePort
        $psi.WorkingDirectory = $HostDir
        $psi.UseShellExecute = $false
        $psi.RedirectStandardOutput = $true
        $psi.RedirectStandardError = $true

        $proc = [System.Diagnostics.Process]::Start($psi)
        if (-not $proc.WaitForExit($TimeoutSec * 1000)) {
            try { $proc.Kill() } catch {}
            return $false
        }
        return $proc.ExitCode -eq 0
    } catch {
        return $false
    }
}

function Find-BoardPort {
    <#  Try -Port first if Windows even lists it, then every debug probe,
        then everything else. There IS a way to ask which one a programmer
        is on, contrary to what this said before: an ST-Link VCP enumerates
        under VID 0483, so find_board.py --kinds sorts the candidates
        without opening any of them, and only the ones worth trying get a
        round trip.

        Still loops, one Test-BoardPort call per candidate, rather than a
        single find_board.py --discover: each attempt gets its own "trying
        COMn..." line instead of one silent wait for all of them.  #>
    param([string]$PreferredPort, [string]$HostDir)

    $ports = @()
    try { $ports = [System.IO.Ports.SerialPort]::GetPortNames() } catch { $ports = @() }
    if ($ports.Count -eq 0) {
        Say 'fail' 'autodetect' 'no COM ports at all - nothing to try'
        return $null
    }

    # One call, no port opened: "COM4 probe" per line. A failure here is not
    # fatal - every port simply counts as 'serial' and the old order stands.
    $kind = @{}
    try {
        $lines = & python (Join-Path $HostDir 'tools/find_board.py') --kinds
        foreach ($line in $lines) {
            $bits = "$line".Trim() -split '\s+'
            if ($bits.Count -ge 2) { $kind[$bits[0]] = $bits[1] }
        }
    } catch {}

    $ordered = @()
    if ($PreferredPort -and ($ports -contains $PreferredPort)) {
        $ordered += $PreferredPort
    }
    $rest = $ports | Where-Object { $_ -ne $PreferredPort }
    $ordered += ($rest | Where-Object { $kind[$_] -eq 'probe' })
    $ordered += ($rest | Where-Object { $kind[$_] -ne 'probe' })

    foreach ($candidate in $ordered) {
        $what = $kind[$candidate]
        if (-not $what) { $what = 'serial' }
        Say 'wait' 'autodetect' "trying $candidate ($what)..."
        if (Test-BoardPort -CandidatePort $candidate -HostDir $HostDir) {
            Say 'ok' 'autodetect' "$candidate answered ($what)"
            return $candidate
        }
    }
    Say 'warn' 'autodetect' ('none of ' + ($ports -join ', ') + ' answered')
    return $null
}
