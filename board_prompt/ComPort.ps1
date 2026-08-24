<#
    Finding which COM port this board is actually on - board_prompt.ps1's
    -AutodetectComport path. Needs Say (Say.ps1) to be dot-sourced already.
#>

function Test-BoardPort {
    <#  Open one COM port from Python's own side, exactly the way dbg.py
        would, and see whether this board answers on it.

        A .NET SerialPort.Open() would only prove the port exists and
        nothing else has it - a USB mouse dongle opens fine and is not this
        board. Going through coaxial.connect() is the same Modbus round
        trip a real session makes, so a wrong port fails for the same reason
        it would fail then, not a different, weaker check that passes here
        and fails a moment later inside dbg.py itself.  #>
    param([string]$CandidatePort, [string]$HostDir, [int]$TimeoutSec = 6)

    $probe = "from coaxial import connect, disconnect$([Environment]::NewLine)" +
             "b = connect([(1, 115200, '$CandidatePort')])$([Environment]::NewLine)" +
             "disconnect(b)$([Environment]::NewLine)"
    $tmp = [System.IO.Path]::GetTempFileName() + '.py'
    Set-Content -Path $tmp -Value $probe -Encoding utf8

    try {
        $psi = New-Object System.Diagnostics.ProcessStartInfo
        $psi.FileName = 'python'
        $psi.Arguments = '"' + $tmp + '"'
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
    } finally {
        Remove-Item $tmp -ErrorAction SilentlyContinue
    }
}

function Find-BoardPort {
    <#  Try -Port first if Windows even lists it, then every other COM port
        in whatever order Windows enumerates them - there is no way to ask
        which one a programmer was just plugged into short of opening each
        and finding out.  #>
    param([string]$PreferredPort, [string]$HostDir)

    $ports = @()
    try { $ports = [System.IO.Ports.SerialPort]::GetPortNames() } catch { $ports = @() }
    if ($ports.Count -eq 0) {
        Say 'fail' 'autodetect' 'no COM ports at all - nothing to try'
        return $null
    }

    $ordered = @()
    if ($PreferredPort -and ($ports -contains $PreferredPort)) {
        $ordered += $PreferredPort
    }
    $ordered += ($ports | Where-Object { $_ -ne $PreferredPort })

    foreach ($candidate in $ordered) {
        Say 'wait' 'autodetect' "trying $candidate..."
        if (Test-BoardPort -CandidatePort $candidate -HostDir $HostDir) {
            Say 'ok' 'autodetect' "$candidate answered"
            return $candidate
        }
    }
    Say 'warn' 'autodetect' ('none of ' + ($ports -join ', ') + ' answered')
    return $null
}
