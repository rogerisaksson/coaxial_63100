# machine.ps1 - Windows, winget, python (found, or installed from python.org), git, gh,
# a host gcc, the execution policy.

function Get-PythonGate {
    <#
  The minors CI runs host/ on (.github/workflows/host.yml), oldest first: the floor, and last
        the one it is developed on. Setup takes and installs only these: a newer python is one
        no suite has run on.
#>
    # $script:, not $Root: Find-Python's own $root is the same name to PowerShell. The list
    # flowed, python: ['3.12', '3.14'], or in a block, a - '3.12' line each.
    $ci = Get-Content (Join-Path $script:Root '.github\workflows\host.yml') -Raw -ErrorAction SilentlyContinue
    $matrix = [regex]::Match([string]$ci,
                             'python:[ \t]*(\[[^\]]*\]|(\s*-[ \t]*[''"]?\d+\.\d+[''"]?)+)').Groups[1].Value
    return @([regex]::Matches($matrix, '\d+\.\d+') | ForEach-Object { [version]$_.Value } | Sort-Object)
}

function Get-ShellPython {
    <#
  The python a new shell finds first - the registry's PATH, the machine's then the user's, each
        directory tried by PATHEXT - its path, '' for none. env.ps1, .mcp.json and the hooks run
        it. Found, never run: pymanager's alias, finding no runtime, downloaded one (2026-09-29).
#>
    $dirs = ([Environment]::GetEnvironmentVariable('Path', 'Machine') + ';' +
             [Environment]::GetEnvironmentVariable('Path', 'User')) -split ';' | Where-Object { $_ }
    foreach ($dir in $dirs) {
        foreach ($ext in ($env:PATHEXT -split ';' | Where-Object { $_ })) {
            $exe = Join-Path ([Environment]::ExpandEnvironmentVariables($dir)) ('python' + $ext)
            if (Test-Path $exe) { return $exe }
        }
    }
    return ''
}

function Test-PythonRuns {
    <#
  Does this python.exe start, and is it a minor CI runs (Get-PythonGate) ?
        Answering to the name is not the same as being one, and this machine
        had both ways of failing at once (measured 2026-09-03):
          the Store alias %LOCALAPPDATA%\Microsoft\WindowsApps\python.exe is
                            a zero-byte reparse point that prints "Python
                            was
                            not found" and exits 49.
                            `Get-Tool 'python'` was satisfied, this script
                            reported `ok python` with a blank version, and
                            then
                            everything downstream failed for its own
                            apparent reason: pip, the suites, the model
                            probe,
                            host gcc - which is asked of python -m
                            tools.cores.build and so goes through python too - and
                            the
                            MCP server in .mcp.json.
                            and the report pointed at none of them.
          an old one Anaconda's 3.8 and Visual Studio's 3.6 were both on
                            that machine.
                            requires-python >= 3.12.
        So run it and read the version back.
        worth its 40 ms here: everything after this point assumes it works.
#>
    param([string]$Exe)

    if ([string]::IsNullOrWhiteSpace($Exe)) { return $false }
    if (-not (Test-Path $Exe)) { return $false }

    # The stub is zero bytes.
    $item = Get-Item $Exe -ErrorAction SilentlyContinue
    if (($null -ne $item) -and ($item.Length -eq 0)) { return $false }

    # --version rather than -c: PowerShell 5.1 rewrites the quoting of a -c
    # snippet on its way to a native executable, which is why Invoke-Python
    # exists.
    $line = ''
    try { $line = (& $Exe --version 2>$null | Select-Object -First 1) } catch { return $false }
    if ($line -notmatch '^Python\s+(\d+)\.(\d+)') { return $false }

    $gate = Get-PythonGate
    $minor = [version]"$($Matches[1]).$($Matches[2])"
    return (($gate.Count -gt 0) -and ($minor -ge $gate[0]) -and ($minor -le $gate[-1]))
}


function Find-Python {
    <#
  python.exe, from a python.org per-user install.
        The installer writes under %LOCALAPPDATA%\Programs\Python\PythonXXX
        and
        only adds itself to the *user* PATH - which, like every PATH edit in
        this script, reaches shells opened after it and not this one.
        the well-known install directory before concluding python is absent,
        so
        an install this same run just performed does not get reported as
        missing a moment later.
        Every candidate is run before it is believed - see Test-PythonRuns
        for
        what was on the bench machine that made that necessary.
#>
    $found = Get-Tool 'python'
    if (Test-PythonRuns $found) { return $found }

    $root = Join-Path $env:LOCALAPPDATA 'Programs\Python'
    if (Test-Path $root) {
        # Sorted on the number, not the name: as strings Python39 comes after
        # Python314, which is exactly the pair this project sits between.
        $dirs = Get-ChildItem $root -Directory -Filter 'Python3*' -ErrorAction SilentlyContinue |
            Sort-Object {
                $m = [regex]::Match($_.Name, '^Python3(\d+)$')
                if ($m.Success) { [int]$m.Groups[1].Value } else { -1 }
            } -Descending
        foreach ($dir in $dirs) {
            $exe = Join-Path $dir.FullName 'python.exe'
            if (Test-PythonRuns $exe) {
                $env:PATH = $dir.FullName + ';' + (Join-Path $dir.FullName 'Scripts') + ';' + $env:PATH
                return $exe
            }
        }
    }

    # Last, the launcher: it knows about installs this script did not make - an
    # all-users one, or a path nobody would guess.
    $launcher = Get-Tool 'py'
    if ($null -ne $launcher) {
        $listed = @()
        try { $listed = (& $launcher -0p 2>$null) } catch { $listed = @() }
        foreach ($line in $listed) {
            if ($line -match '([A-Za-z]:\\[^"]+python\.exe)') {
                $exe = $Matches[1].Trim()
                if (Test-PythonRuns $exe) {
                    $env:PATH = (Split-Path $exe) + ';' + $env:PATH
                    return $exe
                }
            }
        }
    }

    return $null
}

function Resolve-PythonVersion {
    <#
  The newest release of the minor CI develops on (Get-PythonGate) with a windows amd64
        installer, asked of python.org's own ftp index rather than pinned as a constant here
        -
        the same reasoning as Resolve-Model: one source of truth, asked
        live,
        rather than a version number that goes stale the day it is written.
        -PythonVersion overrides this outright, which is also the fallback
        when the index cannot be reached at all - a bench with no internet
        to
        python.org has nowhere else this number could come from.
#>

    if ($PythonVersion) { return $PythonVersion }
    $minor = (Get-PythonGate)[-1]

    try {
        $index = Invoke-WebRequest -Uri 'https://www.python.org/ftp/python/' -UseBasicParsing -TimeoutSec 15
    } catch {
        Write-Item 'python.org index' 'failed' $_.Exception.Message
        return "$minor.0"
    }
    $versions = [regex]::Matches($index.Content, 'href="(3\.\d+\.\d+)/"') |
        ForEach-Object { $_.Groups[1].Value } |
        Where-Object { $_ -like "$minor.*" } |
        Sort-Object -Property @{ Expression = { [version]$_ } } -Descending

    foreach ($v in $versions) {
        $url = "https://www.python.org/ftp/python/$v/python-$v-amd64.exe"
        try {
            $head = Invoke-WebRequest -Uri $url -Method Head -UseBasicParsing -TimeoutSec 15
            if ($head.StatusCode -eq 200) { return $v }
        } catch {
            continue
        }
    }
    return "$minor.0"
}

function Install-PythonFromOrg {
    <#
  Python, from python.org rather than winget.
        A per-user install (InstallAllUsers=0) needs no administrator, which
        matters here because nothing else in this script does either.
        PrependPath=1 is what makes `python` resolve in shells opened after
        this one; Include_test=0 skips the standard library test suite,
        which
        this project never imports.
#>

    $version = Resolve-PythonVersion
    $url = "https://www.python.org/ftp/python/$version/python-$version-amd64.exe"
    $installer = Join-Path $env:TEMP "python-$version-amd64.exe"
    $gate = Get-PythonGate
    $minor = [version](($version -split '\.')[0..1] -join '.')
    if (($minor -lt $gate[0]) -or ($minor -gt $gate[-1])) {
        Write-Item 'python' 'failed' ('-PythonVersion {0}: CI runs {1}-{2}, and setup takes no other' -f $version, $gate[0], $gate[-1])
        Add-Todo ('setup.ps1 -PythonVersion {0}.x, or without it for the newest {0}' -f $gate[-1])
        return $null
    }

    Write-Item 'python' 'missing' "python.org $version"
    if (-not (Confirm-Step "download and run the python.org $version installer ?  (per-user, no admin)")) {
        Add-Todo "download $url and run it, or: setup.ps1 -Yes -PythonVersion $version"
        return $null
    }

    try {
        Invoke-WebRequest -Uri $url -OutFile $installer -UseBasicParsing
    } catch {
        Write-Item 'python' 'failed' ('download failed: ' + $_.Exception.Message)
        Add-Todo "download $url by hand and run it with: /quiet InstallAllUsers=0 PrependPath=1"
        return $null
    }

    try {
        $proc = Start-Process -FilePath $installer -ArgumentList @(
            '/quiet', 'InstallAllUsers=0', 'PrependPath=1', 'Include_test=0'
        ) -Wait -PassThru
    } catch {
        Write-Item 'python' 'failed' $_.Exception.Message
        Add-Todo "run $installer by hand: /quiet InstallAllUsers=0 PrependPath=1"
        return $null
    } finally {
        Remove-Item $installer -Force -ErrorAction SilentlyContinue
    }

    if ($proc.ExitCode -ne 0) {
        Write-Item 'python' 'failed' ("installer exit " + $proc.ExitCode)
        Add-Todo "run the python.org $version installer by hand - it exited $($proc.ExitCode)"
        return $null
    }

    $python = Find-Python
    if ($null -eq $python) {
        Write-Item 'python' 'failed' 'installer reported success but python.exe was not found'
        Add-Todo "python installed but not found under $(Join-Path $env:LOCALAPPDATA 'Programs\Python') - check the install"
        return $null
    }
    Write-Item 'python' 'done' $python
    return $python
}


function Test-Machine {
    Write-Head 'machine'
    Write-Item 'windows' 'ok' ((Get-CimInstance Win32_OperatingSystem).Caption)
    Write-Item 'powershell' 'ok' $PSVersionTable.PSVersion.ToString()

    # winget first: everything below asks it for the thing it is missing.
    if ($null -eq (Get-Tool 'winget')) {
        Write-Item 'winget' 'missing' 'nothing can be installed automatically'
        Add-Todo 'install App Installer from the Microsoft Store to get winget'
    } else {
        Write-Item 'winget' 'ok' ''
    }

    $gate = Get-PythonGate
    if ($gate.Count -eq 0) {
        Write-Item 'python' 'failed' '.github/workflows/host.yml names no python matrix: no minor to take'
        Add-Todo 'give .github/workflows/host.yml its python matrix back - setup takes the minors CI runs'
        return $null
    }
    $python = Find-Python
    if ($null -eq $python) {
        if ($Check) {
            $want = $(if ($PythonVersion) { $PythonVersion } else { [string]$gate[-1] })
            Write-Item 'python' 'missing' ('none of {0}-{1}, what CI runs - would install python.org {2}' -f $gate[0], $gate[-1], $want)
            Add-Todo ('run without -Check to install python {0} from python.org, or install it by hand' -f $want)
            return $null
        }
        $python = Install-PythonFromOrg
        if ($null -eq $python) { return $null }
        # Find-Python already spliced the new install's directory into this
        # process's PATH, so the interpreter is usable immediately - no new
        # shell needed, unlike the winget install this replaced.
    }
    $version = Invoke-Python -Python $python -Code @'
import sys
print('%d.%d.%d  %s' % (sys.version_info[0], sys.version_info[1],
                        sys.version_info[2], sys.executable))
'@
    Write-Item 'python' 'ok' $version

    # The one setup installs into must be the one a new shell runs: python.org's installer
    # leaves PATH alone unless asked, and another python can stand ahead of it.
    # pymanager's alias (WindowsApps) and its bin\ shims run its own runtimes.
    $mine, $shell = ($version -split '  ', 2)[1], (Get-ShellPython)
    $manager = Join-Path $env:LOCALAPPDATA 'Python'
    $same = $shell -and (((Split-Path $shell) -eq (Split-Path $mine)) -or (($mine -like "$manager\*") -and
            (($shell -like "$manager\bin\*") -or ($shell -like "$env:LOCALAPPDATA\Microsoft\WindowsApps\*"))))
    if (-not $same) {
        $dir = Split-Path $mine
        $why = $(if ($shell) { "a new shell runs $shell" } else { 'a new shell finds none' })
        Write-Item 'python on PATH' 'missing' "$why - env.ps1, .mcp.json and the hooks run it"
        if (Confirm-Step "put $dir first on your user PATH ?") {
            $user = [Environment]::GetEnvironmentVariable('Path', 'User')
            [Environment]::SetEnvironmentVariable('Path', "$dir;$(Join-Path $dir 'Scripts');$user", 'User')
            if ((Split-Path (Get-ShellPython)) -eq $dir) {
                Write-Item 'python on PATH' 'done' "$dir - new shells"
            } else {
                Write-Item 'python on PATH' 'failed' 'the machine PATH finds another python first'
                Add-Todo "take the other python off the machine PATH, or put $dir ahead of it"
            }
        } else {
            Add-Todo "put $dir and its Scripts first on your user PATH - new shells run another python"
        }
    }

    $git = Get-Tool 'git'
    if ($null -eq $git) {
        Write-Item 'git' 'missing' 'Git.Git'
        if (Install-WingetPackage -Id 'Git.Git' -Why 'to update this checkout') {
            Write-Item 'git' 'done' 'installed - new shells only'
        }
    } else {
        Write-Item 'git' 'ok' $git
    }

    # gh reads what CI could not say in public.
    $gh = Get-Tool 'gh'
    if ($null -eq $gh) {
        $ghExe = Join-Path $env:ProgramFiles 'GitHub CLI\gh.exe'
        if (Test-Path $ghExe) { $gh = $ghExe }
    }
    if ($null -eq $gh) {
        Write-Item 'gh' 'missing' 'optional - GitHub.cli, for watching CI runs'
        Add-Todo -Optional 'winget install --id GitHub.cli --exact, then once: gh auth login'
    } else {
        $authed = ((& $gh auth status 2>&1) -join ' ') -match 'Logged in'
        if ($authed) {
            Write-Item 'gh' 'ok' $gh
        } else {
            Write-Item 'gh' 'ok' 'installed - `gh auth login` once to watch CI runs'
            Add-Todo -Optional 'gh auth login   (one time - lets `gh run watch` follow CI)'
        }
    }


    # A host C compiler, for test_modbus_core.py. -m from host/ puts host/ on
    # sys.path: host/ is not installed yet.
    $cc = ''
    if ($null -ne $python) {
        Push-Location $Host_
        try {
            $cc = (& $python '-m' 'tools.cores.build' 2>$null | Select-Object -First 1)
        } catch {
            $cc = ''
        } finally {
            Pop-Location
        }
    }
    if ([string]::IsNullOrWhiteSpace($cc)) {
        Write-Item 'host gcc' 'missing' 'BrechtSanders.WinLibs.POSIX.UCRT - test_modbus_core.py skips without it'
        if (Install-WingetPackage -Id 'BrechtSanders.WinLibs.POSIX.UCRT' `
                                  -Why 'to build the Modbus core on this machine') {
            Write-Item 'host gcc' 'done' 'installed - new shells only'
        }
    } else {
        Write-Item 'host gcc' 'ok' $cc
    }

    # What a new shell obeys: the first scope that sets a policy, this run's own Process one
    # (Bypass, from the command line) apart; none set is Restricted on Windows 11. The plan's
    # yes sets CurrentUser: a new machine's env.ps1 would not load otherwise.
    $policy, $where = Get-ShellPolicy
    if ($policy -in 'Restricted', 'AllSigned') {
        Write-Item 'script execution' 'missing' "$where=$policy - env.ps1 will not load"
        if (($AllowScripts -or $script:OnlyChanges) -and -not $Check) {
            Set-ExecutionPolicy -Scope CurrentUser RemoteSigned -Force
            $policy, $where = Get-ShellPolicy
            if ($policy -in 'Restricted', 'AllSigned') {
                Write-Item 'script execution' 'failed' "$where=$policy outranks CurrentUser - a group policy"
                Add-Todo "a group policy sets ${where}=${policy} - env.ps1 needs RemoteSigned"
            } else {
                Write-Item 'script execution' 'done' "$where=$policy"
            }
        } else {
            Add-Todo 'Set-ExecutionPolicy -Scope CurrentUser RemoteSigned - env.ps1 loads in a normal shell (the plan''s yes sets it)'
        }
    } else {
        Write-Item 'script execution' 'ok' "$where=$policy"
    }
    return $python
}

function Get-ShellPolicy {
    <#
  (policy, scope) a new shell obeys: the first of MachinePolicy, UserPolicy, CurrentUser,
        LocalMachine that sets one; ('Restricted', 'default') where none does.
#>
    $set = @(Get-ExecutionPolicy -List | Where-Object {
        ($_.Scope -ne 'Process') -and ($_.ExecutionPolicy -ne 'Undefined') })
    if ($set.Count -eq 0) { return @('Restricted', 'default') }
    return @([string]$set[0].ExecutionPolicy, [string]$set[0].Scope)
}
