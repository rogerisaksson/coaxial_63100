# machine.ps1 - Windows, winget, python (found, or installed from python.org), git, gh,
# a host gcc, the execution policy.

function Test-PythonRuns {
    <#
  Does this python.exe start, and is it new enough to run host/ ?
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

    $major = [int]$Matches[1]
    $minor = [int]$Matches[2]
    if ($major -gt 3) { return $true }
    return (($major -eq 3) -and ($minor -ge 12))   # host/pyproject.toml
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
  The newest CPython release that ships a windows amd64 installer, asked
        of python.org's own ftp index rather than pinned as a constant here
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

    try {
        $index = Invoke-WebRequest -Uri 'https://www.python.org/ftp/python/' -UseBasicParsing -TimeoutSec 15
    } catch {
        Write-Item 'python.org index' 'failed' $_.Exception.Message
        return '3.13.1'
    }
    $versions = [regex]::Matches($index.Content, 'href="(3\.\d+\.\d+)/"') |
        ForEach-Object { $_.Groups[1].Value } |
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
    return '3.13.1'
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

    $python = Find-Python
    if ($null -eq $python) {
        if ($Check) {
            Write-Item 'python' 'missing' 'would install from python.org - see -PythonVersion'
            Add-Todo 'run without -Check to install python from python.org, or install python 3.9+ by hand'
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

    $policy = Get-ExecutionPolicy -Scope CurrentUser
    if ($policy -eq 'Undefined' -or $policy -eq 'Restricted') {
        Write-Item 'script execution' 'missing' "CurrentUser=$policy - env.ps1 will not load"
        if ($AllowScripts -and -not $Check) {
            Set-ExecutionPolicy -Scope CurrentUser RemoteSigned -Force
            Write-Item 'script execution' 'done' 'CurrentUser=RemoteSigned'
        } else {
            Add-Todo 'to dot-source env.ps1 in a normal shell: Set-ExecutionPolicy -Scope CurrentUser RemoteSigned  (or re-run this script with -AllowScripts)'
        }
    } else {
        Write-Item 'script execution' 'ok' "CurrentUser=$policy"
    }
    return $python
}
