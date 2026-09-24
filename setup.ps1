<#
.SYNOPSIS
    Sets up a machine to build, flash and drive the coaxial_63100 board.
.DESCRIPTION
    Run -Check first.
.PARAMETER Check
    Report only.
.PARAMETER Yes
    Do not ask before each install.
.PARAMETER Model
    The ollama tag to pull.
.PARAMETER Prefer
    What the automatic choice optimises for.
.PARAMETER SkipOllama
    Leave the model side alone - for a machine that only builds and flashes.
.PARAMETER SkipCubeMX
    Do not install STM32CubeMX at all.
.PARAMETER CubeMXInstaller
    Run an STM32CubeMX installer that is already on disk, instead of taking
    the bundle.
.PARAMETER SkipCubeIDE
    Do not look for or offer STM32CubeIDE at all.
.PARAMETER CubeIDEInstaller
    Run an STM32CubeIDE installer that is already on disk.
.PARAMETER SkipDriver
    Do not touch the ST-Link USB driver.
.PARAMETER SkipFirmware
    Do not look for the STM32Cube FW_H7 package.
.PARAMETER FirmwarePackage
    A STM32Cube_FW_H7_Vx.y.z.zip, or an already unpacked copy of one, to
    install into the CubeMX repository.
.PARAMETER Repository
    Where CubeMX keeps its firmware packages.
.PARAMETER WingetToolchain
    Install cmake, ninja and arm-none-eabi-gcc from winget instead of
    relying on the VS Code extension's bundles.
.PARAMETER PythonVersion
    The CPython release to install from python.org when python is absent,
    e.g. '3.13.1'.
.PARAMETER AllowScripts
    Set the CurrentUser execution policy to RemoteSigned, so that `.
#>
[CmdletBinding()]
param(
    [switch]$Check,
    [switch]$Yes,
    [string]$Model,
    [ValidateSet('speed', 'capability')]
    [string]$Prefer = 'speed',
    [switch]$SkipOllama,
    [switch]$SkipCubeMX,
    [switch]$SkipDriver,
    [switch]$SkipFirmware,
    [string]$FirmwarePackage,
    [string]$CubeMXInstaller,
    [switch]$SkipCubeIDE,
    [string]$CubeIDEInstaller,
    [string]$Repository = (Join-Path $env:USERPROFILE 'STM32Cube\Repository'),
    [switch]$WingetToolchain,
    [string]$PythonVersion,
    [switch]$AllowScripts
)

$ErrorActionPreference = 'Continue'
$Root = $PSScriptRoot
$Host_ = Join-Path $Root 'host'
$BundleRoot = Join-Path $env:LOCALAPPDATA 'stm32cube\bundles'
$CubeH7Url = 'https://www.st.com/en/embedded-software/stm32cubeh7.html'
$CubeMXUrl = 'https://www.st.com/en/development-tools/stm32cubemx.html'
$CubeIDEUrl = 'https://www.st.com/en/development-tools/stm32cubeide.html'
$script:Todo = @()
$script:Optional = @()

# The daemon settings, shared with board_chat.ps1 rather than written twice.
. (Join-Path $PSScriptRoot 'host\board_chat\Tuning.ps1')

# ---- reporting -------------------------------------------------------------

function Write-Head {
    param([string]$Text)
    Write-Host ''
    Write-Host "-- $Text " -ForegroundColor Cyan -NoNewline
    Write-Host ('-' * [Math]::Max(0, 60 - $Text.Length)) -ForegroundColor DarkCyan
}

function Write-Item {
    param([string]$Name, [string]$State, [string]$Detail = '')
    $colour = 'Gray'
    if ($State -eq 'ok')      { $colour = 'Green' }
    if ($State -eq 'done')    { $colour = 'Green' }
    if ($State -eq 'missing') { $colour = 'Yellow' }
    if ($State -eq 'failed')  { $colour = 'Red' }
    if ($State -eq 'manual')  { $colour = 'Magenta' }
    Write-Host ('  {0,-8}' -f $State) -ForegroundColor $colour -NoNewline
    Write-Host ('{0,-26} ' -f $Name) -NoNewline
    Write-Host $Detail -ForegroundColor DarkGray
}

function Add-Todo {
    <#
  Something the operator still has to do.
        numbered list and out of the exit code: STM32CubeIDE needs an st.com
        login and is not part of this project's workflow, and printing it as
        step 1 of "next" made a finished setup read as an unfinished one -
        to a person, and to anything automating this.
#>
    param([string]$Text, [switch]$Optional)
    if ($Optional) {
        $script:Optional += $Text
    } else {
        $script:Todo += $Text
    }
}

function Confirm-Step {
    param([string]$Text)
    if ($Check) { return $false }
    if ($Yes)   { return $true }
    return (Read-YesNo ("  " + $Text + "  [y/N]"))
}

function Read-YesNo {
    <#
  y or n, and no on anything else - including no console at all.
        Read-Host throws when stdin is at end of file, which is what a piped
        or
        scheduled run looks like.
        through, with some things installed and some not; caught, it is
        simply
        the answer 'no', and the run finishes with the rest on the todo
        list.
        A run that wants everything says so with -Yes.
#>
    param([string]$Prompt)

    try {
        $answer = Read-Host $Prompt
    } catch {
        Write-Host '  (no console to ask on - taking that as no)' -ForegroundColor DarkGray
        return $false
    }
    return ($answer -match '^(y|yes)$')
}

function Get-Tool {
    param([string]$Name)
    $found = Get-Command $Name -ErrorAction SilentlyContinue
    if ($null -eq $found) { return $null }
    return $found.Source
}

function Invoke-Python {
    <#
  Run a snippet through a temp file rather than `python -c`.
        Windows PowerShell 5.1 rewrites the quoting of arguments on their
        way to
        a native executable, so a -c snippet containing quotes or a % format
        arrives at the interpreter mangled.
#>
    param([string]$Python, [string]$Code)

    $tmp = Join-Path $env:TEMP ('coaxial-probe-' + [guid]::NewGuid().ToString('N') + '.py')
    Set-Content -Path $tmp -Value $Code -Encoding utf8
    try {
        return (& $Python $tmp)
    } finally {
        Remove-Item $tmp -Force -ErrorAction SilentlyContinue
    }
}

function Install-WingetPackage {
    <#
  One winget install, with the three flags that make it non-interactive
        and the one exit code that is not a failure.
        winget answers -1978335189 (0x8A15002B) for "already installed",
        which
        happens whenever a package is on the machine but its shim has not
        reached this shell's PATH - the exact situation this script is in
        half
        the time.
        with a todo it does not need.
#>
    param([string]$Id, [string]$Why = '')

    if ($null -eq (Get-Tool 'winget')) {
        Add-Todo ("install $Id by hand - winget is absent on this machine")
        return $false
    }
    $prompt = "winget install $Id ?"
    if ($Why -ne '') { $prompt = $prompt + "  ($Why)" }
    if (-not (Confirm-Step $prompt)) {
        Add-Todo "winget install --id $Id --exact"
        return $false
    }
    winget install --id $Id --exact --silent `
                   --accept-package-agreements --accept-source-agreements
    return ($LASTEXITCODE -eq 0 -or $LASTEXITCODE -eq -1978335189)
}

function Update-WingetPackage {
    <#
  One winget upgrade, and the exit codes that mean "already newest".
        Installed is not the same as current, and for the editor the
        difference is real: the STM32 extension pack is version-matched to
        VS Code, and a bench two years behind on the editor gets extensions
        that decline to load rather than an error that says why.
        Two codes are a no-op rather than a failure, and they are the same
        code twice.
        $LASTEXITCODE holds inside this script; 43 is that number as
        anything
        outside sees it, because a process exit code is truncated to its low
        byte and 0x8A15002B ends in 0x2B.
        same whether it is called here or from a wrapper.
        Two situations answer with it, measured 2026-09-03: a package that
        is
        already newest, and one winget did not install - VS Code on this
        bench
        came from somewhere else, and `winget list --id
        Microsoft.VisualStudioCode --exact` prints "No installed package
        found" and still exits 0, so the list is no use as a test.
        a failure and neither belongs on the todo list: an editor winget
        does
        not manage is one that updates itself.
        Anything else is reported and left on the todo list: an upgrade that
        half-ran is worth knowing about, and this script never has the
        package's own updater's context for why it declined.
#>
    param([string]$Id, [string]$What, [string]$Why = '')

    if ($null -eq (Get-Tool 'winget')) { return $false }

    $prompt = "winget upgrade $Id ?"
    if ($Why -ne '') { $prompt = $prompt + "  ($Why)" }
    if (-not (Confirm-Step $prompt)) { return $false }

    winget upgrade --id $Id --exact --silent `
                   --accept-package-agreements --accept-source-agreements
    if ($LASTEXITCODE -eq 0) {
        Write-Item $What 'done' 'upgraded to the newest winget offers'
        return $true
    }
    if (($LASTEXITCODE -eq 43) -or ($LASTEXITCODE -eq -1978335189)) {
        Write-Item $What 'ok' 'nothing newer from winget (already current, or not a winget install)'
        return $false
    }
    Write-Item $What 'failed' "winget exit $LASTEXITCODE"
    Add-Todo "winget upgrade --id $Id --exact"
    return $false
}

function Find-Code {
    <#
  `code` is a .cmd shim, and the installer adds its directory to the user
        PATH - which reaches shells opened afterwards, never this one.
        where both the user and the machine installer put it before
        concluding
        VS Code is absent.
#>
    $found = Get-Tool 'code'
    if ($null -ne $found) { return $found }

    $candidates = @(
        (Join-Path $env:LOCALAPPDATA 'Programs\Microsoft VS Code\bin\code.cmd'),
        (Join-Path $env:ProgramFiles 'Microsoft VS Code\bin\code.cmd'),
        (Join-Path ${env:ProgramFiles(x86)} 'Microsoft VS Code\bin\code.cmd')
    )
    foreach ($c in $candidates) {
        if (($null -ne $c) -and (Test-Path $c)) {
            $env:PATH = (Split-Path $c) + ';' + $env:PATH
            return $c
        }
    }
    return $null
}

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
                            requires-python >= 3.10.
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
    return (($major -eq 3) -and ($minor -ge 10))   # host/pyproject.toml
}

function Get-CodeVersion {
    <#
  The editor's version and where it is, as one line for the report.
        `code --version` prints three: the version, the commit and the
        architecture.
        version beside it does not answer "is this bench current?" - which
        is
        the question the upgrade step below exists for.
#>
    param([string]$Code)

    $version = ''
    try { $version = (& $Code --version 2>$null | Select-Object -First 1) } catch { $version = '' }
    if ([string]::IsNullOrWhiteSpace($version)) { return $Code }
    return ($version + '  ' + $Code)
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

function Find-Cube {
    <#
  cube.exe is the bundle manager, and it is the whole reason this script
        can install the ST toolchain without a browser.
        itself: it ships inside the stm32cube-ide-core VS Code extension,
        which
        is why the extension has to be installed before the toolchain can
        be.
#>
    $roots = @(
        (Join-Path $env:USERPROFILE '.vscode\extensions'),
        (Join-Path $env:USERPROFILE '.vscode-insiders\extensions'),
        (Join-Path $env:USERPROFILE '.vscode-server\extensions')
    )
    foreach ($root in $roots) {
        if (-not (Test-Path $root)) { continue }
        $ext = Get-ChildItem $root -Directory -Filter 'stmicroelectronics.stm32cube-ide-core-*' `
                             -ErrorAction SilentlyContinue |
               Sort-Object Name -Descending | Select-Object -First 1
        if ($null -eq $ext) { continue }
        $exe = Join-Path $ext.FullName 'resources\binaries\win32\x86_64\cube.exe'
        if (Test-Path $exe) { return $exe }
    }
    return $null
}

function Get-NewestBundle {
    param([string]$Name)
    $dir = Join-Path $BundleRoot $Name
    if (-not (Test-Path $dir)) { return $null }
    return Get-ChildItem $dir -Directory -ErrorAction SilentlyContinue |
        Sort-Object -Property @{Expression = {
            $stem = ($_.Name -split '\+')[0]
            try { [version]$stem } catch { [version]'0.0.0' }
        }} -Descending | Select-Object -First 1
}

# ---- 1. the machine itself -------------------------------------------------

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

    # The LTSpice models are a submodule with its deploy key on the bench
    # machine's GitLab account - a clone elsewhere fails at fetch, not at
    # checkout, and everything that needs its numbers reads them from
    # coaxial/model/inverter.py, which carries the traced constants in-tree.
    if ($null -ne $git) {
        $sub = (& $git -C $Root submodule status electronic_simulations 2>$null)
        if ($null -ne $sub -and $sub -match '^-') {
            Write-Item 'electronic_simulations' 'missing' 'optional - LTSpice sources; the traced constants are in coaxial/model/inverter.py'
            Add-Todo -Optional 'git submodule update --init electronic_simulations   (needs the GitLab SSH key)'
        } elseif ($null -ne $sub) {
            Write-Item 'electronic_simulations' 'ok' 'submodule checked out'
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

# ---- 2. python packages ----------------------------------------------------

function Install-PythonDeps {
    param([string]$Python)
    Write-Head 'python packages'
    if ($null -eq $Python) { return }

    $requirements = Join-Path $Host_ 'requirements.txt'
    if (-not (Test-Path $requirements)) {
        Write-Item 'requirements.txt' 'failed' $requirements
        return
    }

    # The list to probe comes from requirements.txt itself, never a copy here:
    # a copy drifted once - numpy and rich joined the file and this probe kept
    # saying 'all present' on a machine that had neither.
    $absent = Invoke-Python -Python $Python -Code @'
import importlib.util as util
import re

RENAMED = {'pyserial': 'serial', 'pyyaml': 'yaml'}
missing = []
for line in open(r'REQUIREMENTS_PATH', encoding='utf-8'):
    line = line.split('#')[0].strip()
    if not line:
        continue
    dist = re.split(r'[<>=!~\[ ]', line)[0]
    module = RENAMED.get(dist.lower(), dist.lower().replace('-', '_'))
    if util.find_spec(module) is None:
        missing.append(dist)
print(','.join(missing))
'@.Replace('REQUIREMENTS_PATH', $requirements)

    if ([string]::IsNullOrWhiteSpace($absent)) {
        Write-Item 'requirements' 'ok' 'all present'
    } else {
        Write-Item 'requirements' 'missing' $absent
        if (Confirm-Step "pip install -r host/requirements.txt ?") {
            & $Python -m pip install --disable-pip-version-check -r $requirements
            if ($LASTEXITCODE -eq 0) {
                Write-Item 'requirements' 'done' 'installed'
            } else {
                Write-Item 'requirements' 'failed' "pip exit $LASTEXITCODE"
                Add-Todo "pip install -r host/requirements.txt failed - read the output above"
            }
        } else {
            Add-Todo "python -m pip install -r host/requirements.txt"
        }
    }

    # Required: every script, test and view imports the installed packages
    # (nothing sets sys.path); pyproject also hands out `coaxial`, `coaxial-dbg`
    # and `coaxial-mcp` as commands.
    $installed = Invoke-Python -Python $Python -Code @'
try:
    from importlib.metadata import version
    print(version('coaxial63100'))
except Exception:
    print('')
'@
    if (-not [string]::IsNullOrWhiteSpace($installed)) {
        Write-Item 'pip install -e host/' 'ok' ('coaxial63100 ' + $installed)
    } else {
        Write-Item 'pip install -e host/' 'missing' 'required: scripts, tests and views import it'
        if (Confirm-Step 'pip install -e host/ ?  (editable: the checkout stays the source)') {
            & $Python -m pip install --disable-pip-version-check -e $Host_
            if ($LASTEXITCODE -eq 0) {
                Write-Item 'pip install -e host/' 'done' 'installed'
            } else {
                Write-Item 'pip install -e host/' 'failed' "pip exit $LASTEXITCODE"
                Add-Todo 'pip install -e host/ failed - read the output above'
            }
        } else {
            Add-Todo 'python -m pip install -e host/'
        }
    }

    # The notebooks name their kernel, `coaxial_63100`, registered on this
    # interpreter by make_notebooks.py, so an editor holding two CPythons of
    # the same version opens them on the one the packages are in.
    $maker = Join-Path $Host_ 'tools\notebooks\make_notebooks.py'
    $kernel = (& $Python $maker --kernel status) -join ' '
    if ($LASTEXITCODE -eq 0) {
        Write-Item 'notebook kernel' 'ok' $kernel
    } else {
        Write-Item 'notebook kernel' 'missing' $kernel
        if (Confirm-Step 'register the notebook kernel on this python ?  (one kernel.json under %APPDATA%\jupyter)') {
            $kernel = (& $Python $maker --kernel install) -join ' '
            if ($LASTEXITCODE -eq 0) {
                Write-Item 'notebook kernel' 'done' $kernel
            } else {
                Write-Item 'notebook kernel' 'failed' $kernel
                Add-Todo 'python tools/notebooks/make_notebooks.py --kernel install failed - run it from host/ and read the output'
            }
        } else {
            Add-Todo 'python tools/notebooks/make_notebooks.py --kernel install   (from host/)'
        }
    }
}

# ---- 3. the ST toolchain ---------------------------------------------------

# Bundle name -> the executable that proves it arrived whole.
$BundleNeeds = [ordered]@{
    'gnu-tools-for-stm32' = 'arm-none-eabi-gcc.exe'
    'cmake'               = 'cmake.exe'
    'ninja'               = 'ninja.exe'
    'programmer'          = 'STM32_Programmer_CLI.exe'
    'gnu-gdb-for-stm32'   = 'arm-none-eabi-gdb.exe'
    'stlink-gdbserver'    = ''
    'stlink-server'       = ''
    'stlink-usb-driver'   = ''
}

# Not in the table above because it is 835 MB on disk and nothing in the build
# reaches for it: CubeMX regenerates core/ from the .ioc and is otherwise idle.
if (-not $SkipCubeMX) { $BundleNeeds['stm32cubemx-application'] = '' }

# ONE EXTENSION LIST, the same way pyproject.toml has one dependency list.
$ExtensionWhy = @{
    'stmicroelectronics.stm32-vscode-extension' = 'the toolchain, the debuggers and cube.exe'
    'ms-vscode.cpptools'                        = 'IntelliSense over the HAL'
    'ms-python.python'                          = 'host/'
    'ms-toolsai.jupyter'                        = 'notebook_examples/ in the editor'
    'ollama.ollama'                             = "the local model in VS Code's chat picker"
    'anthropic.claude-code'                     = "the chooser's BOARD CHAT page; .mcp.json wires it to the board"
}

function Get-RecommendedExtensions {
    <#
  The ids out of .vscode/extensions.json.
        ConvertFrom-Json will not read that file as it stands - it is JSONC,
        VS Code allows // comments and this repository uses them to say why
        each id is there.
        file puts one after code, and a // inside a string would have to be
        a
        URL, which an extension id cannot contain.
        Ollama.ollama is dropped here on purpose.
        editor half of the same daemon - so -SkipOllama still leaves the two
        out together, which is what that switch promises.
        A tree without the file still gets the three without which nothing
        in
        this repository resolves, rather than a setup that installs nothing.
#>

    $fallback = @(
        'stmicroelectronics.stm32-vscode-extension',
        'ms-vscode.cpptools',
        'ms-python.python'
    )

    $path = Join-Path $Root '.vscode\extensions.json'
    if (-not (Test-Path $path)) { return $fallback }

    try {
        $stripped = (Get-Content $path) | Where-Object { $_ -notmatch '^\s*//' }
        $json = ($stripped -join "`n") | ConvertFrom-Json
    } catch {
        Write-Item 'extensions.json' 'failed' ('unreadable: ' + $_.Exception.Message)
        return $fallback
    }

    $ids = @($json.recommendations)
    if ($ids.Count -eq 0) { return $fallback }
    return @($ids | Where-Object { $_ -ne 'ollama.ollama' })
}

$Extensions = Get-RecommendedExtensions

function Test-Bundles {
    Write-Head 'ST toolchain (bundles under %LOCALAPPDATA%\stm32cube)'
    $absent = @()
    foreach ($name in $BundleNeeds.Keys) {
        $bundle = Get-NewestBundle $name

        # CubeMX is the one entry here that has a second, equally valid home:
        # st.com's installer puts it under Program Files.
        if (($null -eq $bundle) -and ($name -eq 'stm32cubemx-application')) {
            $standalone = Find-CubeMX
            if ($null -ne $standalone) {
                Write-Item $name 'ok' ('installed from st.com: ' + $standalone)
                continue
            }
        }

        if ($null -eq $bundle) {
            Write-Item $name 'missing' ''
            $absent += $name
            continue
        }
        $exe = $BundleNeeds[$name]
        $detail = $bundle.Name
        if ($exe -ne '') {
            $path = Join-Path $bundle.FullName ('bin\' + $exe)
            if (-not (Test-Path $path)) { $detail = $bundle.Name + ' (no ' + $exe + ')' }
        }
        Write-Item $name 'ok' $detail
    }
    return $absent
}

function Install-VsCodeExtensions {
    param([string[]]$Absent)

    $code = Find-Code
    if ($null -eq $code) {
        Write-Item 'vs code' 'missing' 'Microsoft.VisualStudioCode'
        if (Install-WingetPackage -Id 'Microsoft.VisualStudioCode' `
                                  -Why 'it carries cube.exe, which fetches the ST toolchain') {
            $code = Find-Code
        }
        if ($null -eq $code) {
            Add-Todo ('install VS Code and run this script again - or re-run with ' +
                      '-WingetToolchain for a build toolchain that does not need it')
            return
        }
        Write-Item 'vs code' 'done' (Get-CodeVersion $code)
    } else {
        Write-Item 'vs code' 'ok' (Get-CodeVersion $code)

        # Installed is not current, and here that is not cosmetic: the STM32
        # pack is version-matched to the editor, and an editor left behind gets
        # extensions that decline to load rather than an error saying why.
        Update-WingetPackage -Id 'Microsoft.VisualStudioCode' -What 'vs code' `
                             -Why 'the STM32 pack is version-matched to the editor' | Out-Null
    }

    # --list-extensions is one call and it is not a fast one, so it is asked
    # once here rather than per extension.
    $installed = (& $code --list-extensions 2>$null)
    foreach ($id in $Extensions) {
        $why = $ExtensionWhy[$id]
        if ($null -eq $why) { $why = '' }
        if ($installed -contains $id) {
            Write-Item $id 'ok' $why
            continue
        }
        Write-Item $id 'missing' $why
        $prompt = "code --install-extension $id ?"
        if ($why -ne '') { $prompt = $prompt + "  ($why)" }
        if (Confirm-Step $prompt) {
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

    # The editor's configuration is checked in, so on a full clone it is
    # already right and there is nothing to install: the Board chat terminal
    # profile, Ctrl+Shift+B, IntelliSense over the HAL and the MCP server all
    # arrive with the repository.
    $config = @(
        '.vscode\settings.json',
        '.vscode\tasks.json',
        '.vscode\extensions.json',
        '.vscode\c_cpp_properties.json',
        '.mcp.json'
    )
    $gone = @($config | Where-Object { -not (Test-Path (Join-Path $Root $_)) })
    if ($gone.Count -eq 0) {
        Write-Item 'workspace config' 'ok' 'terminal profile, tasks, IntelliSense, MCP server'
    } else {
        Write-Item 'workspace config' 'missing' ($gone -join ', ')
        Add-Todo ('these are checked in and absent here - restore them with ' +
                  '`git checkout -- ' + ($gone -join ' ') + '`')
    }

    Install-Bundles -Absent $Absent
}

function Install-Bundles {
    <#
  The ST toolchain, without a browser.
        `cube bundle install --yes NAME` downloads from developer.st.com and
        unpacks under %LOCALAPPDATA%, no account and no click-through, and
        the
        --yes answers the licence question the same way opening the bundles
        manager in VS Code would.
        set up and not finish; it finishes it now.
        One bundle at a time on purpose.
        fails whole, and the failure that matters here is a 300 MB download
        dropping at 90% on bench wifi - the other seven should still be in
        place afterwards.
#>
    param([string[]]$Absent)

    if ($Absent.Count -eq 0) { return }

    $cube = Find-Cube
    if ($null -eq $cube) {
        Write-Item 'cube' 'missing' 'comes with the stm32cube-ide-core extension'
        Add-Todo ('the STM32 extension was just installed but its cube.exe is not ' +
                  'on disk yet - open VS Code once to let it unpack, then run this ' +
                  'script again to fetch: ' + ($Absent -join ', '))
        return
    }
    Write-Item 'cube' 'ok' $cube

    if ($Check) {
        Write-Item 'bundles' 'missing' ($Absent -join ', ')
        Add-Todo ('run without -Check to fetch: ' + ($Absent -join ', ') +
                  '  (cube bundle install --yes ...)')
        return
    }

    foreach ($name in $Absent) {
        $size = ''
        if ($name -eq 'stm32cubemx-application') { $size = '308 MB down, 835 MB on disk' }
        $prompt = "cube bundle install $name ?"
        if ($size -ne '') { $prompt = $prompt + "  ($size)" }
        if (-not (Confirm-Step $prompt)) {
            Add-Todo "cube bundle install --yes $name"
            continue
        }
        & $cube bundle install --yes $name
        # cube's exit code is not the whole answer: it is 0 for "nothing to do"
        # as well as for a completed install.
        if ($null -ne (Get-NewestBundle $name)) {
            Write-Item $name 'done' (Get-NewestBundle $name).Name
        } else {
            Write-Item $name 'failed' "cube exit $LASTEXITCODE"
            Add-Todo "cube bundle install --yes $name   (failed once already)"
        }
    }
}

function Install-StLinkDriver {
    <#
  The USB driver, which is the one thing here that needs administrator
        rights - so it is asked for on its own rather than folded into the
        bundle loop above, and -SkipDriver turns it off outright.
        Without it Windows enumerates the ST-Link as an unknown device: the
        VCP
        appears, so `board all` finds a COM port, while SWD does not, so
        flashing fails with a probe error that reads like a cable fault.
        the elevation prompt.
#>

    if ($SkipDriver) { return }

    $cube = Find-Cube
    if ($null -eq $cube) { return }
    if ($null -eq (Get-NewestBundle 'stlink-usb-driver')) { return }

    $out = (& $cube stlink-usb-driver-install-check 2>&1) -join "`n"
    if ($out -match 'stlink') {
        Write-Item 'st-link usb driver' 'ok' 'installed'
        return
    }
    Write-Item 'st-link usb driver' 'missing' 'needs administrator'
    if (-not (Confirm-Step 'install the ST-Link USB driver ?  (elevation prompt)')) {
        Add-Todo "cube stlink-usb-driver-adm-install   (run from an elevated shell)"
        return
    }
    try {
        $proc = Start-Process -FilePath $cube -ArgumentList 'stlink-usb-driver-adm-install' `
                              -Verb RunAs -Wait -PassThru
        if ($proc.ExitCode -eq 0) {
            Write-Item 'st-link usb driver' 'done' 'installed'
        } else {
            Write-Item 'st-link usb driver' 'failed' ("exit " + $proc.ExitCode)
            Add-Todo 'cube stlink-usb-driver-adm-install   (run from an elevated shell)'
        }
    } catch {
        Write-Item 'st-link usb driver' 'failed' $_.Exception.Message
        Add-Todo 'cube stlink-usb-driver-adm-install   (run from an elevated shell)'
    }
}

function Find-CubeMX {
    <#
  STM32CubeMX.exe, from either kind of install.
        The bundle is the tidy one and what this script installs.
        who took the installer from st.com has it under Program Files
        instead,
        and telling that machine CubeMX is missing would be this script
        being
        wrong in a way the user can see.
#>

    $bundle = Get-NewestBundle 'stm32cubemx-application'
    if ($null -ne $bundle) {
        $exe = Get-ChildItem $bundle.FullName -Recurse -Filter 'STM32CubeMX*.exe' `
                             -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($null -ne $exe) { return $exe.FullName }
    }
    $candidates = @(
        (Join-Path $env:ProgramFiles 'STMicroelectronics\STM32Cube\STM32CubeMX\STM32CubeMX.exe'),
        (Join-Path ${env:ProgramFiles(x86)} 'STMicroelectronics\STM32Cube\STM32CubeMX\STM32CubeMX.exe'),
        (Join-Path $env:LOCALAPPDATA 'Programs\STMicroelectronics\STM32Cube\STM32CubeMX\STM32CubeMX.exe')
    )
    foreach ($c in $candidates) {
        if (($null -ne $c) -and (Test-Path $c)) { return $c }
    }
    return $null
}

function Install-CubeMXFromInstaller {
    <#
  The other way to get CubeMX: st.com's own installer.
        The bundle above is 308 MB and needs nothing but cube.exe, so it is
        what this script reaches for first.
        where that failed, or where the bundle is not wanted - and for the
        person who would rather see the licence they are agreeing to.
        -CubeMXInstaller runs an installer already on disk.
        download page is opened and the run carries on: a missing CubeMX
        stops
        nothing here.
        model - only to regenerate core/ from the .ioc.
#>

    if ($SkipCubeMX) { return }
    if ($null -ne (Find-CubeMX)) { return }

    if ($CubeMXInstaller) {
        if (-not (Test-Path $CubeMXInstaller)) {
            Write-Item 'STM32CubeMX' 'failed' ('no such path: ' + $CubeMXInstaller)
            Add-Todo ('-CubeMXInstaller pointed at ' + $CubeMXInstaller + ', which does not exist')
            return
        }
        if ($Check) {
            Write-Item 'STM32CubeMX' 'missing' ('would run ' + $CubeMXInstaller)
            return
        }
        # ST's installer is a GUI with a licence page in it.
        Write-Item 'STM32CubeMX' 'missing' ('running ' + $CubeMXInstaller)
        try {
            Start-Process -FilePath $CubeMXInstaller -Wait
        } catch {
            Write-Item 'STM32CubeMX' 'failed' $_.Exception.Message
            Add-Todo ('running ' + $CubeMXInstaller + ' failed - read the error above')
            return
        }
        $found = Find-CubeMX
        if ($null -ne $found) {
            Write-Item 'STM32CubeMX' 'done' $found
        } else {
            Write-Item 'STM32CubeMX' 'failed' 'the installer ran, but no STM32CubeMX.exe was found'
            Add-Todo 'STM32CubeMX still not found after running the installer - check where it put itself'
        }
        return
    }

    Write-Item 'STM32CubeMX' 'missing' 'not as a bundle, not under Program Files'
    if (Confirm-Step 'open the STM32CubeMX download page in a browser ?') {
        Start-Process $CubeMXUrl
        Write-Item 'download page' 'done' $CubeMXUrl
    }
    Add-Todo ($CubeMXUrl + '  -> download the installer, then: setup.ps1 -CubeMXInstaller <the exe>' +
              '   (or let the bundle route fetch it: cube bundle install --yes stm32cubemx-application)')
}

function Find-CubeIDE {
    <#
  stm32cubeide.exe, the Eclipse-based IDE - not the STM32 VS Code
        extension, and not a cube.exe bundle.
        it, which defaults to C:\ST\STM32CubeIDE_x.y.z but lets the user
        pick
        anywhere, so this searches rather than assumes a single path.
#>

    $roots = @(
        'C:\ST',
        $env:ProgramFiles,
        ${env:ProgramFiles(x86)}
    )
    foreach ($root in $roots) {
        if (($null -eq $root) -or (-not (Test-Path $root))) { continue }
        $exe = Get-ChildItem $root -Recurse -Depth 3 -Filter 'stm32cubeide.exe' `
                             -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($null -ne $exe) { return $exe.FullName }
    }
    return $null
}

function Install-CubeIDE {
    <#
  STM32CubeIDE, which nothing in this project's own workflow needs - the
        documented route is VS Code plus cube-cmake.
        it was asked for: whoever wants the Eclipse-based IDE alongside that
        workflow can have this step fetch it, the same way
        Install-CubeMXFromInstaller
        does for CubeMX.
        ST puts this behind a login and a click-through licence on st.com,
        the
        same as CubeMX, so there is no unattended download here either: the
        page opens, a human takes it from there, and -CubeIDEInstaller is
        the
        way back in once the installer is on disk.
#>

    if ($SkipCubeIDE) { return }

    Write-Head 'STM32CubeIDE (optional - not part of this project''s own workflow)'

    $found = Find-CubeIDE
    if ($null -ne $found) {
        Write-Item 'STM32CubeIDE' 'ok' $found
        return
    }

    if ($CubeIDEInstaller) {
        if (-not (Test-Path $CubeIDEInstaller)) {
            Write-Item 'STM32CubeIDE' 'failed' ('no such path: ' + $CubeIDEInstaller)
            Add-Todo ('-CubeIDEInstaller pointed at ' + $CubeIDEInstaller + ', which does not exist')
            return
        }
        if ($Check) {
            Write-Item 'STM32CubeIDE' 'missing' ('would run ' + $CubeIDEInstaller)
            return
        }
        # ST's installer is a GUI with a licence page in it.
        Write-Item 'STM32CubeIDE' 'missing' ('running ' + $CubeIDEInstaller)
        try {
            Start-Process -FilePath $CubeIDEInstaller -Wait
        } catch {
            Write-Item 'STM32CubeIDE' 'failed' $_.Exception.Message
            Add-Todo ('running ' + $CubeIDEInstaller + ' failed - read the error above')
            return
        }
        $found = Find-CubeIDE
        if ($null -ne $found) {
            Write-Item 'STM32CubeIDE' 'done' $found
        } else {
            Write-Item 'STM32CubeIDE' 'failed' 'the installer ran, but no stm32cubeide.exe was found'
            Add-Todo 'STM32CubeIDE still not found after running the installer - check where it put itself'
        }
        return
    }

    Write-Item 'STM32CubeIDE' 'missing' 'needs a login on st.com'
    if ($Check) {
        Add-Todo -Optional ($CubeIDEUrl + '  -> download the installer, then: setup.ps1 -CubeIDEInstaller <the exe>')
        return
    }
    if (Confirm-Step 'open the STM32CubeIDE download page in a browser ?') {
        Start-Process $CubeIDEUrl
        Write-Item 'download page' 'done' $CubeIDEUrl
    }
    Add-Todo -Optional ($CubeIDEUrl + '  -> download the installer, then: setup.ps1 -CubeIDEInstaller <the exe>')
}

function Get-FirmwareVersion {
    <#
  Which FW_H7 the .ioc was generated against.
        Read from the project rather than pinned here, because CubeMX writes
        it
        and CubeMX is the thing that will complain about it.
            ProjectManager.FirmwarePackage=STM32Cube FW_H7 V1.13.0
#>

    $ioc = Join-Path $Root 'coaxial_63100.ioc'
    if (-not (Test-Path $ioc)) { return $null }
    $line = Select-String -Path $ioc -Pattern '^ProjectManager\.FirmwarePackage=' `
                          -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($null -eq $line) { return $null }
    if ($line.Line -match 'V([0-9]+\.[0-9]+\.[0-9]+)') { return $Matches[1] }
    return $null
}

function Install-FirmwarePackage {
    <#
  The STM32Cube FW_H7 package, which is the one thing here that ST still
        puts behind a login.
        First, what it is and is not for.
        is in this repository, so gcc has its HAL and CMSIS whether or not
        this
        step ever runs.
        repository that has no FW_H7 V1.13.0 in it means CubeMX offers to
        download one, and regenerating against a different version is a
        different core/ than the one in git.
        Three ways in, in the order they are worth trying:
          1.
             share, a stick or another bench, this needs no browser and no
             account.
          2.
             is already installed by the bundle step above.
             st.com account; if it does, so be it - that is ST's decision,
             not
             this script's.
          3.
             licence, downloads the zip, and hands it back through 1.
        What this function will not do is scrape a download out of st.com
        with
        a fabricated user agent.
        (sw-center.st.com) answer 404 now, and the GitHub mirror is
        submodules -
        `drivers/STM32H7xx_HAL_Driver` is a gitlink there, so a zipball of
        the
        tag contains none of the sources that matter.
#>

    if ($SkipFirmware) { return }

    Write-Head 'STM32Cube FW_H7 (CubeMX only - the build does not need it)'

    $version = Get-FirmwareVersion
    if ($null -eq $version) {
        Write-Item 'firmware package' 'ok' 'the .ioc names none'
        return
    }
    $name = 'STM32Cube_FW_H7_V' + $version
    $target = Join-Path $Repository $name

    if (Test-Path $target) {
        Write-Item $name 'ok' $target
        return
    }
    Write-Item $name 'missing' ('not in ' + $Repository)

    # 1. handed to us
    if ($FirmwarePackage) {
        if (-not (Test-Path $FirmwarePackage)) {
            Write-Item 'firmware package' 'failed' ('no such path: ' + $FirmwarePackage)
            Add-Todo ('-FirmwarePackage pointed at ' + $FirmwarePackage + ', which does not exist')
            return
        }
        if ($Check) {
            Write-Item 'firmware package' 'missing' ('would install from ' + $FirmwarePackage)
            return
        }
        if (-not (Test-Path $Repository)) {
            New-Item -ItemType Directory -Path $Repository -Force | Out-Null
        }
        try {
            if ((Get-Item $FirmwarePackage) -is [System.IO.DirectoryInfo]) {
                Copy-Item $FirmwarePackage -Destination $target -Recurse -Force
            } else {
                # The zip carries its own STM32Cube_FW_H7_Vx.y.z top directory,
                # so it expands into the repository rather than into the
                # target.
                Expand-Archive -Path $FirmwarePackage -DestinationPath $Repository -Force
            }
        } catch {
            Write-Item 'firmware package' 'failed' $_.Exception.Message
            Add-Todo ('unpacking ' + $FirmwarePackage + ' failed - read the error above')
            return
        }
        if (Test-Path $target) {
            Write-Item $name 'done' $target
        } else {
            Write-Item $name 'failed' ('unpacked, but there is no ' + $name + ' in ' + $Repository)
            Add-Todo ('the package unpacked under a different name - rename it to ' + $name)
        }
        return
    }

    # 2.
    $mx = Get-NewestBundle 'stm32cubemx-application'
    if ($null -ne $mx) {
        Write-Item 'STM32CubeMX' 'ok' 'its package manager can fetch this one'
        Add-Todo -Optional ('open CubeMX (the `cubemx` command) and let it install ' + $name +
                            ' - Help > Manage embedded software packages')
    }

    # 3. the page
    if (Confirm-Step 'open the STM32CubeH7 download page in a browser ?') {
        Start-Process $CubeH7Url
        Write-Item 'download page' 'done' $CubeH7Url
    }
    Add-Todo -Optional ($CubeH7Url + '  -> download ' + $name +
                        ', then: setup.ps1 -FirmwarePackage <the zip>')
}

function Install-WingetToolchain {
    Write-Head 'toolchain from winget'
    $packages = [ordered]@{
        'Kitware.CMake'               = 'cmake'
        'Ninja-build.Ninja'           = 'ninja'
        'Arm.GnuArmEmbeddedToolchain' = 'arm-none-eabi-gcc'
    }
    foreach ($id in $packages.Keys) {
        $exe = $packages[$id]
        if ($null -ne (Get-Tool $exe)) {
            Write-Item $exe 'ok' (Get-Tool $exe)
            continue
        }
        Write-Item $exe 'missing' $id
        if (Confirm-Step "winget install $id ?") {
            winget install --id $id --exact --accept-package-agreements --accept-source-agreements
            Write-Item $exe 'done' 'installed - open a new shell for PATH'
        } else {
            Add-Todo "winget install --id $id --exact"
        }
    }
    # No winget package publishes STM32CubeProgrammer.
    $programmer = Get-NewestBundle 'programmer'
    if ($null -ne $programmer) {
        Write-Item 'STM32_Programmer_CLI' 'ok' $programmer.Name
    } elseif ($null -ne (Find-Cube)) {
        Install-Bundles -Absent @('programmer')
    } else {
        Write-Item 'STM32_Programmer_CLI' 'manual' 'not on winget, and no cube.exe here'
        Add-Todo ('STM32CubeProgrammer has to come from st.com by hand (free ' +
                  'account, click-through licence). Install it, then add its bin ' +
                  'directory to PATH - or drop -WingetToolchain and let the ' +
                  'bundle manager fetch it.')
    }
}

# ---- 4. ollama, the model, and the editor extension ------------------------

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
    $machine = $picked.machine
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

# ---- 5. does any of it work -----------------------------------------------

function Test-Setup {
    param([string]$Python)
    Write-Head 'checks'
    if ($null -eq $Python) { return }

    Push-Location $Host_
    try {
        # Offline by design: no board, no ollama.
        $output = (& $Python 'tests/test_ollama_tools.py')
        $tail = ($output | Select-Object -Last 1)
        if ($LASTEXITCODE -eq 0) {
            Write-Item 'host test suite' 'ok' $tail
        } else {
            Write-Item 'host test suite' 'failed' $tail
            Add-Todo 'python tests/test_ollama_tools.py failed - run it and read the output'
        }
    } finally {
        Pop-Location
    }

    $ports = @()
    try {
        $ports = [System.IO.Ports.SerialPort]::GetPortNames()
    } catch {
        $ports = @()
    }
    if ($ports.Count -gt 0) {
        Write-Item 'serial ports' 'ok' ($ports -join ', ')
    } else {
        Write-Item 'serial ports' 'missing' 'no COM port - is the ST-Link plugged in?'
    }

    # Not Get-Tool: cube-cmake is only on PATH after env.ps1 has run, and this
    # script deliberately does not touch PATH.
    $cmake = $null
    $ext = Get-ChildItem (Join-Path $env:USERPROFILE '.vscode\extensions') -Directory `
                         -Filter 'stmicroelectronics.stm32cube-ide-build-cmake-*' `
                         -ErrorAction SilentlyContinue |
           Sort-Object Name -Descending | Select-Object -First 1
    if ($null -ne $ext) {
        $candidate = Join-Path $ext.FullName 'resources\cube-cmake\win32\x86_64\cube-cmake.exe'
        if (Test-Path $candidate) { $cmake = $candidate }
    }
    if ($null -eq $cmake) { $cmake = Get-Tool 'cube-cmake' }
    if ($null -eq $cmake) {
        Write-Item 'cube-cmake' 'missing' 'comes with the VS Code extension'
    } else {
        Write-Item 'cube-cmake' 'ok' $cmake
    }

    if (-not $SkipCubeMX) {
        # Find-CubeMX, so a standalone install from st.com counts as much as
        # the bundle.
        $mxExe = Find-CubeMX
        if ($null -eq $mxExe) {
            Write-Item 'STM32CubeMX' 'missing' 'cube bundle install --yes stm32cubemx-application'
        } else {
            Write-Item 'STM32CubeMX' 'ok' $mxExe
        }
    }
}

# ---- main ------------------------------------------------------------------

Write-Host ''
Write-Host 'coaxial_63100 setup' -ForegroundColor White
Write-Host ("  " + $Root) -ForegroundColor DarkGray
if ($Check) {
    Write-Host '  -Check: reporting only, nothing will be installed' -ForegroundColor DarkGray
}

# Asked once, at the top, rather than left to be discovered a dozen prompts in.
if ((-not $Check) -and (-not $Yes)) {
    Write-Host ''
    Write-Host '  Unattended, or one question per step?' -ForegroundColor White
    Write-Host '    y  install everything that is missing without asking again' -ForegroundColor DarkGray
    Write-Host '    n  ask before each install  (default)' -ForegroundColor DarkGray
    if (-not $SkipCubeMX) {
        Write-Host '    note: unattended includes STM32CubeMX, 308 MB down and 835 MB on disk.' -ForegroundColor DarkGray
        Write-Host '          -SkipCubeMX leaves it out.' -ForegroundColor DarkGray
    }
    if (-not $SkipDriver) {
        Write-Host '    note: the ST-Link USB driver still raises its own elevation prompt.' -ForegroundColor DarkGray
        Write-Host '          Windows asks that one; this script cannot answer it for you.' -ForegroundColor DarkGray
    }
    if (Read-YesNo '  unattended? [y/N]') {
        $Yes = $true
        Write-Host '  unattended: nothing below will ask.' -ForegroundColor Green
    } else {
        Write-Host '  interactive: every install is a separate y/N.' -ForegroundColor DarkGray
    }
}

$python = Test-Machine
Install-PythonDeps -Python $python

if ($WingetToolchain) {
    Install-WingetToolchain
} else {
    $absent = Test-Bundles
    Install-VsCodeExtensions -Absent $absent
    Install-StLinkDriver
}

# Outside the branch above on purpose: -WingetToolchain is the case where no
# bundle route exists, so the installer and the download page are the only ways
# CubeMX arrives at all.
Install-CubeMXFromInstaller
Install-FirmwarePackage
Install-CubeIDE

if (-not $SkipOllama) {
    Install-Ollama -Python $python
} 

Test-Setup -Python $python

Write-Head 'next'
if ($script:Todo.Count -eq 0) {
    Write-Host '  nothing outstanding.' -ForegroundColor Green
} else {
    $n = 0
    foreach ($item in $script:Todo) {
        $n = $n + 1
        Write-Host ("  {0}. {1}" -f $n, $item) -ForegroundColor Yellow
    }
}
if ($script:Optional.Count -gt 0) {
    Write-Host ''
    Write-Host '  optional, nothing here needs them:' -ForegroundColor DarkGray
    foreach ($item in $script:Optional) {
        Write-Host ("    - " + $item) -ForegroundColor DarkGray
    }
}

Write-Host ''
Write-Host '  every shell:' -ForegroundColor White
Write-Host '    . .\env.ps1                 tools on PATH, plus board_chat/dbg/board/cbuild/cflash/cubemx'
Write-Host ''
Write-Host '  then:' -ForegroundColor White
Write-Host '    cbuild                      build, zero warnings expected'
Write-Host '    cflash                      flash over SWD and start'
Write-Host '    board all                   measure, no model involved'
Write-Host '    dbg "why is the NTC 25.00?" ask the local model, cheaply'
Write-Host '    board_chat                  a prompt with the model and the board in it'
Write-Host '    cubemx                      open the .ioc in STM32CubeMX'
Write-Host ''
if ($Check) {
    Write-Host '  run again without -Check to install what is missing.' -ForegroundColor DarkGray
    Write-Host ''
}

# The exit code is the whole answer for anything driving this without reading
# it: 0 means the environment is ready, non-zero means the numbered list above
# is not empty.
exit $(if ($script:Todo.Count -gt 0) { 1 } else { 0 })
