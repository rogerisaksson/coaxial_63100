<#
.SYNOPSIS
    Sets up a machine to build, flash and drive the coaxial_63100 board.

        powershell -ExecutionPolicy Bypass -File .\setup.ps1 -Check
        powershell -ExecutionPolicy Bypass -File .\setup.ps1

.DESCRIPTION
    Run -Check first. It changes nothing and prints what is present, what is
    missing and what each missing thing will cost to fix, which is the question
    you actually have on a fresh machine.

    What this project needs, and where each part comes from:

      Python, git and VS Code           winget
      four python packages              host/requirements.txt, via pip
      ARM gcc, cmake, ninja             STM32 "bundles", fetched by cube.exe
      STM32_Programmer_CLI             same
      STM32CubeMX                       same - the stm32cubemx-application bundle
      ST-Link gdbserver, server, driver same
      cube-cmake                        the STM32 VS Code extension
      ollama and one model              winget or ollama.com/install.ps1,
                                        then `ollama pull`
      the VS Code extensions            code --install-extension: the STM32
                                        pack, cpptools, python, and Ollama's
                                        own - see .vscode/extensions.json
      STM32Cube FW_H7                   st.com, and only for CubeMX. The build
                                        has Drivers/ in the repository.

    The ST half used to be the awkward half. ST publishes nothing to winget and
    the installers on st.com sit behind a login and a click-through licence, so
    for a while the honest answer was "open VS Code once and let its bundle
    manager download the toolchain". It is not any more. The STM32 VS Code
    extension ships `cube.exe`, and

        cube bundle install --yes gnu-tools-for-stm32 programmer ...

    pulls exactly the same bundles from developer.st.com with no account, no
    browser and no click. That is what this script drives now, and it is why a
    fresh machine no longer needs a human in the middle of the toolchain
    install. The one thing that still has to arrive first is the extension
    itself, which `code --install-extension` handles.

    The whole of it, on a machine with nothing but Windows and winget:

        powershell -ExecutionPolicy Bypass -File .\setup.ps1 -Yes -AllowScripts

    Python and VS Code arrive through winget, and a winget install only reaches
    the PATH of shells opened after it - so on a bare machine the first run
    installs those and asks to be run once more. The second run is the one that
    finishes; every run after that reports 'nothing outstanding' and changes
    nothing.

    If you would rather not have VS Code in the loop, -WingetToolchain installs
    cmake, ninja and Arm's own gcc from winget instead. The build works; the
    programmer still has to come from ST by hand, and the note at the end says so
    rather than leaving you to find out at flashing time.

    Nothing here needs administrator rights except the winget installs, and
    nothing is written to the system PATH. Per-shell PATH is env.ps1's job.

.PARAMETER Check
    Report only. No installs, no downloads, no writes.

.PARAMETER Yes
    Do not ask before each install. Without it the script opens by asking once
    whether to run unattended, so this switch is the way to answer that question
    from a command line - in a scheduled run, or from another script.

.PARAMETER Model
    The ollama tag to pull. Left alone, the machine decides: cores, RAM and the
    size of the graphics card are measured and the largest tools-capable model
    that fits the card whole is chosen - see host/coaxial_ollama/capability.py,
    or ask it yourself with

        python -m coaxial_ollama.capability

    Pass a tag to overrule that. It has to be a tools-capable one, because
    everything in host/coaxial_ollama reaches the board through tool calls and
    a model without them will describe a measurement instead of taking one.

.PARAMETER Prefer
    What the automatic choice optimises for. 'speed' takes the largest model
    that fits the graphics card whole. 'capability' allows a larger one to hang
    half out of the card and run the rest on the CPU, which measured about five
    times slower per token - worth it when the answer matters more than the
    wait.

.PARAMETER SkipOllama
    Leave the model side alone - for a machine that only builds and flashes.
    Skips the binary, the model pull and the VS Code extension alike.

.PARAMETER SkipCubeMX
    Do not install STM32CubeMX at all. The bundle is 308 MB down and 835 MB on
    disk and nothing in the build needs it - only regenerating Core/ from the
    .ioc does. On a bench that never opens the .ioc, this is the switch that
    saves the disk.

.PARAMETER CubeMXInstaller
    Run an STM32CubeMX installer that is already on disk, instead of taking the
    bundle. This is the other end of the download page: the script opens
    st.com's CubeMX page when CubeMX is absent and the bundle was not taken,
    you download the installer, and this switch runs it on the next pass.

.PARAMETER SkipDriver
    Do not touch the ST-Link USB driver. Installing it is the one step in this
    script that needs administrator rights, so it is asked for on its own and
    skipped outright here.

.PARAMETER SkipFirmware
    Do not look for the STM32Cube FW_H7 package. Nothing in the build needs it -
    Drivers/ is in this repository - so a machine that only builds and flashes
    can ignore it entirely.

.PARAMETER FirmwarePackage
    A STM32Cube_FW_H7_Vx.y.z.zip, or an already unpacked copy of one, to install
    into the CubeMX repository. This is the way in if you have the package from
    somewhere other than st.com - a share, a memory stick, another bench - and
    it is the only path here that needs no browser and no account.

.PARAMETER Repository
    Where CubeMX keeps its firmware packages. Defaults to
    %USERPROFILE%\STM32Cube\Repository, which is CubeMX's own default; pass this
    if yours was moved.

.PARAMETER WingetToolchain
    Install cmake, ninja and arm-none-eabi-gcc from winget instead of relying on
    the VS Code extension's bundles.

.PARAMETER AllowScripts
    Set the CurrentUser execution policy to RemoteSigned, so that `. .\env.ps1`
    works in an ordinary shell afterwards. Off by default: it is a change to how
    this account runs every script, not just this one, and it is your call.
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
    [string]$Repository = (Join-Path $env:USERPROFILE 'STM32Cube\Repository'),
    [switch]$WingetToolchain,
    [switch]$AllowScripts
)

$ErrorActionPreference = 'Continue'
$Root = $PSScriptRoot
$Host_ = Join-Path $Root 'host'
$BundleRoot = Join-Path $env:LOCALAPPDATA 'stm32cube\bundles'
$CubeH7Url = 'https://www.st.com/en/embedded-software/stm32cubeh7.html'
$CubeMXUrl = 'https://www.st.com/en/development-tools/stm32cubemx.html'
$script:Todo = @()

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
    param([string]$Text)
    $script:Todo += $Text
}

function Confirm-Step {
    param([string]$Text)
    if ($Check) { return $false }
    if ($Yes)   { return $true }
    return (Read-YesNo ("  " + $Text + "  [y/N]"))
}

function Read-YesNo {
    <#  y or n, and no on anything else - including no console at all.

        Read-Host throws when stdin is at end of file, which is what a piped or
        scheduled run looks like. Left unhandled that kills the script halfway
        through, with some things installed and some not; caught, it is simply
        the answer 'no', and the run finishes with the rest on the todo list.
        A run that wants everything says so with -Yes.  #>
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
    <#  Run a snippet through a temp file rather than `python -c`.

        Windows PowerShell 5.1 rewrites the quoting of arguments on their way to
        a native executable, so a -c snippet containing quotes or a % format
        arrives at the interpreter mangled. A file has no quoting to lose.  #>
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
    <#  One winget install, with the three flags that make it non-interactive
        and the one exit code that is not a failure.

        winget answers -1978335189 (0x8A15002B) for "already installed", which
        happens whenever a package is on the machine but its shim has not
        reached this shell's PATH - the exact situation this script is in half
        the time. Treating that as failure would send a working machine away
        with a todo it does not need.  #>
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

function Find-Code {
    <#  `code` is a .cmd shim, and the installer adds its directory to the user
        PATH - which reaches shells opened afterwards, never this one. Look
        where both the user and the machine installer put it before concluding
        VS Code is absent.  #>
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

function Find-Cube {
    <#  cube.exe is the bundle manager, and it is the whole reason this script
        can install the ST toolchain without a browser. It is not a bundle
        itself: it ships inside the stm32cube-ide-core VS Code extension, which
        is why the extension has to be installed before the toolchain can be.  #>
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

    $python = Get-Tool 'python'
    if ($null -eq $python) { $python = Get-Tool 'py' }
    if ($null -eq $python) {
        Write-Item 'python' 'missing' 'Python.Python.3.13'
        if (Install-WingetPackage -Id 'Python.Python.3.13' -Why 'the host library, the CLI and every test') {
            # The installer edits the user PATH, which this process read at
            # start-up and will not read again. Nothing below can use the
            # interpreter that now exists, so say so and stop here.
            Write-Item 'python' 'done' 'installed - not on this shell PATH yet'
            Add-Todo 'open a NEW shell and run this script again - python is installed but only new shells can see it'
        } else {
            Add-Todo 'install python 3.9 or newer, then run this script again'
        }
        return $null
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

    # Import names, not distribution names: pyserial imports as serial and PyYAML
    # as yaml, and asking the interpreter is far quicker than asking pip.
    $absent = Invoke-Python -Python $Python -Code @'
import importlib.util as util

WANTED = {'serial': 'pyserial', 'yaml': 'PyYAML', 'mcp': 'mcp',
          'anyio': 'anyio', 'pytest': 'pytest'}
print(','.join(dist for module, dist in WANTED.items()
               if util.find_spec(module) is None))
'@

    if ([string]::IsNullOrWhiteSpace($absent)) {
        Write-Item 'requirements' 'ok' 'all present'
        return
    }
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

# ---- 3. the ST toolchain ---------------------------------------------------

# Bundle name -> the executable that proves it arrived whole. An empty string
# means the bundle has no bin/ worth checking; its presence is the whole test.
#
# The names are cube's own, as `cube bundle list-online` prints them, which is
# also how they appear as directories under %LOCALAPPDATA%\stm32cube\bundles.
# That double duty is deliberate: one name serves both the install and the
# check, so a bundle cannot be installed under one spelling and looked for
# under another.
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
# reaches for it: CubeMX regenerates Core/ from the .ioc and is otherwise idle.
# -SkipCubeMX is how a bench that never opens the .ioc says so.
if (-not $SkipCubeMX) { $BundleNeeds['stm32cubemx-application'] = '' }

# The editor side, and only what this repository actually configures. Each one
# earns its place, because an extension installed "just in case" is a thing to
# keep updated on every bench for no reason:
#
#   the STM32 pack   carries cube.exe and cube-cmake, and pulls in cmake-tools,
#                    the serial monitor, the memory inspector, clangd and the
#                    debug adapters as a pack. Installing it installs fifteen.
#   cpptools         .vscode/c_cpp_properties.json is written for it; without it
#                    nothing in Core/ or Drivers/ resolves.
#   python           host/ is the other half of this project.
#
# Ollama.ollama is not here: it belongs to the model half and is installed with
# it, so -SkipOllama leaves it out. Anything the pack already brings is left to
# the pack rather than listed twice.
$Extensions = [ordered]@{
    'stmicroelectronics.stm32-vscode-extension' = 'the toolchain, the debuggers and cube.exe'
    'ms-vscode.cpptools'                        = 'IntelliSense over the HAL'
    'ms-python.python'                          = 'host/'
}

function Test-Bundles {
    Write-Head 'ST toolchain (bundles under %LOCALAPPDATA%\stm32cube)'
    $absent = @()
    foreach ($name in $BundleNeeds.Keys) {
        $bundle = Get-NewestBundle $name

        # CubeMX is the one entry here that has a second, equally valid home:
        # st.com's installer puts it under Program Files. Asking for a 308 MB
        # bundle of something the machine already has is the kind of thing that
        # makes a setup script get skipped.
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
        Write-Item 'vs code' 'done' $code
    } else {
        Write-Item 'vs code' 'ok' $code
    }

    # --list-extensions is one call and it is not a fast one, so it is asked
    # once here rather than per extension.
    $installed = (& $code --list-extensions)
    foreach ($id in $Extensions.Keys) {
        $why = $Extensions[$id]
        if ($installed -contains $id) {
            Write-Item $id 'ok' $why
            continue
        }
        Write-Item $id 'missing' $why
        if (Confirm-Step "code --install-extension $id ?  ($why)") {
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

    Install-Bundles -Absent $Absent
}

function Install-Bundles {
    <#  The ST toolchain, without a browser.

        `cube bundle install --yes NAME` downloads from developer.st.com and
        unpacks under %LOCALAPPDATA%, no account and no click-through, and the
        --yes answers the licence question the same way opening the bundles
        manager in VS Code would. This used to be the one step the script could
        set up and not finish; it finishes it now.

        One bundle at a time on purpose. A single invocation with eight names
        fails whole, and the failure that matters here is a 300 MB download
        dropping at 90% on bench wifi - the other seven should still be in
        place afterwards.  #>
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
        # as well as for a completed install. The directory is.
        if ($null -ne (Get-NewestBundle $name)) {
            Write-Item $name 'done' (Get-NewestBundle $name).Name
        } else {
            Write-Item $name 'failed' "cube exit $LASTEXITCODE"
            Add-Todo "cube bundle install --yes $name   (failed once already)"
        }
    }
}

function Install-StLinkDriver {
    <#  The USB driver, which is the one thing here that needs administrator
        rights - so it is asked for on its own rather than folded into the
        bundle loop above, and -SkipDriver turns it off outright.

        Without it Windows enumerates the ST-Link as an unknown device: the VCP
        appears, so `board all` finds a COM port, while SWD does not, so
        flashing fails with a probe error that reads like a cable fault. Worth
        the elevation prompt.  #>

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
    <#  STM32CubeMX.exe, from either kind of install.

        The bundle is the tidy one and what this script installs. But somebody
        who took the installer from st.com has it under Program Files instead,
        and telling that machine CubeMX is missing would be this script being
        wrong in a way the user can see.  #>

    $bundle = Get-NewestBundle 'stm32cubemx-application'
    if ($null -ne $bundle) {
        $exe = Get-ChildItem $bundle.FullName -Recurse -Filter 'STM32CubeMX.exe' `
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
    <#  The other way to get CubeMX: st.com's own installer.

        The bundle above is 308 MB and needs nothing but cube.exe, so it is
        what this script reaches for first. This is the path for a machine
        where that failed, or where the bundle is not wanted - and for the
        person who would rather see the licence they are agreeing to.

        -CubeMXInstaller runs an installer already on disk. Without it, the
        download page is opened and the run carries on: a missing CubeMX stops
        nothing here. It is not needed to build, flash, measure or drive the
        model - only to regenerate Core/ from the .ioc.  #>

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
        # ST's installer is a GUI with a licence page in it. Waited on rather
        # than fired and forgotten, so the check below means something.
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

function Get-FirmwareVersion {
    <#  Which FW_H7 the .ioc was generated against.

        Read from the project rather than pinned here, because CubeMX writes it
        and CubeMX is the thing that will complain about it. The line looks like

            ProjectManager.FirmwarePackage=STM32Cube FW_H7 V1.13.0  #>

    $ioc = Join-Path $Root 'coaxial_63100.ioc'
    if (-not (Test-Path $ioc)) { return $null }
    $line = Select-String -Path $ioc -Pattern '^ProjectManager\.FirmwarePackage=' `
                          -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($null -eq $line) { return $null }
    if ($line.Line -match 'V([0-9]+\.[0-9]+\.[0-9]+)') { return $Matches[1] }
    return $null
}

function Install-FirmwarePackage {
    <#  The STM32Cube FW_H7 package, which is the one thing here that ST still
        puts behind a login.

        First, what it is and is not for. The build does not need it: Drivers/
        is in this repository, so gcc has its HAL and CMSIS whether or not this
        step ever runs. What needs it is CubeMX - opening the .ioc against a
        repository that has no FW_H7 V1.13.0 in it means CubeMX offers to
        download one, and regenerating against a different version is a
        different Core/ than the one in git.

        Three ways in, in the order they are worth trying:

          1. -FirmwarePackage <zip or folder>. If the package is already on a
             share, a stick or another bench, this needs no browser and no
             account. It is the only fully automatic path.
          2. CubeMX itself. Its package manager downloads FW packages, and it
             is already installed by the bundle step above. It may ask for an
             st.com account; if it does, so be it - that is ST's decision, not
             this script's.
          3. The web page, opened in the default browser. A human agrees to the
             licence, downloads the zip, and hands it back through 1.

        What this function will not do is scrape a download out of st.com with
        a fabricated user agent. The URLs that used to serve these unauthenticated
        (sw-center.st.com) answer 404 now, and the GitHub mirror is submodules -
        `Drivers/STM32H7xx_HAL_Driver` is a gitlink there, so a zipball of the
        tag contains none of the sources that matter.  #>

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
                # so it expands into the repository rather than into the target.
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

    # 2. CubeMX, if it is here
    $mx = Get-NewestBundle 'stm32cubemx-application'
    if ($null -ne $mx) {
        Write-Item 'STM32CubeMX' 'ok' 'its package manager can fetch this one'
        Add-Todo ('open CubeMX (the `cubemx` command) and let it install ' + $name +
                  ' - Help > Manage embedded software packages')
    }

    # 3. the page
    if (Confirm-Step 'open the STM32CubeH7 download page in a browser ?') {
        Start-Process $CubeH7Url
        Write-Item 'download page' 'done' $CubeH7Url
    }
    Add-Todo ($CubeH7Url + '  -> download ' + $name +
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
    # No winget package publishes STM32CubeProgrammer. But -WingetToolchain
    # means "do not make me use VS Code", not "pretend cube.exe is not there":
    # if the extension is on this machine anyway, the programmer is a bundle
    # like any other and there is no reason to send anyone to st.com for it.
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
    <#  Both installers put ollama.exe under LOCALAPPDATA, and neither reaches
        the PATH of a shell that was already open. Look in the two known places
        before concluding it is absent, and splice whatever we find into this
        process's PATH so the model pull below works without a new shell.  #>
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
    <#  Two ways in, and the order is deliberate.

        winget is the tidy one: a known package, an uninstall entry, no code off
        the internet through iex. But Ollama.Ollama is not on every machine's
        source index, and the winget install wants elevation.

        Ollama's own installer script is the fallback. It is a per-user install
        under %LOCALAPPDATA%\Programs\Ollama and needs no administrator - which
        is also why it is worth having when winget declines. It is a piped
        remote script, so it is behind its own confirmation rather than folded
        into the first one.  #>

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
    <#  The API answering matters more than the binary existing: everything in
        host/coaxial_ollama talks to the HTTP endpoint, not to the CLI.

        Retried, because a daemon started a second ago has not bound 11434 yet
        and a single failed probe would send a working machine away with a
        manual step it does not need.  #>
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
    <#  The editor half of the same daemon. Ollama.ollama is Ollama's own
        extension: it registers the locally pulled models with VS Code's chat
        model picker, so the model answering in the editor is the one on 11434
        that dbg.py and the runner already talk to - no second copy of the
        weights, no second configuration to keep in step.

        Nothing in this repository needs it. dbg.py, the runner and the MCP
        server all speak HTTP, so a machine with no VS Code gets a note here
        and not a failure.  #>

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
    if ((& $code --list-extensions) -contains $id) {
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
    <#  Which tag this machine should run, if nobody said.

        The answer comes from capability.py rather than from a constant here,
        because it is the same question the runner and dbg.py ask and there
        should be one answer to it. A machine with no python yet, or a probe
        that fails, falls back to the tag this bench was built on rather than
        stopping the install.  #>
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
        # A split model needs the layer count on every call, and only dbg.py and
        # the runner can pass it. Said here so it is not a surprise later.
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

    $tags = Get-OllamaTags
    if (($null -eq $tags) -and (-not $Check)) {
        # A fresh install has not started its daemon, and the install that just
        # ran did not put one in this session. Start it rather than making the
        # pull below a manual step; -Check never gets here, by design.
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
        # Offline by design: no board, no ollama. If this fails, the fault is in
        # the install, not on the bench.
        $output = (& $Python 'tests/test_ollama.py')
        $tail = ($output | Select-Object -Last 1)
        if ($LASTEXITCODE -eq 0) {
            Write-Item 'host test suite' 'ok' $tail
        } else {
            Write-Item 'host test suite' 'failed' $tail
            Add-Todo 'python tests/test_ollama.py failed - run it and read the output'
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
    # script deliberately does not touch PATH. Look where it lives instead, the
    # same way env.ps1 does, or a fresh machine is told it is missing when it is
    # sitting right there.
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
        # Find-CubeMX, so a standalone install from st.com counts as much as the
        # bundle. The bundle's internal layout is ST's business and has moved
        # before, which is why it is searched rather than assumed.
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
# A setup of this length is either something you are watching or something you
# started and walked away from, and the difference should be a decision rather
# than a habit of hitting y.
#
# -Check never asks: there is nothing to consent to when nothing installs. -Yes
# has already answered.
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

Write-Host ''
Write-Host '  every shell:' -ForegroundColor White
Write-Host '    . .\env.ps1                 tools on PATH, plus bench/dbg/board/cbuild/cflash/cubemx'
Write-Host ''
Write-Host '  then:' -ForegroundColor White
Write-Host '    cbuild                      build, zero warnings expected'
Write-Host '    cflash                      flash over SWD and start'
Write-Host '    board all                   measure, no model involved'
Write-Host '    dbg "why is the NTC 25.00?" ask the local model, cheaply'
Write-Host '    bench                       a prompt with the model and the board in it'
Write-Host '    cubemx                      open the .ioc in STM32CubeMX'
Write-Host ''
if ($Check) {
    Write-Host '  run again without -Check to install what is missing.' -ForegroundColor DarkGray
    Write-Host ''
}
