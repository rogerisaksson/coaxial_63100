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

      Python 3.9+ and four packages     host/requirements.txt, via pip
      ARM gcc, cmake, ninja             STM32 VS Code extension "bundles"
      STM32_Programmer_CLI             same
      cube-cmake                        the extension itself
      ollama and one small model        winget or ollama.com/install.ps1,
                                        then `ollama pull`

    The ST half is the awkward half, and it is worth knowing why. ST does not
    publish its tools to winget, and the direct downloads from st.com are behind
    a login and a click-through licence - so a script cannot fetch them without
    either shipping credentials or lying about having agreed to something. What
    it CAN do is install the VS Code extension, which has its own bundle manager
    and downloads the toolchain itself on first use. That is the path this script
    takes, and the one this machine was built on.

    If you would rather not have VS Code in the loop, -WingetToolchain installs
    cmake, ninja and Arm's own gcc from winget instead. The build works; the
    programmer still has to come from ST by hand, and the note at the end says so
    rather than leaving you to find out at flashing time.

    Nothing here needs administrator rights except the winget installs, and
    nothing is written to the system PATH. Per-shell PATH is env.ps1's job.

.PARAMETER Check
    Report only. No installs, no downloads, no writes.

.PARAMETER Yes
    Do not ask before each install.

.PARAMETER Model
    The ollama tag to pull. Small is the point: it runs beside the bench.

.PARAMETER SkipOllama
    Leave the model side alone - for a machine that only builds and flashes.

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
    [string]$Model = 'qwen3:4b',
    [switch]$SkipOllama,
    [switch]$WingetToolchain,
    [switch]$AllowScripts
)

$ErrorActionPreference = 'Continue'
$Root = $PSScriptRoot
$Host_ = Join-Path $Root 'host'
$BundleRoot = Join-Path $env:LOCALAPPDATA 'stm32cube\bundles'
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
    $answer = Read-Host "  $Text  [y/N]"
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

    $python = Get-Tool 'python'
    if ($null -eq $python) { $python = Get-Tool 'py' }
    if ($null -eq $python) {
        Write-Item 'python' 'missing' 'winget install Python.Python.3.13'
        Add-Todo 'install python 3.9 or newer, then run this script again'
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
        Write-Item 'git' 'missing' 'winget install Git.Git'
    } else {
        Write-Item 'git' 'ok' $git
    }

    $winget = Get-Tool 'winget'
    if ($null -eq $winget) {
        Write-Item 'winget' 'missing' 'nothing can be installed automatically'
        Add-Todo 'install App Installer from the Microsoft Store to get winget'
    } else {
        Write-Item 'winget' 'ok' ''
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

$BundleNeeds = [ordered]@{
    'gnu-tools-for-stm32' = 'arm-none-eabi-gcc.exe'
    'cmake'               = 'cmake.exe'
    'ninja'               = 'ninja.exe'
    'programmer'          = 'STM32_Programmer_CLI.exe'
    'gnu-gdb-for-stm32'   = 'arm-none-eabi-gdb.exe'
    'stlink-server'       = ''
}

$Extensions = @(
    'stmicroelectronics.stm32-vscode-extension'
)

function Test-Bundles {
    Write-Head 'ST toolchain (bundles under %LOCALAPPDATA%\stm32cube)'
    $absent = @()
    foreach ($name in $BundleNeeds.Keys) {
        $bundle = Get-NewestBundle $name
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

    $code = Get-Tool 'code'
    if ($null -eq $code) {
        Write-Item 'vs code' 'missing' 'winget install Microsoft.VisualStudioCode'
        Add-Todo 'install VS Code, or re-run with -WingetToolchain for a toolchain without it'
        return
    }
    Write-Item 'vs code' 'ok' $code

    $installed = (& $code --list-extensions)
    foreach ($id in $Extensions) {
        if ($installed -contains $id) {
            Write-Item $id 'ok' ''
            continue
        }
        Write-Item $id 'missing' ''
        if (Confirm-Step "code --install-extension $id ?") {
            & $code --install-extension $id --force
            if ($LASTEXITCODE -eq 0) {
                Write-Item $id 'done' 'installed'
            } else {
                Write-Item $id 'failed' "code exit $LASTEXITCODE"
            }
        } else {
            Add-Todo "code --install-extension $id"
        }
    }

    if ($Absent.Count -gt 0) {
        # The extension downloads its own bundles, and only when VS Code is
        # running. There is no supported CLI for it, so this is a step a script
        # can set up and cannot finish.
        Write-Item 'bundles' 'manual' ($Absent -join ', ')
        Add-Todo ('open this folder in VS Code once and let the STM32 bundles ' +
                  'manager download: ' + ($Absent -join ', ') +
                  '. It prompts on the first build; cube-cmake --build --preset Debug triggers it.')
    }
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
    # No winget package publishes STM32CubeProgrammer, and the ST download is
    # behind an account. Say so here rather than at flashing time.
    Write-Item 'STM32_Programmer_CLI' 'manual' 'not on winget'
    Add-Todo ('STM32CubeProgrammer has to come from st.com by hand (free ' +
              'account, click-through licence). Install it, then add its bin ' +
              'directory to PATH - or use the VS Code extension route, where ' +
              'the bundle manager fetches it for you.')
}

# ---- 4. ollama and one small model ----------------------------------------

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

function Install-Ollama {
    Write-Head 'ollama'
    $ollama = Find-Ollama
    if ($null -eq $ollama) {
        Write-Item 'ollama' 'missing' 'Ollama.Ollama, or ollama.com/install.ps1'
        $ollama = Install-OllamaBinary
        if ($null -eq $ollama) {
            Add-Todo 'winget install --id Ollama.Ollama --exact   (or: irm https://ollama.com/install.ps1 | iex)'
            Add-Todo "ollama pull $Model"
            return
        }
        Write-Item 'ollama' 'done' $ollama
    } else {
        Write-Item 'ollama' 'ok' $ollama
    }

    # The API answering matters more than the binary existing: everything in
    # host/coaxial_ollama talks to the HTTP endpoint, not to the CLI.
    $reachable = $false
    try {
        $tags = Invoke-RestMethod -Uri 'http://localhost:11434/api/tags' -TimeoutSec 5
        $reachable = $true
    } catch {
        $tags = $null
    }

    if (-not $reachable) {
        Write-Item 'ollama serve' 'missing' 'nothing answering on 11434'
        Add-Todo ('start ollama - it normally runs as a service after install - then: ollama pull ' + $Model)
        return
    }

    $names = @()
    if ($null -ne $tags.models) { $names = $tags.models | ForEach-Object { $_.name } }
    Write-Item 'ollama serve' 'ok' ("$($names.Count) model(s): " + ($names -join ', '))

    $stem = ($Model -split ':')[0]
    $have = $names | Where-Object { ($_ -split ':')[0] -eq $stem }
    if ($null -ne $have) {
        Write-Item 'model' 'ok' ($have -join ', ')
        return
    }

    Write-Item 'model' 'missing' $Model
    if (Confirm-Step "ollama pull $Model ?  (a few GB)") {
        ollama pull $Model
        if ($LASTEXITCODE -eq 0) {
            Write-Item 'model' 'done' $Model
        } else {
            Write-Item 'model' 'failed' "ollama exit $LASTEXITCODE"
        }
    } else {
        Add-Todo "ollama pull $Model"
    }
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

    $cube = Get-Tool 'cube-cmake'
    if ($null -eq $cube) {
        Write-Item 'cube-cmake' 'missing' 'comes with the VS Code extension'
    } else {
        Write-Item 'cube-cmake' 'ok' $cube
    }
}

# ---- main ------------------------------------------------------------------

Write-Host ''
Write-Host 'coaxial_63100 setup' -ForegroundColor White
Write-Host ("  " + $Root) -ForegroundColor DarkGray
if ($Check) {
    Write-Host '  -Check: reporting only, nothing will be installed' -ForegroundColor DarkGray
}

$python = Test-Machine
Install-PythonDeps -Python $python

if ($WingetToolchain) {
    Install-WingetToolchain
} else {
    $absent = Test-Bundles
    Install-VsCodeExtensions -Absent $absent
}

if (-not $SkipOllama) {
    Install-Ollama
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
Write-Host '    . .\env.ps1                 tools on PATH, plus dbg/board/cbuild/cflash'
Write-Host ''
Write-Host '  then:' -ForegroundColor White
Write-Host '    cbuild                      build, zero warnings expected'
Write-Host '    cflash                      flash over SWD and start'
Write-Host '    board all                   measure, no model involved'
Write-Host '    dbg "why is the NTC 25.00?" ask the local model, cheaply'
Write-Host ''
if ($Check) {
    Write-Host '  run again without -Check to install what is missing.' -ForegroundColor DarkGray
    Write-Host ''
}
