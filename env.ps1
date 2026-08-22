<#
.SYNOPSIS
    Puts this project's tools on PATH for the current shell. Dot-source it:

        . .\env.ps1

.DESCRIPTION
    Nothing on this machine installs the ARM toolchain, cmake, ninja or the
    programmer onto the system PATH. The STM32 VS Code extension downloads them
    as "bundles" under %LOCALAPPDATA%\stm32cube\bundles and drives them itself,
    which is why `cube-cmake --build` works while plain `cmake` and
    `STM32_Programmer_CLI` are nowhere to be found.

    That is fine for building from the editor and awkward for everything else:
    flashing from a prompt, reading a map file with objdump, or checking which
    gcc actually produced the image. This script finds the newest of each bundle
    and prepends its bin directory, for this shell only. Nothing is written to
    the registry and no system PATH is touched, so a stale entry cannot outlive
    the window it was made in.

    Versions are resolved at run time rather than baked in: the bundles manager
    updates them without asking, and a path pinned in a file goes stale the
    first time it does. Both naming schemes are handled - 2.23.0 as well as
    2.22.0+st.1.

    It also defines the six commands this project is actually driven with:

        bench    the model, the board and a prompt            (bench.ps1)
        dbg      ask the local model about the board          (host/dbg.py)
        board    the plain CLI, no model                      (python -m coaxial)
        cbuild   build the firmware, zero warnings expected
        cflash   flash over SWD and start the core
        cubemx   open the .ioc in STM32CubeMX

.PARAMETER Quiet
    Print nothing on success.
#>
[CmdletBinding()]
param([switch]$Quiet)

$script:CoaxialRoot = $PSScriptRoot
$BundleRoot = Join-Path $env:LOCALAPPDATA 'stm32cube\bundles'

function Get-NewestBundleBin {
    <#  Newest version of one bundle, as its bin directory.
        The sort key strips the vendor suffix: '2.22.0+st.1' and '2.23.0' have
        to compare as versions, and [version] chokes on the '+'.  #>
    param([string]$Name)

    $dir = Join-Path $BundleRoot $Name
    if (-not (Test-Path $dir)) { return $null }

    $newest = Get-ChildItem $dir -Directory -ErrorAction SilentlyContinue |
        Sort-Object -Property @{Expression = {
            $stem = ($_.Name -split '\+')[0]
            try { [version]$stem } catch { [version]'0.0.0' }
        }} -Descending | Select-Object -First 1

    if ($null -eq $newest) { return $null }
    $bin = Join-Path $newest.FullName 'bin'
    if (Test-Path $bin) { return $bin }
    return $newest.FullName
}

$Wanted = [ordered]@{
    'gnu-tools-for-stm32' = 'arm-none-eabi-gcc.exe'
    'cmake'               = 'cmake.exe'
    'ninja'               = 'ninja.exe'
    'programmer'          = 'STM32_Programmer_CLI.exe'
    'gnu-gdb-for-stm32'   = 'arm-none-eabi-gdb.exe'
}

$added = @()
$missing = @()
foreach ($name in $Wanted.Keys) {
    $bin = Get-NewestBundleBin $name
    if ($null -eq $bin) { $missing += $name; continue }
    if ($env:Path -notlike "*$bin*") { $env:Path = "$bin;$env:Path" }
    $added += ($name + ' ' + (Split-Path $bin -Parent | Split-Path -Leaf))
}

# cube.exe is the bundle manager - `cube bundle install`, `cube stlink-detect`
# and the rest. It ships inside the extension too, and setup.ps1 drives it to
# fetch the toolchain in the first place.
$core = Get-ChildItem (Join-Path $env:USERPROFILE '.vscode\extensions') -Directory `
        -Filter 'stmicroelectronics.stm32cube-ide-core-*' -ErrorAction SilentlyContinue |
        Sort-Object Name -Descending | Select-Object -First 1
if ($null -ne $core) {
    $cubeBin = Join-Path $core.FullName 'resources\binaries\win32\x86_64'
    if ((Test-Path $cubeBin) -and ($env:Path -notlike "*$cubeBin*")) {
        $env:Path = "$cubeBin;$env:Path"
        $added += 'cube'
    }
}

# cube-cmake itself ships inside the VS Code extension, not as a bundle.
$ext = Get-ChildItem (Join-Path $env:USERPROFILE '.vscode\extensions') -Directory `
        -Filter 'stmicroelectronics.stm32cube-ide-build-cmake-*' -ErrorAction SilentlyContinue |
        Sort-Object Name -Descending | Select-Object -First 1
if ($null -ne $ext) {
    $cube = Join-Path $ext.FullName 'resources\cube-cmake\win32\x86_64'
    if ((Test-Path $cube) -and ($env:Path -notlike "*$cube*")) {
        $env:Path = "$cube;$env:Path"
    }
} else {
    $missing += 'cube-cmake (VS Code extension)'
}

# ollama installs per-user under LOCALAPPDATA and reaches the PATH of shells
# opened after the install, which is never the one you are standing in. Same
# treatment as the bundles, and for the same reason: this shell only.
if ($null -eq (Get-Command 'ollama' -ErrorAction SilentlyContinue)) {
    $ollamaBin = @(
        (Join-Path $env:LOCALAPPDATA 'Programs\Ollama'),
        (Join-Path $env:LOCALAPPDATA 'Microsoft\WinGet\Links')
    ) | Where-Object { Test-Path (Join-Path $_ 'ollama.exe') } | Select-Object -First 1

    if ($null -eq $ollamaBin) {
        $missing += 'ollama'
    } else {
        $env:Path = "$ollamaBin;$env:Path"
        $added += 'ollama'
    }
} else {
    $added += 'ollama'
}

function cubemx {
    <# Open the .ioc in STM32CubeMX.

       The bundle's own layout is ST's business, so the executable is searched
       for rather than assumed - and the search is here, not at dot-source time,
       because it walks 800 MB of bundle and nobody wants that in every shell. #>
    param([string]$Ioc = 'coaxial_63100.ioc')

    $dir = Join-Path $env:LOCALAPPDATA 'stm32cube\bundles\stm32cubemx-application'
    if (-not (Test-Path $dir)) {
        Write-Host 'STM32CubeMX is not installed: run .\setup.ps1' -ForegroundColor Yellow
        return
    }
    $exe = Get-ChildItem $dir -Recurse -Filter 'STM32CubeMX.exe' -ErrorAction SilentlyContinue |
           Select-Object -First 1
    if ($null -eq $exe) {
        Write-Host 'the stm32cubemx-application bundle has no STM32CubeMX.exe in it' `
            -ForegroundColor Yellow
        return
    }
    Start-Process -FilePath $exe.FullName -ArgumentList (Join-Path $script:CoaxialRoot $Ioc)
}

function bench {
    <# The prompt loop, with the daemon started and the model already loaded.
       bench.ps1 does the preflight; this is just the short way to say it. #>
    & (Join-Path $script:CoaxialRoot 'bench.ps1') @args
}

function dbg {
    <# Ask the local model about the board. See host/coaxial_ollama/debug.py. #>
    Push-Location (Join-Path $script:CoaxialRoot 'host')
    try { python dbg.py @args } finally { Pop-Location }
}

function board {
    <# The plain CLI: measure, no model in the loop. #>
    Push-Location (Join-Path $script:CoaxialRoot 'host')
    try { python -m coaxial @args } finally { Pop-Location }
}

function cbuild {
    <# Build. Zero warnings is the standard, not an aspiration. #>
    Push-Location $script:CoaxialRoot
    try { cube-cmake --build --preset Debug @args } finally { Pop-Location }
}

function cflash {
    <# Flash over SWD and start the core.

       SWD, not JTAG: any connect that asserts NRST on this probe fails with
       'Unable to get core ID'. And --start rather than -hardRst, or the core is
       left halted with no clue as to why. #>
    param([string]$Elf = 'build/Debug/coaxial_63100.elf')
    Push-Location $script:CoaxialRoot
    try {
        STM32_Programmer_CLI -c port=SWD mode=UR -d $Elf -v --start
    } finally { Pop-Location }
}

if (-not $Quiet) {
    Write-Host ("PATH  + " + ($added -join ', ')) -ForegroundColor DarkGray
    if ($missing.Count -gt 0) {
        Write-Host ("absent: " + ($missing -join ', ') + "  -> run .\setup.ps1") `
            -ForegroundColor Yellow
    }
    Write-Host 'commands: bench, dbg, board, cbuild, cflash, cubemx' -ForegroundColor DarkGray
}
