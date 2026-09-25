<#
.SYNOPSIS
    From a clone to a built image answering on the emulator, in one run.
.DESCRIPTION
    update    git pull --ff-only; the pulled setup instead, if it changed   setup\update.ps1
    check     every dependency, nothing fetched: the plan                    setup\<area>.ps1
    install   the plan, after one yes (-Yes: none)                           setup\<area>.ps1
    build     both images, Debug and Release, zero warnings                  setup\build.ps1
    emulate   the Debug image on Renode, answering the library               setup\build.ps1
    Areas: machine, python, toolchain, emulator, ollama. Exit 0: nothing outstanding.
    Every shell afterwards: . .\env.ps1
.PARAMETER Check
    Report only: no pull, no install, no build.
.PARAMETER Yes
    Install the plan without asking.
.PARAMETER NoPull
    Leave the checkout as it is.
.PARAMETER NoBuild
    Stop after the install.
.PARAMETER Tests
    After the emulator, the offline gate (host: tools/dev/run_tests.py --offline).
.PARAMETER Upgrade
    Also upgrade VS Code through winget: the STM32 pack is version-matched to it.
.PARAMETER Model
    The ollama tag to pull.
.PARAMETER Prefer
    What the automatic model choice optimises for.
.PARAMETER SkipOllama
    Leave the model side alone - a machine that only builds and flashes.
.PARAMETER SkipRenode
    Leave the emulator out.
.PARAMETER SkipCubeMX
    Do not install STM32CubeMX.
.PARAMETER SkipDriver
    Do not touch the ST-Link USB driver.
.PARAMETER SkipFirmware
    Do not look for the STM32Cube FW_H7 package.
.PARAMETER FirmwarePackage
    A STM32Cube_FW_H7_Vx.y.z.zip, or an unpacked copy, into the CubeMX repository.
.PARAMETER CubeMXInstaller
    An STM32CubeMX installer already on disk, instead of the bundle.
.PARAMETER SkipCubeIDE
    Do not look for or offer STM32CubeIDE.
.PARAMETER CubeIDEInstaller
    An STM32CubeIDE installer already on disk.
.PARAMETER Repository
    Where CubeMX keeps its firmware packages.
.PARAMETER WingetToolchain
    cmake, ninja and arm-none-eabi-gcc from winget instead of the VS Code bundles.
.PARAMETER PythonVersion
    The CPython release to install from python.org when python is absent, e.g. '3.13.1'.
.PARAMETER AllowScripts
    Set the CurrentUser execution policy to RemoteSigned, so env.ps1 loads.
#>
[CmdletBinding()]
param(
    [switch]$Check,
    [switch]$Yes,
    [switch]$NoPull,
    [switch]$NoBuild,
    [switch]$Tests,
    [switch]$Upgrade,
    [string]$Model,
    [ValidateSet('speed', 'capability')]
    [string]$Prefer = 'speed',
    [switch]$SkipOllama,
    [switch]$SkipRenode,
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
$script:Todo = @()
$script:Optional = @()
$script:Python = $null
$script:Argv = $PSBoundParameters

foreach ($part in 'report', 'machine', 'python', 'toolchain', 'emulator', 'ollama', 'checks',
                  'update', 'build') {
    . (Join-Path $Root "setup\$part.ps1")
}

function Invoke-Dependencies {
    <#
  Every dependency, each after what it needs: checked with $Check set, installed without.
#>
    $script:Python = Test-Machine
    Install-PythonDeps -Python $script:Python
    if ($WingetToolchain) {
        Install-WingetToolchain
    } else {
        Install-VsCodeExtensions -Absent (Test-Bundles)
        Install-StLinkDriver
    }
    Install-CubeMXFromInstaller
    Install-FirmwarePackage
    Install-CubeIDE
    Install-Renode
    Install-Simulations
    if (-not $SkipOllama) { Install-Ollama -Python $script:Python }
}

Write-Banner

if (-not ($Check -or $NoPull)) { Update-Checkout }

# Everything checked before anything is fetched; the misses are the plan.
$asked = $Check
$Check = $true
Invoke-Dependencies
$Check = $asked
if (($script:Todo.Count -gt 0) -and (-not $Check) -and (Approve-Plan $script:Todo)) {
    $script:Todo = @()
    $script:Optional = @()
    $Yes = $true
    $script:OnlyChanges = $true
    Invoke-Dependencies
    $script:OnlyChanges = $false
}

if (-not ($Check -or $NoBuild)) {
    Build-Target
    Test-Emulated
    if ($Tests) { Test-Gate }
}
Test-Setup -Python $script:Python
Write-Summary

exit $(if ($script:Todo.Count -gt 0) { 1 } else { 0 })
