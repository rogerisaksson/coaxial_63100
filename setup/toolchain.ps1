# toolchain.ps1 - the ST toolchain: VS Code and its extensions, the cube bundles (gcc,
# cmake, ninja, programmer, gdb, CubeMX), the ST-Link driver, CubeMX and CubeIDE installers,
# the FW_H7 package; or the winget toolchain.

$BundleRoot = Join-Path $env:LOCALAPPDATA 'stm32cube\bundles'
$CubeH7Url = 'https://www.st.com/en/embedded-software/stm32cubeh7.html'
$CubeMXUrl = 'https://www.st.com/en/development-tools/stm32cubemx.html'
$CubeIDEUrl = 'https://www.st.com/en/development-tools/stm32cubeide.html'

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
        # -Upgrade only: an install pass must not restart the editor it runs in.
        if ($Upgrade) {
            Update-WingetPackage -Id 'Microsoft.VisualStudioCode' -What 'vs code' `
                                 -Why 'the STM32 pack is version-matched to the editor' | Out-Null
        }
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
    if (Confirm-Step -Attended 'open the STM32CubeMX download page in a browser ?') {
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
    if (Confirm-Step -Attended 'open the STM32CubeIDE download page in a browser ?') {
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
    if (Confirm-Step -Attended 'open the STM32CubeH7 download page in a browser ?') {
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
