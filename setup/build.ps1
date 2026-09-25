# build.ps1 - the target built, both images at both presets with zero warnings; the Debug
# image run on the emulator until it answers the library; with -Tests, the offline gate.

function Build-Target {
    Write-Head 'build'
    if ($null -eq $script:Python) {
        Write-Item 'firmware' 'missing' 'no python to run the build'
        return
    }
    $tool = Join-Path $Host_ 'tools\target\build_and_flash.py'
    foreach ($preset in 'Debug', 'Release') {
        $output = @(& $script:Python -X utf8 $tool --build-only --preset $preset 2>&1)
        $line = "$($output | Where-Object { "$_" -match '^BUILD' } | Select-Object -Last 1)"
        if (($LASTEXITCODE -eq 0) -and ($line -match ' 0 warnings')) {
            Write-Item "image $preset" 'ok' ($line -replace '^BUILD\s+ok\s+', '')
        } else {
            if ($line -eq '') { $line = "$($output | Select-Object -Last 1)" }
            Write-Item "image $preset" 'failed' $line
            Add-Todo "python host/tools/target/build_and_flash.py --build-only --preset $preset - read its output"
        }
    }
}

function Test-Emulated {
    Write-Head 'emulated board'
    if ($null -eq (Find-Renode)) {
        Write-Item 'Renode' 'missing' 'the emulator stage needs it'
        return
    }
    if (-not (Test-Path (Join-Path $Root 'build\Debug\coaxial_63100.elf'))) {
        Write-Item 'Debug image' 'missing' 'the build above did not leave one'
        return
    }
    Push-Location $Host_
    try {
        $output = @(& $script:Python -X utf8 'tools\emu\emulator.py' --world bench --check 2>&1)
    } finally {
        Pop-Location
    }
    $line = "$($output | Select-Object -Last 1)"
    if ($LASTEXITCODE -eq 0) {
        Write-Item 'image on Renode' 'ok' $line
    } else {
        Write-Item 'image on Renode' 'failed' $line
        Add-Todo 'python host/tools/emu/emulator.py --check - the image did not answer on Renode'
    }
}

function Test-Gate {
    Write-Head 'tests'
    Push-Location $Host_
    try {
        $output = @(& $script:Python -X utf8 'tools\dev\run_tests.py' --offline 2>&1)
    } finally {
        Pop-Location
    }
    $line = "$($output | Where-Object { "$_" -match '^Total:' } | Select-Object -Last 1)"
    if ($LASTEXITCODE -eq 0) {
        Write-Item 'offline gate' 'ok' $line
    } else {
        Write-Item 'offline gate' 'failed' $line
        Add-Todo 'python tools/dev/run_tests.py --offline (from host/) - read its output'
    }
}
