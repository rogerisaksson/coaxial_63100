# build.ps1 - the target built, both images at both presets with zero warnings; the Debug
# image run on the emulator until it answers the library; the papers in notebook_examples/
# behind their sources written and run on it; with -Tests, the offline gate.

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

function Update-Notebooks {
    <#
  The papers in notebook_examples/ out of host/tools/notebooks/: those behind their sources -
        cells other than the builder's, a code cell never run - written and run, each laid where
        it ran whole, one whose prose alone changed laid with the outputs it has
        (host/tools/notebooks/current.py, which git's hooks ask after a pull too).
#>
    Write-Head 'notebooks'
    if (-not $script:Packages) {
        Write-Item 'notebooks' 'missing' 'after the python packages above'
        return
    }
    $maker = @('-X', 'utf8', '-m', 'tools.notebooks.current')
    $behind = Invoke-Host -Python $script:Python -Arguments ($maker + '--behind')
    if ($LASTEXITCODE -ne 0) {
        Write-Item 'notebooks' 'failed' $behind
        Add-Todo 'python -m tools.notebooks.current --behind failed - run it in host/'
    } elseif ([string]::IsNullOrWhiteSpace($behind)) {
        Write-Item 'notebooks' 'ok' 'each paper its sources'', each run'
    } else {
        Write-Item 'notebooks' 'missing' ('behind their sources: ' + $behind)
        if (Confirm-Step 'lay them ?  (run where their code changed: a kernel and an emulator each)') {
            $tail = Invoke-Host -Python $script:Python -Arguments $maker
            if ($LASTEXITCODE -eq 0) {
                Write-Item 'notebooks' 'done' 'laid'
            } else {
                Write-Item 'notebooks' 'failed' $tail
                Add-Todo 'python -m tools.notebooks.current failed - run it in host/ and read which paper'
            }
        } else {
            Add-Todo 'python -m tools.notebooks.current   (in host/)'
        }
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
