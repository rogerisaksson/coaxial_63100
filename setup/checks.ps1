# checks.ps1 - does any of it work: the host suite that needs no board, the serial ports,
# cube-cmake, CubeMX.

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
