# linux.ps1 - the offline gate on Linux here (run_tests.py --linux, host/tools/dev/linux.py): GitHub's runner
# as a container on this machine's engine - WSL 2 under it, the engine and its group, the modules that drive it
# (host/requirements-linux.txt), the runner's images.

function Find-Docker {
    <#
  The engine's command line: on PATH, else where its installer puts it - a shell opened before the install
        lacks its PATH, so its folder joins this one's -; $null where neither.
#>
    $found = Get-Tool 'docker'
    if ($null -ne $found) { return $found }
    $exe = Join-Path $env:ProgramFiles 'Docker\Docker\resources\bin\docker.exe'
    if (-not (Test-Path $exe)) { return $null }
    $env:PATH = (Split-Path $exe) + ';' + $env:PATH
    return $exe
}

function Test-DockerUsers {
    <#
  Whether this logon holds the docker-users group the engine's service answers. Windows gives a logon its
        groups as it signs in: a member added since (the installer adds its user) waits for the next sign-in;
        the engine runs without its service till then.
#>
    # The logon's own groups, by .NET: `whoami` is Git's coreutils one first on some PATHs (no /groups).
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $me = $identity.Name
    $held = @($identity.Groups | ForEach-Object {
        try { $_.Translate([Security.Principal.NTAccount]).Value } catch { '' } })
    if ($held -match '\\docker-users$') {
        Write-Item 'docker-users' 'ok' $me
        return
    }
    $member = @(Get-LocalGroupMember -Group 'docker-users' -ErrorAction SilentlyContinue |
                Where-Object { $_.Name -eq $me }).Count -gt 0
    if ($member) {
        Write-Item 'docker-users' 'missing' 'a member since this sign-in: the next one carries it'
        Add-Todo -Optional 'sign out and in once: the docker-users group reaches a logon as it signs in'
        return
    }
    Write-Item 'docker-users' 'missing' "$me is no member: the engine's service refuses it"
    $add = "Add-LocalGroupMember -Group 'docker-users' -Member '$me'"
    if (Confirm-Step "add $me to docker-users ?  (an administrator's yes; the next sign-in carries it)") {
        $proc = Start-Process powershell -Verb RunAs -Wait -PassThru -ArgumentList '-NoProfile', '-Command', $add
        if ($proc.ExitCode -eq 0) {
            Write-Item 'docker-users' 'done' 'added - sign out and in once'
            Add-Todo -Optional 'sign out and in once: the docker-users group reaches a logon as it signs in'
        } else {
            Write-Item 'docker-users' 'failed' "exit $($proc.ExitCode)"
            Add-Todo "$add   (as administrator), then sign out and in"
        }
    } else {
        Add-Todo "$add   (as administrator), then sign out and in"
    }
}

function Install-Linux {
    <#
  WSL 2, the container engine, host/requirements-linux.txt and the runner's images, each after what it needs.
#>
    param([string]$Python)
    Write-Head 'linux runner'

    & wsl.exe --version *> $null
    if ($LASTEXITCODE -eq 0) {
        Write-Item 'wsl' 'ok' 'WSL 2'
    } else {
        Write-Item 'wsl' 'missing' 'WSL 2 - the Linux the engine runs on'
        $done = $false
        if (Confirm-Step 'wsl --install --no-distribution ?  (an administrator''s yes; a restart where Windows asks)') {
            $proc = Start-Process wsl.exe -Verb RunAs -Wait -PassThru -ArgumentList '--install', '--no-distribution'
            $done = ($proc.ExitCode -eq 0)
            if ($done) { Write-Item 'wsl' 'done' 'installed' } else { Write-Item 'wsl' 'failed' "exit $($proc.ExitCode)" }
        }
        if (-not $done) { Add-Todo 'wsl --install --no-distribution   (as administrator)' }
    }

    $docker = Find-Docker
    if ($null -eq $docker) {
        Write-Item 'container engine' 'missing' 'Docker.DockerDesktop - the gate on Linux here'
        if (Install-WingetPackage -Id 'Docker.DockerDesktop' -Why 'the gate on Linux: run_tests.py --linux') {
            $docker = Find-Docker
        }
        if ($null -eq $docker) { return }
    }
    Test-DockerUsers
    # Its whole output, then its code: Select-Object -First stops a program early and its code reads -1.
    $version = @(& $docker info --format '{{.ServerVersion}}' 2>$null)
    if ($LASTEXITCODE -ne 0) {
        $desktop = Join-Path $env:ProgramFiles 'Docker\Docker\Docker Desktop.exe'
        Write-Item 'container engine' 'missing' 'installed, not answering'
        if ((Test-Path $desktop) -and (Confirm-Step 'start the container engine ?')) { Start-Process $desktop }
        Add-Todo 'the container engine started: setup.ps1 again once `docker info` answers'
        return
    }
    Write-Item 'container engine' 'ok' "docker $($version[0])"
    if (-not $Python) { return }

    $requirements = Join-Path $Host_ 'requirements-linux.txt'
    $absent = Invoke-Host -Python $Python -Arguments @('-X', 'utf8', 'tools/dev/depends.py', '--absent', $requirements)
    if (($LASTEXITCODE -eq 0) -and [string]::IsNullOrWhiteSpace($absent)) {
        Write-Item 'requirements-linux' 'ok' 'all present'
    } elseif (Confirm-Step 'pip install -r host/requirements-linux.txt ?') {
        # through the lock, so what it brings that the host has already stays at its pinned version
        & $Python -m pip install --disable-pip-version-check -r $requirements -c (Join-Path $Host_ 'constraints.txt')
        if ($LASTEXITCODE -ne 0) {
            Write-Item 'requirements-linux' 'failed' "pip exit $LASTEXITCODE"
            Add-Todo 'pip install -r host/requirements-linux.txt failed - read the output above'
            return
        }
        Write-Item 'requirements-linux' 'done' 'installed'
    } else {
        Write-Item 'requirements-linux' 'missing' $absent
        Add-Todo 'python -m pip install -r host/requirements-linux.txt'
        return
    }

    # The runner's images: minutes each the first time, seconds after - the engine keeps their steps.
    $kept = Invoke-Host -Python $Python -Arguments @('-X', 'utf8', 'tools/dev/linux.py', '--kept')
    if ($LASTEXITCODE -eq 0) {
        Write-Item 'runner images' 'ok' $kept
    } elseif (Confirm-Step 'build the runner''s images ?  (a few GB, minutes)') {
        Push-Location $Host_
        try { & $Python -X utf8 tools/dev/linux.py --build } finally { Pop-Location }
        if ($LASTEXITCODE -eq 0) { Write-Item 'runner images' 'done' 'built' }
        else {
            Write-Item 'runner images' 'failed' "exit $LASTEXITCODE"
            Add-Todo 'python tools/dev/linux.py --build failed - run it in host/ and read the output'
        }
    } else {
        Write-Item 'runner images' 'missing' $kept
        Add-Todo 'python tools/dev/linux.py --build   (in host/; run_tests.py --linux builds them too)'
    }
}
