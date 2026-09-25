# report.ps1 - the report, the todo list, the one question, and the helpers every area
# uses: tools on PATH, a python snippet, a winget install or upgrade, the editor.

# The install pass sets $script:OnlyChanges: its 'ok' lines were printed by the check, and a
# heading prints only above something that changed.
$script:OnlyChanges = $false
$script:Head = $null

function Write-Head {
    param([string]$Text)
    $script:Head = $Text
    if (-not $script:OnlyChanges) { Show-Head }
}

function Show-Head {
    if ($null -eq $script:Head) { return }
    Write-Host ''
    Write-Host "-- $($script:Head) " -ForegroundColor Cyan -NoNewline
    Write-Host ('-' * [Math]::Max(0, 60 - $script:Head.Length)) -ForegroundColor DarkCyan
    $script:Head = $null
}

function Write-Item {
    param([string]$Name, [string]$State, [string]$Detail = '')
    if ($script:OnlyChanges -and ($State -eq 'ok')) { return }
    Show-Head
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
    <#
  Whether to do it. -Attended: a browser opened or a login asked for - never unattended; the
        todo line beside it says what to do.
#>
    param([string]$Text, [switch]$Attended)
    if ($Check) { return $false }
    if ($Yes)   { return (-not $Attended) }
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

function Write-Banner {
    Write-Host ''
    Write-Host 'coaxial_63100 setup' -ForegroundColor White
    Write-Host ("  " + $Root) -ForegroundColor DarkGray
    if ($Check) {
        Write-Host '  -Check: reporting only - no pull, no install, no build' -ForegroundColor DarkGray
    }
}

function Approve-Plan {
    <#
  The check's todo list as the plan, and one question for all of it - -Yes answers it.
#>
    param([string[]]$Plan)
    Write-Head 'plan'
    $n = 0
    foreach ($item in $Plan) {
        $n = $n + 1
        Write-Host ("  {0}. {1}" -f $n, $item) -ForegroundColor Yellow
    }
    if ($Yes) { return $true }
    Write-Host '  the ST-Link driver, if listed, still raises its own elevation prompt' -ForegroundColor DarkGray
    return (Read-YesNo '  install all of it?  [y/N]')
}

function Write-Summary {
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
    Write-Host '    .\coaxial_tty.ps1 -Emulated  the terminal on the image in Renode, no board'
    Write-Host '    python host/tools/emu/emulator.py [--nodes N]   the emulator alone, its URL printed'
    Write-Host ''
    if ($Check) {
        Write-Host '  run again without -Check to pull, install what is missing and build.' -ForegroundColor DarkGray
        Write-Host ''
    }
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
