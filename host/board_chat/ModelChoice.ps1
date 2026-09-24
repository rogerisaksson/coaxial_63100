<#
    Which model this run should use, and getting its weights into the OS
    file cache before ollama asks for them. $Root, $Prefer and $Reserve
    are board_chat.ps1's, read from the caller's scope.
#>

function Invoke-Warm {
    <#
  host/tools/dev/warm_model.py --auto owns the decision; its docstring has
        the numbers. Never fatal: a failed run loads cold.
#>
    param([string]$Tag)

    try {
        $lines = & python (Join-Path $Root 'tools\dev\warm_model.py') $Tag --auto 2>&1
        $state = if ($LASTEXITCODE -eq 0) { 'ok' } else { 'warn' }
        foreach ($line in $lines) { Say $state 'warm' $line }
    } catch {
        Say 'warn' 'warm' ('skipped: ' + $_.Exception.Message)
    }
}

function Get-Choice {
    <#
  Which model this machine should run, and how much of it fits the card.
        capability.py owns the answer - the same answer setup.ps1 and
        `dbg -m auto` get - so a bench does not end up with three opinions
        about which tag is right. A failed measurement falls back to the tag
        this bench was built on and says so.
#>

    Push-Location $Root
    try {
        $argv = @('-m', 'coaxial_ollama.capability', '--json', '--prefer', $Prefer)
        if ($Reserve -gt 0) { $argv += @('--reserve-gb', [string]$Reserve) }
        $json = (& python @argv) -join ''
    } catch {
        $json = ''
    } finally {
        Pop-Location
    }
    if ([string]::IsNullOrWhiteSpace($json)) {
        Say 'warn' 'model choice' 'could not measure this machine - falling back'
        return @{ model = 'gemma4:12b'; num_gpu = $null; why = 'fallback' }
    }
    try {
        $picked = $json | ConvertFrom-Json
    } catch {
        Say 'warn' 'model choice' 'capability.py said something unreadable'
        return @{ model = 'gemma4:12b'; num_gpu = $null; why = 'fallback' }
    }
    return @{ model = $picked.model
              num_gpu = $picked.options.num_gpu
              why = $picked.why
              machine = $picked.machine }
}
