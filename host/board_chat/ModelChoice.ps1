<#
    Which model this run should use, and getting its weights into the OS
    file cache before ollama asks for them. Needs Say (Say.ps1) already in
    scope; both functions here also read board_chat.ps1's own $Root,
    $Prefer and $Reserve straight from the caller's scope.
#>

function Invoke-Warm {
    <#  host/tools/warm_model.py --auto owns the decision, not this script:
        a real timed read of the model's own blob against a measured free-RAM
        figure, not a guess from the disk's reported type. This bench's own
        NVMe/SSDs clear the "worth it" threshold several times over, so here
        it always says skip - see the script's docstring for the numbers. A
        machine with a slower disk and RAM to spare gets a different answer
        from the same measurement, with nothing to configure for it.

        Never fatal: a machine where this fails is a reason to load cold,
        not a reason to stop.  #>
    param([string]$Tag)

    Push-Location $Root
    try {
        $lines = & python (Join-Path 'tools' 'warm_model.py') $Tag --auto 2>&1
        foreach ($line in $lines) { Say 'ok' 'warm' $line }
    } catch {
        Say 'warn' 'warm' ('skipped: ' + $_.Exception.Message)
    } finally {
        Pop-Location
    }
}

function Get-Choice {
    <#  Which model this machine should run, and how much of it fits the card.

        capability.py owns the answer - the same answer setup.ps1 and
        `dbg -m auto` get - so a bench does not end up with three opinions
        about which tag is right. A machine where the probe fails is not a
        reason to stop: fall back to the tag this bench was built on and say
        so.  #>

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
