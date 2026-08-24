<#
    Daemon state: what tags exist, what is resident on the card, and
    unloading whatever should not be there before this run adds its own.
    Needs Say (Say.ps1) and $Api (set in board_prompt.ps1 itself) already in
    scope - Clear-Resident also reads $KeepOthers, board_prompt.ps1's own
    switch, straight from the caller's scope.
#>

function Get-Tags {
    param([int]$Tries = 1)
    for ($i = 0; $i -lt $Tries; $i++) {
        try {
            return (Invoke-RestMethod -Uri ($Api + '/api/tags') -TimeoutSec 5)
        } catch {
            if ($i -lt ($Tries - 1)) { Start-Sleep -Seconds 2 }
        }
    }
    return $null
}

function Get-Resident {
    <#  What ollama is holding on the card, as returned by /api/ps. #>
    try {
        $ps = Invoke-RestMethod -Uri ($Api + '/api/ps') -TimeoutSec 10
    } catch {
        return @()
    }
    if ($null -eq $ps.models) { return @() }
    return @($ps.models)
}

function Clear-Resident {
    <#  Unload anything still on the card, except the model about to be used.

        A prompt that exits cleanly hands its weights back, but not every exit
        is clean: a killed window, a -Hold from last time, an `ollama run` in
        another terminal. Those sit there until their keep_alive runs out, and
        the next load then asks a card that is already full - which on this
        bench was a 500 from the daemon reading 'cudaMalloc failed', with
        nothing obviously wrong at either end.

        Keeping a matching model is not an exception to the rule, it is the
        rule: what is being cleared is VRAM nobody is going to use, and weights
        this run is about to load are not that.  #>
    param([string]$Except)

    if ($KeepOthers) { return }

    foreach ($entry in (Get-Resident)) {
        if ($entry.name -eq $Except) {
            Say 'ok' 'resident' ('{0} already loaded, {1:n1} GB - kept' `
                -f $entry.name, ($entry.size_vram / 1GB))
            continue
        }
        try {
            $body = @{ model = $entry.name; prompt = ''; keep_alive = 0 } | ConvertTo-Json
            Invoke-RestMethod -Uri ($Api + '/api/generate') -Method Post -Body $body `
                              -ContentType 'application/json' -TimeoutSec 60 | Out-Null
            Say 'ok' 'unloaded' ('{0} was still resident, {1:n1} GB freed' `
                -f $entry.name, ($entry.size_vram / 1GB))
        } catch {
            Say 'warn' 'unloaded' ('could not unload ' + $entry.name + ': ' + $_.Exception.Message)
        }
    }
}
