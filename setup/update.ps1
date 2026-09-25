# update.ps1 - the checkout brought to its upstream: a fast-forward pull on a clean, tracked
# branch, and the pulled setup run instead of this one when the pull changed it.

function Update-Checkout {
    Write-Head 'update'
    $git = Get-Tool 'git'
    if ($null -eq $git) {
        Write-Item 'git pull' 'missing' 'no git - the machine check installs it'
        return
    }
    $branch = (& $git -C $Root symbolic-ref --short -q HEAD)
    if ([string]::IsNullOrWhiteSpace($branch)) {
        Write-Item 'git pull' 'manual' 'detached HEAD - left as it is'
        return
    }
    $upstream = (& $git -C $Root rev-parse --abbrev-ref --symbolic-full-name '@{u}' 2>$null)
    if ([string]::IsNullOrWhiteSpace($upstream)) {
        Write-Item 'git pull' 'manual' "$branch tracks nothing - left as it is"
        return
    }
    # Tracked files only: a pull that merges into local edits is the operator's call.
    $dirty = @(& $git -C $Root status --porcelain --untracked-files=no)
    if ($dirty.Count -gt 0) {
        Write-Item 'git pull' 'manual' "$($dirty.Count) file(s) changed here - commit or stash, then again"
        return
    }

    $before = (& $git -C $Root rev-parse HEAD)
    & $git -C $Root pull --ff-only --quiet 2>&1 | Out-Null
    if ($LASTEXITCODE -ne 0) {
        Write-Item 'git pull' 'failed' "$branch has diverged from $upstream - pull by hand"
        Add-Todo "git pull on $branch - it does not fast-forward"
        return
    }
    $after = (& $git -C $Root rev-parse HEAD)
    if ($before -eq $after) {
        Write-Item 'git pull' 'ok' "$branch at $upstream"
        return
    }
    $count = (& $git -C $Root rev-list --count "$before..$after")
    Write-Item 'git pull' 'done' "$count commit(s): $($before.Substring(0, 7))..$($after.Substring(0, 7))"

    # The script running is the one before the pull; the rest has to be the one after it.
    $changed = @(& $git -C $Root diff --name-only $before $after -- setup.ps1 setup)
    if ($changed.Count -gt 0) {
        Write-Host '  setup changed with the pull - running the pulled one' -ForegroundColor DarkGray
        $argv = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', (Join-Path $Root 'setup.ps1'),
                  '-NoPull')
        foreach ($key in $script:Argv.Keys) {
            $value = $script:Argv[$key]
            if ($value -is [System.Management.Automation.SwitchParameter]) {
                if ($value.IsPresent) { $argv += "-$key" }
            } else {
                $argv += @("-$key", "$value")
            }
        }
        & (Get-Process -Id $PID).Path @argv
        exit $LASTEXITCODE
    }
}
