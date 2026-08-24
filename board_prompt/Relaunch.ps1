<#
    The one helper -NewWindow needs; the relaunch itself stays in
    board_prompt.ps1 since it is the top of the script's own control flow,
    not something anything else calls into.
#>

function Format-Argument {
    <#  One argument, safe to hand to Start-Process.

        -ArgumentList joins an array with spaces and quotes nothing, so a value
        with a space in it arrives as several arguments. Measured the hard way:
        `-NewWindow -Ask "read the NTC"` reached the new window as -Ask read,
        the, NTC - and `NTC` then bound to -Prefer, which rejected it against
        its ValidateSet. The error named a parameter nobody had typed.  #>
    param([string]$Text)

    if ($Text -match '^[-\w:.\/]+$') { return $Text }
    return '"' + ($Text -replace '"', '\"') + '"'
}
