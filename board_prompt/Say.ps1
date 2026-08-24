<#
    One line of preflight output, coloured by state. Every other file in this
    folder calls this - dot-source it before any of the others, though in
    practice board_prompt.ps1 dot-sources the whole folder up front and none
    of these functions run until well after that, so the order among them
    does not actually matter.
#>

function Say {
    param([string]$State, [string]$Text, [string]$Detail = '')
    $colour = 'Gray'
    if ($State -eq 'ok')   { $colour = 'Green' }
    if ($State -eq 'wait') { $colour = 'Cyan' }
    if ($State -eq 'warn') { $colour = 'Yellow' }
    if ($State -eq 'fail') { $colour = 'Red' }
    Write-Host ('  {0,-6}' -f $State) -ForegroundColor $colour -NoNewline
    Write-Host ('{0,-22} ' -f $Text) -NoNewline
    Write-Host $Detail -ForegroundColor DarkGray
}
