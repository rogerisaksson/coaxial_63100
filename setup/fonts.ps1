# fonts.ps1 - the tty's faces (the user, 2026-10-10): Eurostile, VS Code's terminal's
# (.vscode/settings.json), and Eurostile Extended. URW's and DTC's, never in this public repo:
# each installed for this user from a copy in setup\fonts (ignored) or Downloads, a zip's too.

#: Each face as Windows registers it, and its file.
$Faces = @(
    @{ Name = 'Eurostile'; File = 'eurostile.TTF' },
    @{ Name = 'Eurostile Extended Black'; File = 'EurostileExtendedBlack.ttf' }
)
$FontKey = 'Software\Microsoft\Windows NT\CurrentVersion\Fonts'

function Find-Face {
    <#
  The installed face's file, this user's or the machine's, or $null.
#>
    param([string]$Name)
    foreach ($hive in 'HKCU:', 'HKLM:') {
        $key = Get-ItemProperty -Path (Join-Path $hive $FontKey) -ErrorAction SilentlyContinue
        if ($null -eq $key) { continue }
        $value = $key.PSObject.Properties | Where-Object { $_.Name -eq "$Name (TrueType)" } |
                 Select-Object -First 1
        if ($null -eq $value) { continue }
        $path = [string]$value.Value
        if (-not [IO.Path]::IsPathRooted($path)) { $path = Join-Path $env:WINDIR "Fonts\$path" }
        if (Test-Path $path) { return $path }
    }
    return $null
}

function Find-FaceCopy {
    <#
  A copy of $File on disk: setup\fonts, then Downloads two folders down, then a zip in
        Downloads - @{ Path; Zip }, $null where none.
#>
    param([string]$File)
    $downloads = Join-Path $env:USERPROFILE 'Downloads'
    foreach ($place in (Join-Path $Root 'setup\fonts'), $downloads) {
        if (-not (Test-Path $place)) { continue }
        $found = Get-ChildItem $place -Recurse -Depth 2 -File -Filter $File `
                               -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($null -ne $found) { return @{ Path = $found.FullName; Zip = $null } }
    }
    if (-not (Test-Path $downloads)) { return $null }
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    foreach ($zip in Get-ChildItem $downloads -Filter '*.zip' -File -ErrorAction SilentlyContinue) {
        try {
            $archive = [IO.Compression.ZipFile]::OpenRead($zip.FullName)
            try {
                if ($archive.Entries | Where-Object { $_.Name -eq $File }) {
                    return @{ Path = $File; Zip = $zip.FullName }
                }
            } finally {
                $archive.Dispose()
            }
        } catch {
            continue
        }
    }
    return $null
}

function Install-Face {
    <#
  $Copy (Find-FaceCopy) into this user's fonts as $Name: no administrator, seen by a program
        started after it.
#>
    param([string]$Name, [string]$File, $Copy)
    $dir = Join-Path $env:LOCALAPPDATA 'Microsoft\Windows\Fonts'
    New-Item -ItemType Directory -Force $dir | Out-Null
    $dest = Join-Path $dir $File
    if ($Copy.Zip) {
        $archive = [IO.Compression.ZipFile]::OpenRead($Copy.Zip)
        try {
            $entry = $archive.Entries | Where-Object { $_.Name -eq $File } | Select-Object -First 1
            [IO.Compression.ZipFileExtensions]::ExtractToFile($entry, $dest, $true)
        } finally {
            $archive.Dispose()
        }
    } else {
        Copy-Item $Copy.Path $dest -Force
    }
    New-ItemProperty -Path (Join-Path 'HKCU:' $FontKey) -Name "$Name (TrueType)" -Value $dest `
                     -PropertyType String -Force | Out-Null
    return $dest
}

function Install-Fonts {
    Write-Head 'fonts'
    foreach ($face in $Faces) {
        $have = Find-Face $face.Name
        if ($null -ne $have) {
            Write-Item $face.Name 'ok' $have
            continue
        }
        $copy = Find-FaceCopy $face.File
        $from = if ($null -eq $copy) { $null } elseif ($copy.Zip) { $copy.Zip } else { $copy.Path }
        if ($null -eq $from) {
            Write-Item $face.Name 'missing' "no $($face.File) in setup\fonts or Downloads"
            Add-Todo "$($face.File) into setup\fonts, then setup.ps1 again: the tty's face"
            continue
        }
        if (-not (Confirm-Step "install $($face.Name) for this user from $from ?")) {
            Write-Item $face.Name 'missing' $from
            Add-Todo "$($face.Name) for this user: setup.ps1 again (from $from)"
            continue
        }
        try {
            $dest = Install-Face -Name $face.Name -File $face.File -Copy $copy
            Write-Item $face.Name 'done' "$dest - VS Code restarted draws in it"
        } catch {
            Write-Item $face.Name 'failed' $_.Exception.Message
            Add-Todo "$($face.Name) for this user: open $from and Install"
        }
    }
}
