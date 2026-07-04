param(
    [switch]$Open,
    [switch]$NoPreview,
    [switch]$New
)

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$parser = Join-Path $scriptDir "parse_flight_log.py"

$argsList = @($parser)
if ($New) {
    $argsList += "--new"
} else {
    $argsList += "--latest"
}
if (-not $NoPreview) {
    $argsList += "--preview"
}
if ($Open) {
    $argsList += "--open"
}

Push-Location $repoRoot
try {
    python @argsList
} finally {
    Pop-Location
}
