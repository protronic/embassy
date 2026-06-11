# Search Windows drives for Graphics4D static libraries / sources (Workshop5 / Pico SDK).
# Run in PowerShell after compiling a Workshop5 RP2350 project once.
#
#   cd examples\gen4-rp2350-70ct-clb
#   powershell -ExecutionPolicy Bypass -File scripts\find-graphics4d-lib.ps1

$patterns = @(
    "libgraphics4d*.a",
    "libGraphics4D*.a",
    "Graphics4D.cpp",
    "Graphics4D.h"
)

$roots = @(
    "$env:LOCALAPPDATA\4D Systems",
    "$env:LOCALAPPDATA\4DSYSTEMS",
    "$env:ProgramFiles\4D Systems",
    "$env:ProgramFiles(x86)\4D Systems",
    "$env:USERPROFILE\Documents",
    "$env:USERPROFILE\pico",
    "$env:USERPROFILE\source",
    "C:\Pico",
    "C:\pico"
)

Write-Host "=== Searching for Graphics4D libraries (this can take a minute) ===" -ForegroundColor Cyan

foreach ($root in $roots) {
    if (-not (Test-Path $root)) { continue }
    Write-Host "`n-- $root --"
    foreach ($pat in $patterns) {
        Get-ChildItem -Path $root -Recurse -Filter $pat -ErrorAction SilentlyContinue |
            Select-Object -First 20 FullName |
            ForEach-Object { Write-Host "  $($_.FullName)" }
    }
}

Write-Host "`n=== Tip ===" -ForegroundColor Yellow
Write-Host "1. Open Workshop5, compile any gen4-RP2350-70CT project."
Write-Host "2. Search inside that project's 'build' folder for *.a"
Write-Host "3. Copy the archive to:"
Write-Host "   vendor\Graphics4D-pico\lib\libgraphics4d_rp2350.a"
Write-Host "4. On Linux: ./scripts/check-graphics4d.sh"
