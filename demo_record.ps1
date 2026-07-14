# ============================================================
# Booster T1 Demo Recording Script
# Usage:
#   .\demo_record.ps1           # Standard demo (~2.5 min)
#   .\demo_record.ps1 -Quick    # Quick rehearsal (~40 sec)
#   .\demo_record.ps1 -NoPause  # Auto-run without Enter
# ============================================================

param(
    [switch]$Quick,
    [switch]$NoPause
)

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

if ($Quick) {
    $DurDefault = 5
    $DurPass    = 8
    $DurShoot   = 8
    $DurThreat  = 8
    $PauseSec   = 2
    $ModeLabel  = "Quick"
} else {
    $DurDefault = 15
    $DurPass    = 20
    $DurShoot   = 20
    $DurThreat  = 20
    $PauseSec   = 4
    $ModeLabel  = "Standard"
}

function Write-Banner([string]$Title, [string]$Subtitle) {
    Write-Host ""
    Write-Host ("=" * 62) -ForegroundColor Cyan
    Write-Host "  $Title" -ForegroundColor Cyan
    if ($Subtitle) {
        Write-Host "  $Subtitle" -ForegroundColor DarkGray
    }
    Write-Host ("=" * 62) -ForegroundColor Cyan
    Write-Host ""
}

function Wait-Continue([string]$Hint) {
    Write-Host ""
    Write-Host ">>> $Hint" -ForegroundColor Yellow
    if (-not $NoPause) {
        Write-Host "    (Press Enter to continue, or use -NoPause)" -ForegroundColor DarkGray
        Read-Host | Out-Null
    } else {
        Start-Sleep -Seconds $PauseSec
    }
}

function Ensure-Venv {
    if (-not (Test-Path ".\.venv\Scripts\Activate.ps1")) {
        Write-Host "  Creating venv..." -ForegroundColor DarkYellow
        python -m venv .venv
        .\.venv\Scripts\Activate.ps1
        python -m pip install --upgrade pip -q
        python -m pip install pytest matplotlib -q
    } else {
        .\.venv\Scripts\Activate.ps1
    }
    $env:PYTHONUTF8 = "1"
}

function Run-Demo([string]$Label, [string[]]$PyArgs) {
    $cmdLine = "python main.py " + ($PyArgs -join " ")
    Write-Host "  CMD: $cmdLine" -ForegroundColor DarkGray
    Write-Host ""
    python main.py @PyArgs
    if ($LASTEXITCODE -ne 0) {
        throw "Demo step failed: $Label"
    }
}

# ============================================================
Write-Banner "Booster T1 Multi-Robot Soccer Decision System" "Project 4 Demo Script"
Write-Host "  Mode:   $ModeLabel"
Write-Host "  Path:   $PSScriptRoot"
Write-Host ""

Ensure-Venv

# --- Scene 1: Project structure ---
Write-Banner "Scene 1 / 5" "Project structure"
Write-Host "  common/      WorldState, RobotAction, config"
Write-Host "  strategy/    pass, dribble, shoot, position, block"
Write-Host "  decision/    FSM: CHASE -> PASS -> SHOOT -> BLOCK"
Write-Host "  simulation/  2D mock physics"
Write-Host "  bridge/      Real mode adapters"
Write-Host "  tests/       57 unit and integration tests"
Write-Host "  docs/        SRS, Design, API, TestReport"
Write-Host ""
Write-Host "  Key files:" -ForegroundColor White
foreach ($f in @("main.py", "run.sh", "README.md", "docs/API.md", "docs/DemoScript.md")) {
    if (Test-Path $f) {
        Write-Host "    [OK] $f" -ForegroundColor Green
    }
}

Wait-Continue "Intro: layered architecture and entry point main.py"

# --- Scene 2: Default scenario with ASCII viz ---
Write-Banner "Scene 2 / 5" "Default scenario with ASCII visualization (${DurDefault}s)"
Write-Host "  Shows role assignment and state transitions"
Write-Host ""
Run-Demo "default" @("--duration", "$DurDefault")
Wait-Continue "Explain: ball_carrier / supporter / defender roles"

# --- Scene 3: Pass scenario ---
Write-Banner "Scene 3 / 5" "Pass scenario (${DurPass}s + CSV export)"
Write-Host "  Shows CHASE -> PASS cooperative play"
Write-Host ""
Run-Demo "pass" @(
    "--scenario", "pass",
    "--duration", "$DurPass",
    "--headless",
    "--export-csv"
)

New-Item -ItemType Directory -Force -Path "outputs\csv" | Out-Null
if (Test-Path "outputs\decision_log.csv") {
    Copy-Item "outputs\decision_log.csv" "outputs\csv\decision_log_pass.csv" -Force
    $passLines = (Select-String -Path "outputs\decision_log.csv" -Pattern ",PASS,").Count
    Write-Host ""
    Write-Host "  PASS log entries: $passLines" -ForegroundColor Green
}

Wait-Continue "Explain: two robots cooperate on pass decision"

# --- Scene 4: Shoot + Threat ---
Write-Banner "Scene 4 / 5" "Shoot scenario (${DurShoot}s)"
Run-Demo "shoot" @(
    "--scenario", "shoot",
    "--duration", "$DurShoot",
    "--headless",
    "--export-csv"
)
if (Test-Path "outputs\decision_log.csv") {
    Copy-Item "outputs\decision_log.csv" "outputs\csv\decision_log_shoot.csv" -Force
}

Write-Host ""
Write-Banner "Scene 4 / 5 (cont.)" "Threat / block scenario (${DurThreat}s)"
Write-Host "  Shows BLOCK when opponent threatens our goal"
Write-Host ""
Run-Demo "threat" @(
    "--scenario", "threat",
    "--duration", "$DurThreat",
    "--headless",
    "--export-csv"
)
if (Test-Path "outputs\decision_log.csv") {
    Copy-Item "outputs\decision_log.csv" "outputs\csv\decision_log_threat.csv" -Force
    $blockLines = (Select-String -Path "outputs\decision_log.csv" -Pattern ",BLOCK,").Count
    Write-Host ""
    Write-Host "  BLOCK log entries: $blockLines" -ForegroundColor Green
}

Wait-Continue "Explain: strategy switching for shoot and defense"

# --- Scene 5: Tests and logs ---
Write-Banner "Scene 5 / 5" "Automated tests and decision logs"
Write-Host "  CMD: python -m pytest -q"
Write-Host ""
$pytestOut = python -m pytest -q 2>&1
Write-Host $pytestOut
if ($LASTEXITCODE -ne 0) {
    throw "pytest failed"
}

New-Item -ItemType Directory -Force -Path "outputs\logs" | Out-Null
$pytestOut | Out-File -Encoding utf8 "outputs\logs\pytest_result.txt"

Write-Host ""
Write-Host "  CSV preview (pass scenario, first 5 rows):" -ForegroundColor White
if (Test-Path "outputs\csv\decision_log_pass.csv") {
    Get-Content "outputs\csv\decision_log_pass.csv" -TotalCount 6 | ForEach-Object {
        Write-Host "    $_" -ForegroundColor DarkGray
    }
}

Write-Host ""
Write-Host "  Output files:" -ForegroundColor White
foreach ($f in @(
    "outputs\csv\decision_log_pass.csv",
    "outputs\csv\decision_log_shoot.csv",
    "outputs\csv\decision_log_threat.csv",
    "outputs\logs\pytest_result.txt"
)) {
    if (Test-Path $f) {
        $size = (Get-Item $f).Length
        Write-Host "    [OK] $f  ($size bytes)" -ForegroundColor Green
    }
}

# --- Done ---
Write-Banner "Demo Complete" "Save screen recording to outputs/videos/"
$ts = Get-Date -Format "yyyyMMdd_HHmmss"
Write-Host "  Suggested video path: outputs/videos/demo_$ts.mp4"
Write-Host ""
Write-Host "  Quick rerun:  .\demo_record.ps1 -Quick -NoPause"
Write-Host "  Full record:  .\demo_record.ps1"
Write-Host ""
Write-Host "  Narration script: docs/DemoScript.md"
Write-Host ""
