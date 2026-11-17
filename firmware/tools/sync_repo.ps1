# Copies this project (the folder above tools/) into <repo>\firmware of the group repository clone, ready to
# commit. Usage: .\tools\sync_repo.ps1 [-Repo D:\EMA3148_DACN_CDT2026]
# Mirrors (files deleted here are deleted there), except what is not published: build output, IDE settings, flight
# logs, books, the old firmware, the unused IAR project / reference code / ESP-NOW bridge, and firmware\.agents
# (teammate's files, kept as they are in the repo).
param([string]$Repo = "D:\EMA3148_DACN_CDT2026")
$src = Split-Path $PSScriptRoot -Parent
$dst = Join-Path $Repo "firmware"
if (-not (Test-Path (Join-Path $Repo ".git"))) { Write-Error "Khong thay repo git o $Repo"; exit 1 }
$xd = @(".git", "build", "Debug", "Release", ".metadata", ".settings", ".vscode", "__pycache__", ".agents",
        (Join-Path $src "gcs\logs"), (Join-Path $src "docs"), (Join-Path $src "drone_stm32f405rgt6"),
        (Join-Path $src "EWARM"), (Join-Path $src "s450drone"), (Join-Path $src "esp"))
$xf = @("*.launch", "*.pyc", "*.ino")
robocopy $src $dst /MIR /XD @xd /XF @xf /NFL /NDL /NJH /NP | Out-Null
if ($LASTEXITCODE -ge 8) { Write-Error "robocopy loi $LASTEXITCODE"; exit 1 }
git -C $Repo status --short
exit 0
