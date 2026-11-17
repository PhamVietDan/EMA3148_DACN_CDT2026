# Builds with the GCC/make bundled in STM32CubeIDE. Usage: .\tools\build.ps1 [clean]
param([string]$Target = "all")
$ide = if ($env:CUBEIDE_DIR) { $env:CUBEIDE_DIR } else { "D:\STM32CubeIDE_2.1.1\STM32CubeIDE" }
$plugins = Join-Path $ide "plugins"
$gcc  = Get-ChildItem $plugins -Directory -Filter "*gnu-tools-for-stm32*" | Select-Object -First 1
$make = Get-ChildItem $plugins -Directory -Filter "*externaltools.make*" | Select-Object -First 1
if (-not $gcc -or -not $make) { Write-Error "Khong tim thay toolchain trong $plugins (dat CUBEIDE_DIR)"; exit 1 }
$env:PATH = "$($gcc.FullName)\tools\bin;$($make.FullName)\tools\bin;$env:PATH"
Push-Location (Split-Path $PSScriptRoot -Parent)
try { make -j8 -f tools/Makefile $Target; exit $LASTEXITCODE } finally { Pop-Location }
