# Builds and runs the host tests for App/math_algorithm with a PC gcc (default: MSYS2 ucrt64).
$gccDir = if ($env:HOST_GCC_DIR) { $env:HOST_GCC_DIR } else { "C:\msys64\ucrt64\bin" }
$env:PATH = "$gccDir;$env:PATH"
$root = Split-Path $PSScriptRoot -Parent
$out = Join-Path $root "build\host"
New-Item -ItemType Directory -Force $out | Out-Null
$srcs = @("tests\test_lib.c", "App\math_algorithm\filter.c", "App\math_algorithm\kalman1d.c", "App\control_logic\telem_proto.c", "App\sensor_and_radio_driver\sbus.c", "tests\test_arming.c", "App\control_logic\arming.c", "tests\test_ctrl.c", "App\math_algorithm\mixer.c", "App\math_algorithm\pid.c", "tests\test_params.c", "App\control_logic\params.c", "App\math_algorithm\crc32.c", "App\math_algorithm\numfmt.c", "App\control_logic\blackbox.c") | ForEach-Object { Join-Path $root $_ }
gcc -std=c11 -O1 -Wall -Wextra -Wdouble-promotion -I (Join-Path $root "App") @srcs -lm -o (Join-Path $out "test_lib.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $out "test_lib.exe")
exit $LASTEXITCODE
