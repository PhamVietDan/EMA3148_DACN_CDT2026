# Builds and runs the host tests for App/lib with a PC gcc (default: MSYS2 ucrt64).
$gccDir = if ($env:HOST_GCC_DIR) { $env:HOST_GCC_DIR } else { "C:\msys64\ucrt64\bin" }
$env:PATH = "$gccDir;$env:PATH"
$root = Split-Path $PSScriptRoot -Parent
$out = Join-Path $root "build\host"
New-Item -ItemType Directory -Force $out | Out-Null
$srcs = @("tests\test_lib.c", "App\lib\filter.c", "App\lib\kalman1d.c", "App\app\telem_proto.c", "App\drivers\sbus.c", "App\drivers\mspv2.c", "App\drivers\mtf01p.c", "App\drivers\dps310.c", "App\drivers\ist8310.c", "tests\test_arming.c", "App\app\arming.c", "tests\test_ctrl.c", "App\lib\mixer.c", "App\lib\pid.c", "tests\test_params.c", "App\app\params.c", "App\lib\crc32.c", "App\lib\numfmt.c", "App\app\blackbox.c", "App\app\capture.c", "tests\test_alt.c", "App\app\alt_est.c", "App\lib\alt_kf.c", "tests\test_alt_hold.c", "App\app\alt_hold.c", "tests\test_levelcal.c", "App\app\levelcal.c", "tests\test_yaw_hold.c", "App\app\yaw_hold.c", "tests\test_flow_est.c", "App\app\flow_est.c", "tests\test_pos_hold.c", "App\app\pos_hold.c", "tests\test_ubx.c", "App\drivers\ubx.c") | ForEach-Object { Join-Path $root $_ }
gcc -std=c11 -O1 -Wall -Wextra -Wdouble-promotion -I (Join-Path $root "App") @srcs -lm -o (Join-Path $out "test_lib.exe")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $out "test_lib.exe")
exit $LASTEXITCODE
