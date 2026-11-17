# Firmware quadcopter STM32F405 + FreeRTOS

Phần này gồm **firmware bay** và **phần mềm trạm mặt đất (GCS)** của đồ án; phần cứng được trình bày riêng.

Firmware được viết lại từ đầu theo kế hoạch trong [`PROJECT_PLAN.md`](PROJECT_PLAN.md); quá trình làm, các lần bay thử và kết quả phân tích log được ghi trong [`PROGRESS.md`](PROGRESS.md).

## Tính năng

| Chế độ / chức năng | Mô tả |
|---|---|
| **Angle** | Cần tay cầm đặt góc nghiêng; vòng góc (P) → vòng tốc độ góc (PID) → mixer |
| **Alt Hold** | Giữ độ cao bằng Kalman 2 trạng thái (gia tốc + ToF), cất cánh tự động lên độ cao đặt, tự học ga hover |
| **Position Hold** | Giữ vị trí trong nhà bằng optical flow: vị trí (P) → vận tốc (PI) → góc nghiêng; thả cần thì phanh rồi neo tại chỗ dừng |
| Giữ hướng (yaw) | Khi cần yaw ở giữa, giữ hướng hiện tại |
| Failsafe | Mất sóng: nếu đang bay và còn độ cao hợp lệ thì tự hạ cánh, nếu không thì tắt động cơ |
| Arming an toàn | Chỉ arm khi IMU đã hiệu chuẩn, ga thấp, cảm biến tốt (lý do bị từ chối hiện trên GCS) |
| Phát hiện bậc ToF (tuỳ chọn) | Bay qua mặt bàn: độ cao vẫn tính so với sàn cất cánh (`alt_step_th`, mặc định tắt) |
| Baro dự phòng | Mất ToF ngắn (≤ 3 s) ở trên 0,5 m: baro giữ độ cao |
| Tham số trong flash | Đọc/ghi qua GCS (`SET`, `SAVE`), có CRC |
| Ghi log | Telemetry 50 Hz, capture tần số cao (~265 Hz) cho phân tích rung, log optical flow |
| GPS | Đọc UBX NAV-PVT (mới quan sát, chưa dùng để bay) |

## Kiến trúc firmware

### Các task và tần số

```
 BMI323 INT1 ──EXTI──► SPI DMA ──► ISR: đổi đơn vị, lọc gyro/accel (3,2 kHz)
                                        │ mỗi 3 mẫu: osSignalSet
                                        ▼
 task_ctrl  (osPriorityRealtime, ~1067 Hz)
   rate PID → mixer → ESC PWM          mỗi 2 chu kỳ: Kalman tư thế + vòng góc
        ▲  setpoint góc / ga                │ tư thế, trạng thái arm
        │                                   ▼
 task_slow  (osPriorityHigh, ~100 Hz, đánh thức theo gói ToF)
   tay cầm SBUS, MTF-01P (ToF + optical flow), la bàn, baro, GPS
   Kalman độ cao → Alt Hold;  ước lượng vận tốc flow → Position Hold
                                        │
                                        ▼
 task_telem (osPriorityBelowNormal, 50 Hz)
   telemetry + lệnh GCS qua USB CDC hoặc UART (HC-05)
```

- **Đường nóng ngắn:** IMU → `task_ctrl` → ESC là một chặng duy nhất, không qua queue. Việc chậm (I2C, phân tích gói, USB) nằm ở task ưu tiên thấp hơn nên không làm trễ vòng rate.
- **Dữ liệu dùng chung:** mỗi đối tượng chỉ có một task ghi, task khác chép cả struct trong vùng găng (`shared_state.h`), không dùng mutex.
- Cả 3 task nằm trong [`App/app/tasks.c`](App/app/tasks.c); thuật toán tách thành file riêng, viết bằng C thuần để test được trên máy tính.

### Chuỗi điều khiển

```
Position Hold:  vị trí (flow) ──P──► vận tốc đặt ──PI──► góc nghiêng đặt ─┐
Tay cầm (Angle): cần roll/pitch ─────────────────────► góc nghiêng đặt ─┤
                                                                        ▼
                         góc đặt ──P──► tốc độ góc đặt ──PID──► mixer ──► 4 ESC
Alt Hold:  độ cao đặt ──► PID độ cao (Kalman z, vz) ──► ga ────────────┘
```

### Ước lượng trạng thái

| Bộ ước lượng | File | Ý tưởng |
|---|---|---|
| Tư thế (roll, pitch, yaw) | `app/attitude.c`, `lib/kalman1d.c` | Kalman 1D mỗi trục: gyro dự báo, gia tốc kế sửa roll/pitch |
| Độ cao | `app/alt_est.c`, `lib/alt_kf.c` | Kalman 2 trạng thái (z, vz): gia tốc dự báo, ToF đo; lọc gai ToF, phát hiện bậc, baro dự phòng |
| Vận tốc ngang | `app/flow_est.c` | Kalman (v, bias gia tốc) mỗi trục: gia tốc ngang dự báo, optical flow (đã bù quay bằng gyro, trễ 2 frame) đo |

## Cấu trúc thư mục

```
App/                 code của nhóm
  app_main.c         khởi tạo, tạo 3 task
  app/               logic: task, ước lượng, điều khiển, arming, tham số, telemetry
  lib/               thuật toán C thuần (PID, mixer, lọc, Kalman, CRC) – test trên PC
  drivers/           driver cảm biến/giao thức: BMI323, DPS310, IST8310, MTF-01P (MSPv2), SBUS, UBX
  bsp/               lớp phần cứng: SPI IMU, PWM ESC, UART (tay cầm, flow, GPS, radio), USB CDC, flash
Core/, Drivers/, Middlewares/, USB_DEVICE/   do STM32CubeMX sinh (HAL, FreeRTOS, USB)
Drone.ioc            cấu hình CubeMX
tests/               test trên máy tính cho lib/ và các thuật toán trong app/
tools/               build dòng lệnh, Makefile, script phân tích (hiệu chuẩn flow, replay Kalman độ cao)
gcs/                 trạm mặt đất (Python + PySide6): đồ thị, chỉnh tham số, ghi log
PROJECT_PLAN.md      thiết kế chi tiết, PROGRESS.md: nhật ký tiến độ
```

## Build và nạp

**STM32CubeIDE:** mở thư mục này như một project (File → Import → Existing Projects into Workspace), build cấu hình *Debug* rồi nạp bằng ST-Link.

**Dòng lệnh** (dùng GCC đi kèm STM32CubeIDE; đặt `CUBEIDE_DIR` nếu IDE cài ở chỗ khác):

```powershell
.\tools\build.ps1          # ra build/Drone.elf, .hex, .bin
```

Code trong `App/` được build với `-Wall -Wextra -Wdouble-promotion` và giữ ở mức **0 warning**.

## Test trên máy tính

Cần GCC cho PC (mặc định MSYS2 UCRT64, đổi bằng biến `HOST_GCC_DIR`):

```powershell
.\tests\run_tests.ps1      # in ALL TESTS PASSED nếu đạt
```

Test gồm: bộ lọc, PID, mixer, Kalman tư thế/độ cao (bay qua bàn, gai ToF, mất ToF, bias gia tốc…), ước lượng flow, giữ vị trí, giữ độ cao, arming, tham số, giao thức SBUS/MSPv2/UBX.

## Trạm mặt đất (GCS)

```powershell
pip install -r gcs/requirements.txt
python gcs/gcs.py
```

Kết nối qua cổng USB của mạch hoặc qua module HC-05 (UART 115200). Log được lưu vào `gcs/logs/` (không đưa lên repo vì nặng).

Một số lệnh hay dùng: `SET <tên> <giá trị>`, `SAVE`, `CAPTURE` (ghi rung tần số cao), `FLOWLOG 1/0`, `LEVELCAL`, `DISARM`.

## Tài liệu tham khảo

- R. Beard, T. McLain – *Small Unmanned Aircraft: Theory and Practice*
- Carbon Aeronautics – *Quadcopter Manual*
- Datasheet: Bosch BMI323, Infineon DPS310, iSentek IST8310, MicoAir MTF-01P, u-blox M10
- Ý tưởng tham khảo (không chép code): PX4, ArduPilot, Betaflight

Code firmware cũ trước khi viết lại không nằm trong nhánh này.
