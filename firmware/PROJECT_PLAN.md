# PROJECT_PLAN — Firmware quadcopter giữ thăng bằng và giữ độ cao (STM32F405 + FreeRTOS)

> Tài liệu này dùng để giao việc cho **Claude Code**. Đọc toàn bộ trước khi viết dòng code đầu tiên.
> Có thể đặt file ở thư mục gốc của repo, và tạo `CLAUDE.md` chỉ chứa một dòng: `Đọc và làm theo PROJECT_PLAN.md`.

---

## 0. Quy tắc làm việc cho Claude Code (bắt buộc)

1. **Làm theo từng mốc nhỏ** (mục 11). Kết thúc mỗi mốc phải: build sạch (0 warning mới), hướng dẫn người dùng nạp code và kiểm tra, sau đó **dừng lại chờ kết quả**. Không làm dồn nhiều mốc.
2. **Không đoán** chân MCU, địa chỉ I2C, giao thức tay cầm, giao thức ESC hay hướng gắn cảm biến. Mọi giá trị đánh dấu `TBD` (mục 13) phải **hỏi người dùng** trước bước cần đến nó.
3. **An toàn motor là ưu tiên số 1** (mục 10):
   - Không viết code tự arm. Không làm motor quay khi chưa được người dùng xác nhận đã **tháo cánh quạt**.
   - Mọi thay đổi ở ESC, arm/disarm, failsafe, mixer phải **nêu rõ trong tóm tắt** để người dùng tự đọc lại.
4. **Không copy nguyên văn code GPL** (Betaflight, ArduPilot đều GPLv3) vào dự án. Chỉ đọc để tham khảo ý tưởng. PX4 (BSD-3) được phép tham khảo gần hơn, nhưng vẫn ghi nguồn.
5. **Giá trị thanh ghi lấy từ datasheet**, không lấy từ trí nhớ. Datasheet nằm trong `docs/`. Chỗ nào chưa chắc thì ghi `// VERIFY:` kèm số trang hoặc mục của datasheet.
6. Ghi tiến độ vào `PROGRESS.md`: mốc nào xong, đo được gì, vấn đề còn mở.
7. Code: C11, **chỉ dùng `float`** (hằng số có hậu tố `f`, dùng `sinf/cosf/atan2f/sqrtf`), bật `-Wall -Wextra -Wdouble-promotion`. Không `malloc` sau khi khởi động. Không dùng `HAL_Delay` trong task.
8. **RTOS: dùng API CMSIS-RTOS v1** (`cmsis_os.h` do CubeMX sinh, giao diện CMSIS_V1). Chỉ có hai ngoại lệ được dùng API gốc FreeRTOS: macro vùng găng (`taskENTER_CRITICAL[_FROM_ISR]`) và hàm chẩn đoán (`uxTaskGetStackHighWaterMark`). Trong ISR chỉ gọi những hàm CMSIS v1 an toàn với ngắt (`osSignalSet`); **VERIFY** bằng cách đọc `cmsis_os.c` của project, kiểm tra hàm có nhánh `inHandlerMode()`.
9. **Không thêm task, queue, mutex hay timer ngoài thiết kế ở mục 4** khi chưa hỏi người dùng (lý do: mục 4.3).
10. **Dùng lại code cũ của người dùng** cho tay cầm và ESC (đã đọc được tay cầm, đã hiệu chuẩn ESC, đã điều khiển được motor). Đọc code cũ **trước** khi viết driver mới (mục 6.0). **Giữ nguyên dải độ rộng xung ESC mà code cũ dùng**, vì ESC đã được hiệu chuẩn theo dải đó. Không chạy lại thủ tục hiệu chuẩn ESC.
11. **Comment vừa đủ, chỉ ở chỗ quan trọng.** Người dùng không muốn code nhiều comment vì khó đọc.
    - **Có comment khi:** giải thích *vì sao* (không phải *làm gì*); đơn vị và hệ trục của biến khi tên chưa nói rõ; giá trị thanh ghi kèm nguồn datasheet (ví dụ `// BMI323 DS §6.1: ODR 3.2 kHz, high-perf`); công thức không hiển nhiên (Kalman, mixer, bù nghiêng); logic an toàn (arm, failsafe, ESC); giả định về thời gian (ISR, vùng găng, ưu tiên ngắt); các thẻ `// VERIFY:`.
    - **Không comment:** điều code đã nói rõ (`i++; // tăng i`); khung trang trí hay banner; code bị comment-out (xóa đi, Git đã lưu lịch sử); comment mỗi dòng.
    - Đầu file: tối đa 1–2 dòng mô tả file làm gì.
    - Ưu tiên **tên biến và tên hàm tự giải thích** (`rate_sp_rad_s`, `tof_valid`) thay cho comment.

---

## 1. Tổng quan dự án

- **Đề tài:** "Thiết kế chế tạo quadcopter và xây dựng thuật toán giữ độ cao, vị trí cho quadcopter trong môi trường khép kín."
- **Mục tiêu của đợt này (3 ngày):**
  1. **Bay ổn định** bằng tay cầm RF (chế độ Angle/Stabilize).
  2. **Giữ độ cao** trong nhà bằng ToF (chế độ Alt Hold).
- **Firmware tự viết** trên STM32F405RGT6, dùng FreeRTOS với API CMSIS-RTOS v1. Trạm mặt đất bằng Python, **tạm thời kết nối qua cổng USB-C của F405 (USB CDC, cổng COM ảo)**; chuyển sang Bluetooth HC-05 ở giai đoạn sau.

### 1.1 Phạm vi

| Trong phạm vi | Ngoài phạm vi (làm sau) |
|---|---|
| FreeRTOS, SPI DMA cho IMU | Giữ vị trí ngang bằng optical flow |
| Kalman 1D cho roll, pitch, yaw (theo Carbon Aeronautics) | Baro làm **phép đo chính** trong Kalman, hoặc Kalman 3 trạng thái [z, v_z, b_baro] |
| Kalman 2D [z, v_z] cho độ cao (theo Carbon Aeronautics); **phép đo chính là ToF (đã chốt)** | GPS (không dùng trong nhà) |
| Phát hiện vật cản bên dưới bằng bước nhảy ToF + độ lệch địa hình `h_terrain`; **giữ độ cao so với sàn lúc cất cánh (đã chốt)** | Bay bám theo bề mặt (nâng lên theo mặt bàn) |
| Baro: **chỉ để kiểm tra nhất quán dài hạn và dự phòng khi mất ToF** (mục 7.4b) | |
| Cảm biến: gyro, accel (BMI323), ToF (MTF-01P), la bàn + baro (GEP-M10-DI) | DShot, FIFO của BMI323 |
| PID phân tầng, mixer quad X, arm/disarm, failsafe | EKF quaternion |
| GUI Python: đồ thị realtime, chỉnh và lưu tham số, ghi log | Giao diện đẹp |
| Trạm mặt đất qua **USB CDC** (cổng USB-C trên mạch) | Trạm mặt đất qua HC-05 (làm sau; giao thức giữ nguyên, chỉ đổi lớp truyền) |

---

## 2. Phần cứng

| Thành phần | Linh kiện | Giao tiếp | Ghi chú |
|---|---|---|---|
| MCU | STM32F405RGT6 | — | Cortex-M4F 168 MHz, FPU đơn, flash 1 MB, SRAM 192 KB (+64 KB CCM) |
| IMU | Bosch BMI323 | SPI 4 dây + INT1 | Datasheet BST-BMI323-DS000-13 |
| La bàn | Trên module GEPRC GEP-M10-DI | I2C | Các nguồn ghi là IST8310 → **phải quét I2C để xác nhận chip** (mục 13) |
| Baro | Trên module GEP-M10-DI (các nguồn ghi DPS310 hoặc DPS368) | I2C, chung bus với la bàn | Xác nhận chip bằng quét I2C. **Chỉ dùng để kiểm tra và dự phòng**, không phải phép đo chính |
| ToF + optical flow | MicoAir MTF-01P | UART 115200, 3,3 V TTL, cấp 5 V | Đặt giao thức **MSP** bằng phần mềm MicoAssistant; 100 Hz; tầm ToF 12 m; vùng mù 2 cm |
| Trạm mặt đất (hiện tại) | Cổng USB-C nối vào USB OTG FS của F405 (PA11 = D−, PA12 = D+) | USB CDC (Full-speed) | **Cần thạch anh HSE** để có xung 48 MHz chính xác cho USB. Chân VBUS sense (PA9) có nối hay không: `TBD` |
| Bluetooth (giai đoạn sau) | HC-05 | UART | Mặc định 9600 baud → đổi sang 115200 bằng lệnh AT `AT+UART=115200,0,0` |
| Tay cầm | Bộ thu RF | `TBD` | Lưu ý: USART của STM32F4 **không đảo được mức RX**. SBUS cần mạch đảo ngoài; CRSF, iBUS và PPM thì không cần |
| ESC | 30 A, ghi "2S–3S" | PWM 4 kênh | Giao thức `TBD`. **Không dùng pin trên 3S** khi chưa xác nhận ESC chịu được |
| Khung | Quad X | — | Thứ tự và chiều quay motor: mục 7.6 |

---

## 3. Kiến trúc phần mềm

### 3.1 Cấu trúc thư mục

```
repo/
├─ PROJECT_PLAN.md, PROGRESS.md, CLAUDE.md
├─ docs/                     # datasheet BMI323, chip la bàn, RM0090, DS F405, ảnh tài liệu Carbon Aeronautics
├─ legacy/                   # CODE CŨ của người dùng: CHỈ ĐỌC, không sửa, không đưa vào build (mục 6.0)
├─ firmware/
│  ├─ Core/, Drivers/, Middlewares/      # do CubeMX sinh, chỉ sửa trong USER CODE
│  └─ App/
│     ├─ bsp/       spi_imu.c, uart_dma.c, i2c_bus.c, pwm_esc.c, time_us.c (DWT), dbg_gpio.c, flash_params.c,
│     │             usb_cdc_port.c (ring buffer RX/TX bọc quanh CDC_Transmit_FS / CDC_Receive_FS)
│     ├─ drivers/   bmi323.c, mag.c, mtf01p_msp.c, rc_<proto>.c, esc.c
│     ├─ lib/       filter.c (pt1, biquad, notch), kalman1d.c, kalman2d.c, pid.c, mixer.c, mathx.c
│     ├─ app/       task_ctrl.c, task_slow.c, task_telem.c, arming.c, failsafe.c,
│     │             calib.c, params.c, telem_proto.c, telem_transport.c/.h (USB CDC hiện tại, UART/HC-05 sau),
│     │             shared_state.c/.h
│     └─ app_main.c # tạo task, khởi tạo; gọi từ main.c (USER CODE)
├─ tests/                    # test lib/ trên PC (gcc), không phụ thuộc HAL
├─ gcs/                      # GUI Python
└─ tools/                    # replay_kf.py: chạy lại Kalman trên log để so sánh
```

### 3.2 Quy tắc phân lớp

1. **`lib/` là C thuần**: không include HAL, không include FreeRTOS. Phải biên dịch và test được trên PC.
2. **`drivers/` không biết đến RTOS**, và **trả về đơn vị SI** (rad/s, m/s², m, µT hoặc giá trị tương đối).
3. **`app/`** lo đồng bộ, thời gian và chia sẻ dữ liệu.
4. **Đơn vị nội bộ:** SI và **radian**. Độ chỉ dùng khi hiển thị trên GUI.
5. **Hệ trục thân:** **FRD** (x tiến, y phải, z xuống). Hệ đất: **NED**. Roll dương là cánh phải hạ xuống, pitch dương là mũi ngẩng lên, yaw dương là quay theo chiều kim đồng hồ khi nhìn từ trên xuống.
6. **Trục cảm biến → trục thân** dùng một ma trận đổi trục (hoán vị + dấu) **khai báo trong tham số**, xác định bằng thử nghiệm ở mốc M3.

### 3.3 Dữ liệu dùng chung (`shared_state.h`)

```c
typedef struct { uint32_t t_us; float gyro[3]; float acc[3]; } imu_sample_t;          // rad/s, m/s^2 (FRD, đã hiệu chuẩn)
typedef struct { uint32_t t_us; float roll, pitch, yaw; float P[3]; } attitude_t;      // rad
typedef struct { uint32_t t_us; float z, vz, h_terrain; uint8_t src; bool valid; } alt_state_t; // m, m/s; z dương lên, so với sàn lúc cất cánh; src: 0 = ToF, 1 = baro dự phòng, 2 = chỉ dự báo
typedef struct { uint32_t t_us; float dist_m; uint8_t quality; bool valid; } tof_t;
typedef struct { uint32_t t_us; float m[3]; bool valid; } mag_t;
typedef struct { uint32_t t_us; float alt_rel_m; bool valid; } baro_t;                    // độ cao tương đối so với lúc khởi động, đã lọc
typedef struct { uint32_t t_us; float roll, pitch, yaw, thr; bool arm_sw; uint8_t mode_sw; bool valid; } rc_t; // stick chuẩn hóa [-1,1], thr [0,1]
typedef struct { float motor[4]; } motor_out_t;                                         // [0,1]
```

- **Đồng hồ micro giây:** dùng `DWT->CYCCNT` (168 MHz). Lấy hiệu bằng phép trừ `uint32_t` để không lỗi khi tràn số.

### 3.4 Đồng bộ dữ liệu dùng chung (không dùng mutex, nhưng **bắt buộc có bảo vệ**)

**Nguyên tắc:** mỗi biến dùng chung có **đúng một nơi ghi**. Nơi đọc lấy **bản sao nguyên vẹn** của cả struct. Không đọc hay ghi trực tiếp từng trường của biến dùng chung từ một ngữ cảnh khác.

| Dữ liệu | Nơi ghi | Nơi đọc | Cơ chế |
|---|---|---|---|
| `imu_sample_t` (đã lọc) | ISR DMA của IMU (3200 Hz) | `task_ctrl` | Task chép struct trong `taskENTER_CRITICAL()`. Vì ISR IMU có ưu tiên 5, nó bị chặn trong lúc chép (dưới 1 µs), nên không bao giờ đọc phải dữ liệu đang ghi dở |
| Tổng a_up 10 ms | ISR DMA của IMU | `task_slow` | Vùng găng: `task_slow` chép tổng và số mẫu rồi xóa về 0 trong `taskENTER_CRITICAL()` |
| `attitude_t`, `motor_out_t`, cờ arm | `task_ctrl` | `task_slow`, `task_telem` | Chép struct trong `taskENTER_CRITICAL()` |
| `alt_state_t`, `tof_t`, `mag_t`, `baro_t`, lệnh ga từ vòng độ cao | `task_slow` | `task_ctrl`, `task_telem` | Chép struct trong `taskENTER_CRITICAL()` |
| `rc_t` | `task_slow` (phân tích gói RC) | `task_ctrl` | Chép struct trong `taskENTER_CRITICAL()` |
| Tham số | `task_telem` (lệnh SET) | Các task khác | `task_telem` ghi vào bảng tạm và bật cờ; `task_ctrl` áp dụng ở **đầu chu kỳ** của nó (chép trong vùng găng) |
| Bộ đệm USB TX | Chỉ `task_telem` | Ngắt USB | Chỉ một task gửi, nên không cần khóa |

**Lý do không dùng mutex ở đây:**
- **ISR không lấy được mutex**, mà dữ liệu IMU lại do ISR ghi.
- Mutex trong `task_ctrl` có thể làm task này **phải chờ** một task ưu tiên thấp hơn, gây jitter cho vòng rate. Kể cả có kế thừa ưu tiên thì thời gian chờ vẫn không xác định trước được.
- Vùng găng chỉ để chép khoảng 20–60 byte, mất **dưới 1 µs**, nên ngắn hơn và dễ dự đoán hơn khóa mutex.

**Lưu ý khi dùng vùng găng:**
- `taskENTER_CRITICAL()` chỉ chặn các ngắt có số ưu tiên **≥ 5** (`configMAX_SYSCALL_INTERRUPT_PRIORITY`). Vì vậy **mọi ISR chạm vào dữ liệu dùng chung phải có ưu tiên ≥ 5**, đúng như cấu hình NVIC ở mục 5.
- Biến dùng chung khai báo `volatile` (hoặc chép bằng `memcpy` bên trong vùng găng).
- **Không bao giờ** làm việc dài trong vùng găng: không printf, không gọi HAL, không tính toán.
- Mutex **chỉ được dùng** nếu sau này có hai task cùng dùng một bus (ví dụ I2C). Hiện tại la bàn và baro cùng bus I2C nhưng **đều chỉ do `task_slow` đọc**, nên **không cần**.

---

## 4. Thời gian và luồng dữ liệu (đã chốt)

| Khâu | Tần số | Chu kỳ | Nơi chạy |
|---|---|---|---|
| Gyro BMI323 (ODR) | **3200 Hz** | 312,5 µs | Chip |
| Accel BMI323 (ODR) | 1600 Hz | 625 µs | Chip (đọc cùng gói với gyro) |
| Đọc IMU (SPI DMA, 14 byte @ 5,25 MHz ≈ 21 µs) | 3200 Hz | 312,5 µs | ISR (EXTI → DMA) |
| LPF/notch gyro + accel, cộng dồn accel | 3200 Hz | — | ISR khi DMA xong |
| **Vòng rate PID + mixer + xuất ESC** | **≈1066,7 Hz** (mỗi 3 mẫu) | ≈0,9375 ms | `task_ctrl` |
| **Kalman 1D tư thế + vòng góc** | ≈533,3 Hz (mỗi 2 chu kỳ ctrl) | ≈1,875 ms | `task_ctrl` |
| Kalman 2D độ cao + PID độ cao, đọc la bàn và baro | **100 Hz** | 10 ms | `task_slow` |
| MTF-01P (MSP) | 100 Hz | — | UART DMA + ngắt IDLE → `task_slow` phân tích gói |
| Telemetry USB CDC | 50 Hz (tham số, tối đa 100 Hz) | 20 ms | `task_telem` |

### 4.1 Chuỗi xử lý IMU

```
BMI323 INT1 (gyro data-ready, cạnh lên)
  → EXTI ISR: ghi t_us = DWT; kéo CS xuống; HAL_SPI_TransmitReceive_DMA(14 byte)
  → DMA xong (ISR): kéo CS lên; chuyển đổi đơn vị; đổi trục sang FRD; trừ bias;
                     LPF (+ notch nếu bật); cộng dồn a_up cho task_slow;
                     n++; nếu n % 3 == 0 → osSignalSet(ctrlTaskHandle, SIG_IMU)
  → task_ctrl (osSignalWait(SIG_IMU, osWaitForever)): rate PID → mixer → ESC; mỗi 2 lần: KF tư thế + vòng góc
```

- Dùng `dt` thật lấy từ timestamp. Không viết cứng `dt`.
- `task_ctrl` **cho ăn watchdog IWDG** sau mỗi chu kỳ hợp lệ.

### 4.1b Nguồn kích hoạt cho từng cảm biến (không có ngắt timer tuần hoàn để đọc cảm biến)

| Cảm biến | Báo có dữ liệu mới | Chuyển byte | Xử lý | Nhịp |
|---|---|---|---|---|
| BMI323 | Chân INT1 (data-ready) → EXTI | SPI DMA | ISR khi DMA xong (đổi đơn vị, lọc), sau đó `task_ctrl` | Đồng hồ của cảm biến, 3200 Hz |
| MTF-01P | Cảm biến tự gửi gói | UART DMA vòng tròn + ngắt IDLE (ISR chỉ gọi `osSignalSet`) | `task_slow` phân tích gói MSP | 100 Hz, theo cảm biến |
| La bàn | Không có chân ngắt | I2C (đọc chờ chấp nhận được vì nằm ở task ưu tiên thấp hơn `task_ctrl`) | `task_slow` | Mỗi lần `task_slow` chạy (khoảng 100 Hz) |
| Baro | Không dùng chân ngắt | I2C, cùng bus | `task_slow` (đọc xen kẽ với la bàn) | Chip tự đo liên tục ở 32–64 Hz; `task_slow` đọc kết quả mới nhất mỗi 10–20 ms |
| Tay cầm | Bộ thu tự gửi (SBUS/CRSF/iBUS); PPM thì dùng timer input capture | UART DMA + IDLE | `task_slow` | Theo bộ thu |
| ESC | — | Timer PWM phần cứng, không ngắt | `task_ctrl` ghi CCR | 400 Hz |

- **Không** đọc IMU bằng ngắt timer. Đồng hồ của BMI323 lệch tới 1,7% so với MCU, nên đọc theo timer sẽ lúc trùng mẫu, lúc bỏ sót mẫu, và có jitter. Đọc theo data-ready thì mỗi mẫu được đọc đúng một lần, ngay khi có.
- **ISR chỉ làm việc ngắn**: khởi động DMA, chép byte, đánh thức task. Ngoại lệ duy nhất là đổi đơn vị và lọc gyro trong ISR DMA, vì phải chạy cho từng mẫu ở 3200 Hz và chỉ tốn vài µs.

### 4.2 FreeRTOS với API CMSIS-RTOS v1

| Task | Ưu tiên (CMSIS v1) | Stack (word, khởi điểm) | Kích hoạt |
|---|---|---|---|
| `task_ctrl` | `osPriorityRealtime` | 1024 | `osSignalWait(SIG_IMU, osWaitForever)`; ISR gọi `osSignalSet` |
| `task_slow` | `osPriorityHigh` | 1024 | `osSignalWait(SIG_TOF, 15)`: **đánh thức khi có gói ToF mới** (ngắt IDLE của UART, khoảng 100 Hz). Hết 15 ms mà không có gói thì vẫn chạy (coi như ToF mất), để Kalman độ cao vẫn dự báo và vẫn đọc la bàn, baro |
| `task_telem` | `osPriorityBelowNormal` | 1024 | `osDelayUntil(&last, 20)`; thêm `osSignalWait(SIG_USB_RX, 0)` để xử lý lệnh từ PC |

**Bảng API dùng trong dự án:**

| Việc | CMSIS-RTOS v1 |
|---|---|
| Tạo task | `osThreadDef(name, func, prio, 0, stack_words)` + `osThreadCreate(osThread(name), arg)` |
| Đánh thức task từ ISR | `osSignalSet(handle, SIG_x)` (bản của ST tự gọi phiên bản FromISR khi ở trong ngắt; **VERIFY** trong `cmsis_os.c`) |
| Chờ tín hiệu | `osSignalWait(SIG_x, timeout)` |
| Chu kỳ cố định | `osDelayUntil(&prev_wake, ms)` (**không** dùng `osDelay` cho vòng tuần hoàn) |
| Khởi động | `osKernelStart()`, chỉ gọi một lần (CubeMX đã sinh) |
| Vùng găng | `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()`; trong ISR dùng `taskENTER_CRITICAL_FROM_ISR()` (ngoại lệ được phép, mục 0.8) |
| Không dùng trong vòng điều khiển | `osMutex*`, `osMessage*`, `osMail*`, `osTimer*` |

- Trong CubeMX phần FreeRTOS: chọn interface **CMSIS_V1**; `configUSE_TASK_NOTIFICATIONS = 1` (để `osSignal*` hoạt động); `configMAX_PRIORITIES = 7`.
- Bật `configASSERT`, `configCHECK_FOR_STACK_OVERFLOW = 2`, `configUSE_MALLOC_FAILED_HOOK = 1`. Heap: `heap_4`, khởi điểm khoảng 24 KB (3 task × 4 KB stack + USB + dư).
- Đo thời gian bằng GPIO debug: bật ở đầu ISR, đầu và cuối `task_ctrl`; in jitter lớn nhất và thời gian chạy lớn nhất qua telemetry.

### 4.3 Vì sao chỉ dùng RTOS ở mức tối giản

| | Không RTOS (vòng lặp chính + ngắt) | **RTOS tối giản (dự án này)** | RTOS "đầy đủ" (mỗi cảm biến một task, queue nối các task, mutex mọi nơi) |
|---|---|---|---|
| Vòng rate có bị việc chậm làm trễ không | Có, nếu USB, I2C, phân tích gói nằm chung vòng lặp chính. Muốn tránh thì phải viết mọi thứ thành máy trạng thái không chặn | **Không**: `task_ctrl` luôn chen ngang task thấp hơn | Không, nhưng mỗi chặng queue cộng thêm thời gian lập lịch và jitter |
| Độ trễ từ IMU đến ESC | Thấp nhất | Thấp: chỉ một lần chuyển ngữ cảnh (cỡ vài µs) | Cao hơn: cảm biến → queue → ước lượng → queue → điều khiển |
| Viết code cho phần chậm (USB, I2C, GUI) | Khó hơn | **Dễ**: được viết kiểu chờ, vì đã nằm ở task ưu tiên thấp | Dễ |
| Chi phí CPU | Gần 0 | Khoảng 1.300 lần chuyển ngữ cảnh/s × vài µs, cỡ **dưới 1% CPU** (ước lượng; đo lại ở M1) | Cao hơn, tăng theo số task và queue |
| RAM | Ít nhất | Khoảng 12 KB stack + heap | Nhiều hơn đáng kể |
| Lỗi đồng bộ có thể gặp | Ít | Ít, vì chỉ 3 task và một nơi ghi cho mỗi dữ liệu | Nhiều: deadlock, đảo ngược ưu tiên, queue đầy |
| Gỡ lỗi thời gian | Dễ | Dễ, đo được bằng GPIO | Khó |

Tham khảo: ngay cả PX4 (một hệ RTOS đầy đủ) cũng giữ đường nóng ngắn. Bộ điều khiển rate (`MulticopterRateControl`) chạy trên work queue riêng `rate_ctrl` và được **kích trực tiếp bởi dữ liệu gyro mới** (callback trên `vehicle_angular_velocity`). Dự án này làm theo cùng nguyên tắc: **IMU → `task_ctrl` → ESC là một chặng duy nhất**, còn RTOS dùng để cách ly các việc chậm khỏi đường đó.

---

## 5. Cấu hình CubeMX (`.ioc`): người dùng làm, Claude Code kiểm tra lại

| Mục | Cấu hình |
|---|---|
| RCC | **HSE thạch anh (bắt buộc cho USB)**, tần số `TBD`, thường 8 MHz. PLL: với HSE 8 MHz thì M = 8, N = 336, **P = 2 → SYSCLK 168 MHz**, **Q = 7 → 48 MHz cho USB**. AHB 168, **APB1 42**, **APB2 84**. **CSS: tắt** |
| SYS | Debug: Serial Wire. **Timebase: TIM7** (không dùng SysTick) |
| FreeRTOS | **CMSIS_V1**, heap_4, `configTICK_RATE_HZ = 1000`, `configUSE_TASK_NOTIFICATIONS = 1`, `LIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5` |
| USB_OTG_FS | Mode: **Device_Only**. VBUS sensing: bật nếu PA9 nối với VBUS của cổng USB-C, nếu không thì **tắt** (`TBD`). Middleware **USB_DEVICE**, class **Communication Device Class (VCP)** |
| SPI (IMU) | Full-duplex master, 8 bit, MSB first, **CPOL=0, CPHA=1Edge (mode 0)**, NSS: phần mềm (GPIO). Prescaler: **SPI1 → /16 = 5,25 MHz**; SPI2/3 → /8 = 5,25 MHz. Không vượt 10 MHz (giới hạn của BMI323). DMA cho RX và TX |
| GPIO | CS của IMU: output, mặc định **mức cao**. INT1: **EXTI cạnh lên**. 2–3 chân debug output |
| USART (MTF-01P) | 115200 8N1, **RX DMA circular + ngắt IDLE** |
| USART (HC-05) | **Giai đoạn sau.** 115200 8N1, TX DMA normal, RX DMA circular + IDLE. Có thể để dành sẵn chân |
| Tay cầm | Theo giao thức `TBD` (UART DMA, hoặc timer input capture cho PPM) |
| I2C (la bàn + baro) | 400 kHz (kiểm tra điện trở kéo lên của module ở mức 3,3 V) |
| TIM cho ESC | PWM 4 kênh, tick 1 MHz (timer trên APB1: PSC = 83; trên APB2: PSC = 167), ARR = 2499 → **400 Hz**, xung 1000–2000 µs |
| IWDG | Timeout ≈ 100 ms. Gọi `__HAL_DBGMCU_FREEZE_IWDG()` để không reset khi dừng ở breakpoint |
| NVIC | Ngắt **có gọi API RTOS hoặc chạm dữ liệu dùng chung** (EXTI IMU, DMA SPI, USART, DMA USART, **OTG_FS**): ưu tiên **≥ 5**. EXTI và DMA của IMU dùng 5 (cao nhất được phép). UART và OTG_FS dùng 6–7. TIM7 timebase để 0 |
| Linker | **Dành riêng sector 11** (128 KB tại `0x080E0000`) để lưu tham số: giảm vùng FLASH trong linker script còn 896 KB |
| Code gen | "Generate peripheral initialization as a pair of .c/.h". Chỉ sửa trong khối USER CODE |

---

## 6. Driver

### 6.0 Code cũ của người dùng: đọc, đánh giá, rồi quyết định dùng lại

Code cũ đã chạy được (đọc tay cầm, ESC đã hiệu chuẩn, điều khiển được motor), nhưng "chạy được" chưa chắc là "đủ tốt để bay". Làm theo 4 bước sau, **xong mới viết driver mới**.

**Bước 1: Kiểm kê.** Đọc toàn bộ code cũ trong thư mục **`legacy/`** (người dùng tự chép vào; **chỉ đọc, không sửa, không đưa vào build**), liệt kê mọi module. Với tay cầm và ESC, ghi vào `PROGRESS.md`:
- Tay cầm: giao thức, chân và ngoại vi (UART hay timer), thứ tự kênh, giá trị min/mid/max của từng kênh, cách phát hiện mất tín hiệu.
- ESC: timer và kênh, **tần số PWM**, **tick timer (độ phân giải)**, **dải độ rộng xung** (ví dụ 1000–2000 µs), giá trị dừng và idle.
- Code dùng HAL, LL hay thanh ghi; có dùng ngắt hay DMA; ưu tiên ngắt bao nhiêu.
- Các phần khác trùng với kế hoạch (nếu có, ví dụ đọc IMU, PID): liệt kê để đánh giá theo cùng cách.

**Bước 2: Đánh giá theo bảng kiểm tra**, đối chiếu với datasheet và mã nguồn tham khảo ở mục 14. Ghi kết quả **Đạt / Không đạt / Không áp dụng** cho từng dòng, kèm dẫn chứng (file:dòng trong code cũ, và tài liệu đối chiếu).

| # | Tiêu chí | Đối chiếu với |
|---|---|---|
| **Tay cầm** | | |
| R1 | Kiểm tra tính hợp lệ của khung: byte đồng bộ, độ dài, **CRC hoặc checksum** (CRSF: CRC8 DVB-S2; iBUS: checksum; SBUS: byte cuối và cờ) | Tài liệu giao thức; Betaflight `src/main/rx/` |
| R2 | Xử lý khung bị cắt hoặc lệch đồng bộ, tự đồng bộ lại | Như trên |
| R3 | **Phát hiện mất tín hiệu**: timeout khi không có khung hợp lệ, và đọc cờ failsafe/frame-lost của bộ thu (nếu giao thức có) | Betaflight `failsafe.c` (giai đoạn 1 mặc định 1,5 s, cần 0,5 s tín hiệu tốt để hồi phục); mục 7.7 |
| R4 | Nhận dữ liệu không chặn CPU: ưu tiên **DMA + IDLE**; ngắt từng byte chấp nhận được ở tốc độ baud thấp | Mục 4.1b |
| R5 | Dữ liệu chia sẻ với nơi đọc được chép nguyên vẹn (không đọc dở khung) | Mục 3.4 |
| R6 | Chuẩn hóa kênh, có vùng chết quanh tâm, giới hạn giá trị | Mục 6.4 |
| **ESC** | | |
| E1 | Tần số và dải xung **khớp với dải ESC đã hiệu chuẩn**; luôn giới hạn (clamp) trong dải | Mục 6.5 |
| E2 | Độ phân giải đủ: tick timer ≥ 1 MHz (≥ 1000 mức cho 1000–2000 µs) | Mục 5 |
| E3 | **Bật preload cho CCR** (bit OCxPE) để đổi giá trị giữa chu kỳ không tạo xung lỗi | RM0090 (phần timer) |
| E4 | **An toàn khi khởi động và reset**: chỉ xuất xung dừng cho đến khi arm; không có đường code nào xuất ga > dừng khi chưa arm | Mục 7.7, mục 10 |
| E5 | Thứ tự motor và chân ra khớp bảng 7.6, hoặc có bảng ánh xạ cấu hình được | Mục 7.6 |
| **Chung** | | |
| G1 | Không dùng `HAL_Delay` hay vòng chờ bận trong đường xử lý định kỳ | Mục 0.7 |
| G2 | Ưu tiên ngắt tương thích FreeRTOS (≥ 5 nếu gọi API RTOS hoặc chạm dữ liệu dùng chung) | Mục 5 |
| G3 | Không dùng `double` ngầm, không cảnh báo khi build với `-Wall -Wextra -Wdouble-promotion` | Mục 0.7 |
| G4 | Biến dùng chung với ngắt khai báo `volatile` và được bảo vệ | Mục 3.4 |

**Bước 3: Quyết định cho từng module**, ghi thành bảng trong `PROGRESS.md` (module | quyết định | lý do):
- **Giữ nguyên**: đạt tất cả tiêu chí liên quan. Chỉ bọc lại theo giao diện `drivers/`.
- **Giữ và sửa**: có tiêu chí không đạt nhưng sửa được gọn (ví dụ thêm CRC, thêm preload, thêm timeout).
- **Viết lại**: sai nhiều chỗ, hoặc kiến trúc không hợp (ví dụ chờ bận trong ngắt). **Hỏi người dùng trước khi viết lại một module đang chạy được.**
- **Bất kể quyết định nào, luôn giữ các thông số vật lý đã kiểm chứng**: dải xung ESC đã hiệu chuẩn, ánh xạ kênh, giá trị min/mid/max. **Không chạy lại hiệu chuẩn ESC.**
- **Tiêu chí an toàn (R3, E1, E4) không đạt thì bắt buộc phải sửa**, không được ghi nhận rồi bỏ qua.

**Bước 4: Kiểm tra hồi quy** (tháo cánh quạt): driver mới phải cho kết quả **giống code cũ**, gồm giá trị kênh trên GUI và phản hồi motor khi đẩy ga. Riêng tần số PWM: nếu code cũ khác 400 Hz (ví dụ 50 Hz), lần chạy đầu giữ tần số cũ; sau đó thử 400 Hz, motor phản hồi mượt thì mới đổi. Ghi kết quả vào `PROGRESS.md`.

### 6.1 BMI323 (đã đối chiếu với datasheet BST-BMI323-DS000-13)

**Giao thức SPI**
- Đọc: byte 1 = `0x80 | addr`, **1 byte dummy**, sau đó mỗi thanh ghi 2 byte **LSB trước**.
- Ghi: byte 1 = `addr` (bit 7 = 0), sau đó LSB, rồi MSB. **Phải ghi đủ 2 byte**; ghi 1 byte sẽ bị bỏ qua.
- Hỗ trợ SPI mode 0 và 3, tối đa 10 MHz.
- **Đọc burst 0x03…0x08** (ACC_X..Z, GYR_X..Z): 1 + 1 + 12 = **14 byte**.

**Thanh ghi cần dùng**

| Thanh ghi | Địa chỉ | Giá trị / ghi chú |
|---|---|---|
| CHIP_ID | 0x00 | Byte thấp = **0x43** |
| ERR_REG | 0x01 | Phải bằng 0 sau khi cấu hình (có bit báo lỗi cấu hình acc/gyr) |
| STATUS | 0x02 | drdy_acc, drdy_gyr, por_detected |
| ACC_DATA_X..Z / GYR_DATA_X..Z | 0x03–0x05 / 0x06–0x08 | int16 |
| ACC_CONF | 0x20 | **0x702C** = mode 0b111 (high performance) \| bw 0 (ODR/2) \| range 0b010 (±8 g) \| ODR 0xC (1,6 kHz) |
| GYR_CONF | 0x21 | **0x704D** = mode 0b111 \| bw 0 (ODR/2) \| range 0b100 (±2000 °/s) \| ODR 0xD (3,2 kHz) |
| IO_INT_CTRL | 0x38 | **0x0005**: INT1 active high (bit 0), push-pull (bit 1 = 0), output enable (bit 2) |
| INT_CONF | 0x39 | 0x0000: non-latched (tự xóa sau khoảng 1/6400 s) |
| INT_MAP2 | 0x3B | **0x0100**: gyr_drdy_int (bit 9:8) = 01 → INT1 |
| CMD | 0x7E | Soft reset: **0xDEAF** |

**Hệ số đổi đơn vị**
- Gyro ±2000 °/s: **16,384 LSB/(°/s)**. Đổi ra rad/s: `raw / 16.384f * (M_PI/180)`.
- Accel ±8 g: **4096 LSB/g**. Đổi ra m/s²: `raw / 4096.0f * 9.80665f`.

**Trình tự khởi tạo**
1. Đợi ≥ 2 ms sau khi cấp nguồn.
2. **Đọc giả (dummy) thanh ghi 0x00 một lần.** Sau khi cấp nguồn, chip ở chế độ I2C; cần một cạnh lên trên CSB để chuyển sang SPI.
3. Đọc CHIP_ID, kiểm tra 0x43.
4. Ghi CMD = 0xDEAF, đợi ≥ 2 ms, rồi **đọc giả lại** (sau soft reset chip quay về I2C).
5. Kiểm tra lại CHIP_ID.
6. Ghi IO_INT_CTRL, INT_CONF, INT_MAP2, ACC_CONF, GYR_CONF.
7. Đợi ≥ 50 ms (thời gian khởi động gyro theo datasheet khoảng 30 ms), sau đó đọc ERR_REG, phải bằng 0.
8. Bật EXTI.

### 6.2 La bàn (GEP-M10-DI, I2C)

1. **Quét I2C** và in ra các địa chỉ tìm thấy. Thông thường IST8310 ở 0x0E, QMC5883L ở 0x0D; baro cũng nằm trên bus này. **Hỏi người dùng** và xác nhận chip trước khi viết driver.
2. Đọc ở 50–100 Hz trong `task_slow`. Driver trả về vector từ trường theo trục thân FRD.
3. Hiệu chuẩn **hard-iron**: xoay drone đủ các hướng trong khoảng 30 s, offset = (max + min)/2 trên từng trục. Lưu vào tham số. Bỏ qua soft-iron ở đợt này.

### 6.2b Baro (GEP-M10-DI, I2C, chung bus với la bàn)

1. Xác nhận chip bằng kết quả quét I2C (các nguồn bán hàng ghi DPS310 hoặc DPS368 của Infineon). Đọc thanh ghi Product ID để biết chính xác. Hai chip này được hãng mô tả là tương thích thanh ghi với nhau; **VERIFY** bằng datasheet của chip thật (đặt trong `docs/`).
2. Khởi tạo: đọc các hệ số hiệu chuẩn trong chip, cấu hình đo liên tục áp suất và nhiệt độ (32–64 Hz, oversampling vừa phải), tính áp suất đã bù nhiệt **theo công thức trong datasheet**.
3. Lúc khởi động (cùng lúc hiệu chuẩn IMU), lấy trung bình áp suất làm `p0`. Độ cao tương đối: `alt_rel = 44330 * (1 − (p/p0)^(1/5.255))` (m), qua LPF khoảng 2 Hz.
4. Che baro khỏi gió cánh quạt và ánh sáng trực tiếp bằng mút xốp (việc phần cứng của người dùng).

### 6.3 MTF-01P (MSPv2 qua UART)

- Trước tiên, người dùng đặt giao thức **MSP** bằng phần mềm MicoAssistant.
- **Khung MSPv2:** `'$' 'X' '<'|'>'` + `flag(u8)` + `cmd(u16 LE)` + `size(u16 LE)` + `payload` + `crc8_dvb_s2` (tính trên các byte từ flag đến hết payload). Chấp nhận cả hai ký tự hướng `<` và `>`. **VERIFY** bằng cách so với dữ liệu thật.
- **0x1F01, rangefinder:** `uint8 quality` (0–255), `int32 distance_mm` (**giá trị âm nghĩa là ngoài tầm**).
- **0x1F02, optical flow:** `uint8 quality`, `int32 motion_x`, `int32 motion_y`. Đợt này chỉ ghi log, chưa dùng.
- Hai gói trên được xác nhận bằng code của Betaflight (`rangefinder_lidarmt.c`) và ArduPilot (`AP_MSP/msp_sensors.h`).
- **Điều kiện ToF hợp lệ:** `distance_mm ≥ 0`, gói mới trong vòng 50 ms, độ nghiêng < 30°. Ghi log trường `quality` để xem ý nghĩa thực tế của nó.

### 6.4 Tay cầm (`TBD`)

- Viết driver theo giao thức người dùng cung cấp. Đầu ra là `rc_t` đã chuẩn hóa.
- `valid = false` nếu không có khung hợp lệ trong **> 200 ms**, hoặc bộ thu báo cờ failsafe.
- Ánh xạ kênh (roll, pitch, yaw, throttle, công tắc ARM, công tắc MODE) là **tham số**.

### 6.5 ESC

- PWM 400 Hz, xung 1000 µs (dừng) đến 2000 µs (tối đa). **Khi disarm luôn xuất 1000 µs.**
- Có `motor_idle` (ví dụ 0,05–0,08) để motor quay chậm khi đã arm.
- **Hiệu chuẩn ESC**: chỉ là thủ tục người dùng tự làm, có tháo cánh quạt và xác nhận qua GUI. Không tự động chạy khi khởi động.

---

## 7. Thuật toán

### 7.1 Hiệu chuẩn khi khởi động (drone đặt yên, nằm ngang)

- Trong 2–3 s: lấy trung bình gyro làm `gyro_bias0[3]`, lấy trung bình `a_up` làm `acc_up_bias0`. Nếu trong thời gian này rung quá ngưỡng thì **làm lại** và báo lên GUI.
- Chỉ cho phép arm sau khi hiệu chuẩn xong.

### 7.2 Lọc tín hiệu

- **Gyro:** biquad LPF, khởi điểm **150 Hz** (chạy ở 3200 Hz). Notch tùy chọn, tắt mặc định; bật sau khi đã xem FFT log rung motor.
- **D-term:** PT1 LPF, khởi điểm **80 Hz**.
- **Accel dùng cho Kalman tư thế:** PT1 LPF, khởi điểm **20–30 Hz**.
- Tham khảo mặc định của Betaflight: gyro LPF1 250 Hz (dynamic 250–500), LPF2 500 Hz, D-term LPF 75–150 Hz. Đó là cho drone FPV có gyro lọc ở chip khác; với BMI323 ở 3200 Hz, băng thông trong chip đã khoảng 531 Hz.
- Mọi tần số cắt đều là **tham số** chỉnh được qua GUI.

### 7.3 Kalman 1D tư thế (theo Carbon Aeronautics, đã sửa lỗi in)

Áp dụng riêng cho **roll** và **pitch**. **Yaw** dùng cùng cấu trúc với phép đo từ la bàn.

```
Dự báo:   angle = angle + dt * rate          // rate: tốc độ góc gyro đã trừ bias và lọc (rad/s)
          P     = P + (dt * sigma_gyro)^2    // Q = dt^2 * sigma_gyro^2
Cập nhật: K     = P / (P + sigma_meas^2)     // R = sigma_meas^2 (KHÔNG nhân dt^2)
          angle = angle + K * (meas - angle)
          P     = (1 - K) * P                // P = (I - K*H)*P, với H = 1
```

- **Hai lỗi in trong tài liệu gốc (không được chép theo):**
  1. Khung lý thuyết ghi R = Tₛ²·3², nhưng code dùng R = 3², và **code mới đúng**.
  2. Bước 5 ghi `(I − K·F)·P`, công thức đúng là `(I − K·H)·P`.
- **Khởi điểm** (theo tài liệu, đổi sang rad): `sigma_gyro = 4°/s`, `sigma_acc = 3°`, `P0 = (2°)²`, `angle0 = 0` (hoặc lấy góc từ accel lúc khởi động). Sau đó **đo lại σ trên drone thật** khi motor chạy.
- Hằng số thời gian tương đương ≈ `sigma_acc / sigma_gyro` (khoảng 0,75 s với giá trị mặc định), **không phụ thuộc dt**.
- **Phép đo từ accel** (hệ FRD, đứng yên thì a ≈ (0, 0, −g)):
  ```
  roll_acc  = atan2f(-ay, -az)
  pitch_acc = atan2f( ax, sqrtf(ay*ay + az*az))
  ```
  Kiểm tra lại bằng thử nghiệm nghiêng (mốc M3).
- **Chỉ cập nhật bằng accel khi** `0.9 g < |a| < 1.1 g` (cùng ngưỡng với Betaflight `imuIsAccelerometerHealthy`). Ngoài khoảng đó chỉ chạy bước dự báo.
- **Tùy chọn (mặc định tắt):** đổi tốc độ góc thân sang tốc độ góc Euler trước khi tích phân:
  `φ̇ = p + sinφ·tanθ·q + cosφ·tanθ·r`, `θ̇ = cosφ·q − sinφ·r`, `ψ̇ = (sinφ·q + cosφ·r)/cosθ`.
- **Yaw:** đo bằng hướng la bàn đã bù nghiêng:
  ```
  Xh = mx*cosθ + my*sinφ*sinθ + mz*cosφ*sinθ
  Yh = my*cosφ − mz*sinφ
  yaw_mag = atan2f(-Yh, Xh)
  ```
  **VERIFY** bằng cách xoay drone về các hướng đã biết. Sai lệch `(yaw_mag − yaw)` phải **đưa về khoảng [−π, π]** trước khi nhân K. Đặt `sigma_mag` lớn (ví dụ 10–20°). Khi la bàn không hợp lệ, chỉ chạy bước dự báo.

### 7.4 Kalman 2D độ cao (theo Carbon Aeronautics, đo bằng ToF)

```
x = [z, vz]^T            (z dương lên, m)
F = [[1, dt],[0, 1]]     G = [dt^2/2, dt]^T      u = a_up − acc_up_bias0
H = [1, 0]               Q = G*G^T * sigma_a^2   R = sigma_tof^2

Dự báo:   x = F x + G u ;   P = F P F^T + Q
Cập nhật: S = H P H^T + R  (là số vô hướng: P00 + R)
          K = P H^T / S    (= [P00/S, P10/S]^T)
          x = x + K (z_meas − H x)
          P = (I − K H) P  // sau đó ép đối xứng: P01 = P10 = (P01 + P10)/2
```

- **Gia tốc thẳng đứng** (FRD → hướng lên, trừ trọng lực):
  `a_up = sinθ·ax − sinφ·cosθ·ay − cosφ·cosθ·az − g`
  (đứng yên và nằm ngang: a = (0, 0, −g) cho a_up = 0). `task_slow` dùng **trung bình a_up** trong 10 ms vừa qua.
- **Phép đo:** `z_meas = dist_m · cosφ · cosθ + h_terrain` (`h_terrain` là độ cao của bề mặt bên dưới so với sàn lúc cất cánh; xem mục 7.4b). Chỉ dùng khi ToF hợp lệ (mục 6.3). Tham khảo: Betaflight `rangefinder.c` cũng nhân cos(nghiêng) và loại bỏ khi nghiêng quá giới hạn.
- **Kiểm tra outlier:** nếu `(z_meas − z)² / S > 9` thì bỏ qua mẫu đó.
- **Khởi điểm:** `sigma_a = 0.3 m/s²`, `sigma_tof = 0.05 m` (thông số chip là 2 cm; Betaflight dùng R = 100 cm², tức σ = 10 cm). `P0 = diag(0.1², 0.1²)`. **Chỉnh lại theo log.**
- Khi chưa có M10b: `alt_state.valid = false` nếu ToF mất tín hiệu quá 0,5 s. Khi đó **không cho vào hoặc phải thoát** chế độ Alt Hold. Sau M10b: dùng baro dự phòng tối đa 3 s (mục 7.4b) trước khi đặt `valid = false`.

### 7.4b Vật cản bên dưới và vai trò của baro (đã chốt với người dùng)

**Quy ước:** z là **độ cao so với sàn lúc cất cánh**. Khi bay ngang qua mặt bàn, drone **giữ nguyên độ cao so với sàn**, không nâng lên theo mặt bàn. ToF đo khoảng cách tới bề mặt bên dưới, nên `z_meas = tof_corr + h_terrain`.

**(1) Phát hiện bước nhảy ToF (không cần baro; làm ở M9/M10).** Tham chiếu trong thời gian ngắn là **bước dự báo bằng gia tốc kế**, không phải baro: drone leo/hạ tối đa khoảng 1 m/s, tức chỉ dịch khoảng 1 cm giữa hai mẫu ToF (10 ms). Ý tưởng tương tự `range_finder_consistency_check.cpp` của PX4 (so tốc độ đổi của rangefinder với vz ước lượng từ IMU).
```
mỗi mẫu ToF hợp lệ (100 Hz):
  tof_corr = dist_m * cosφ * cosθ
  jump     = (tof_corr − tof_prev) − vz * dt        // phần thay đổi mà chuyển động của drone không giải thích được
  nếu đang ở trạng thái NORMAL và |jump| > STEP_TH (khởi điểm 0,10 m):
        chuyển sang CONFIRM, lưu mức mới; KHÔNG cập nhật Kalman bằng ToF (chỉ dự báo)
  nếu ở CONFIRM:
        nếu N_CONFIRM mẫu liên tiếp (khởi điểm 3, tức 30 ms) nằm trong ±3 cm quanh mức mới:
             h_terrain = z_pred − tof_corr            // neo lại, tiếp tục dùng ToF với độ lệch mới
             về NORMAL
        nếu không ổn định trong 100 ms: coi là nhiễu, về NORMAL, giữ h_terrain cũ
  tof_prev = tof_corr
```
- Bay vào trên bàn 0,7 m: ToF giảm 0,7 m, nên h_terrain = +0,7 m, z không đổi. Bay ra: h_terrain trở về khoảng 0.
- Nếu `|h_terrain| < 0,03 m` sau khi trở về sàn thì đặt lại bằng 0.
- Giới hạn: vật thấp hơn STEP_TH (mép thảm, dây) không bị phát hiện, drone nhích vài cm (chấp nhận được). Dốc thoải không tạo bước nhảy, nên do phần (2) xử lý.

**(2) Baro kiểm tra nhất quán dài hạn (M10b).**
- Học `baro_ref = baro_alt_rel − z` bằng LPF chậm (τ khoảng 5 s), **chỉ khi**: z > 0,5 m (ngoài vùng hiệu ứng mặt đất), không ở trạng thái CONFIRM, và `h_terrain == 0`.
- Sai lệch `d = LPF_2s(baro_alt_rel − baro_ref − z)`. Nếu `|d| > 0,30 m` kéo dài > 2 s, nghĩa là có dốc thoải hoặc đã phát hiện sai bước nhảy. Khi đó: neo lại `h_terrain = (baro_alt_rel − baro_ref) − tof_corr`, gửi `MSG,ALT_REANCHOR_BARO` lên GUI.
- Ngưỡng 0,30 m chọn lớn hơn sai số baro. Theo số liệu Betaflight ghi trong `position_estimator.c`, baro lệch tệ nhất khoảng −29 cm ở độ cao 50–100 cm. Chỉnh lại theo log thật.

**(3) Baro dự phòng khi mất ToF (M10b).**
- ToF mất (không hợp lệ hoặc nghiêng > 30°) và z > 0,5 m: cập nhật Kalman bằng `z_meas = baro_alt_rel − baro_ref` với `R = (0,3 m)²`, đặt `src = 1`. Tối đa **3 s**, sau đó `valid = false` (Alt Hold thoát, hoặc failsafe hạ cánh nếu đang mất tay cầm).
- ToF quay lại: chạy bước phát hiện như (1) trước khi dùng lại, để không giật.

**(4) Không dùng baro khi z < 0,5 m** (hiệu ứng mặt đất; theo số liệu Betaflight, sai số baro tệ nhất lên tới −280 cm khi dưới 20 cm). Ở vùng này chỉ dùng ToF hoặc chỉ dự báo.

### 7.5 PID phân tầng

**Vòng rate** (≈1067 Hz, mỗi trục; tham khảo PX4 `rate_control.cpp`, Betaflight `pid.c`):
```
e      = rate_sp − rate
P      = Kp * e
I     += Ki * e * dt        // giới hạn |I| ≤ I_lim; KHÔNG tích phân khi mixer bão hòa theo chiều của e (PX4)
D      = −Kd * d(rate)/dt   // D theo giá trị đo, không theo sai số; qua LPF D-term
out    = P + I + D (+ Kff * rate_sp, tùy chọn)
```
- **Reset I** khi disarm và khi ga < ngưỡng lúc chưa cất cánh (Betaflight `pidResetIterm`, PX4 "landed").
- Đầu ra là mô-men chuẩn hóa trong khoảng [−1, 1].

**Vòng góc** (≈533 Hz): `rate_sp = Kp_ang * (angle_sp − angle)`, giới hạn trong ±`max_rate`. Với yaw ở đợt này: `yaw_rate_sp` lấy trực tiếp từ stick (không giữ hướng).

**Từ stick sang setpoint** (chế độ Angle): `angle_sp = stick * max_angle` (khởi điểm 25°), `yaw_rate_sp = stick * max_yaw_rate` (khởi điểm 120°/s).

**Độ cao** (100 Hz; tham khảo Betaflight `autopilot_multirotor.c altitudeControl`, ArduPilot `AC_PosControl`):
```
vz_sp  = clamp(Kp_z * (z_sp − z), ±vz_max)          // vz_max khởi điểm 0,5 m/s
e_v    = vz_sp − vz
I_v   += Ki_v * e_v * dt  (giới hạn)
thr    = (hover_thr + Kp_v*e_v + I_v) / max(cosφ·cosθ, 0.5)   // bù nghiêng như Betaflight
thr    = clamp(thr, thr_min, thr_max)
```
- **Khi vào Alt Hold:** `z_sp = z` hiện tại và `hover_thr` = ga hiện tại (Betaflight cũng lấy ga tại thời điểm vào chế độ). Như vậy chuyển chế độ không bị giật.
- **Cần ga trong Alt Hold:** vùng chết 40–60% giữ độ cao; ngoài vùng đó cần ga điều khiển tốc độ leo/hạ (`z_sp += vz_cmd*dt`), giống ArduPilot.

### 7.6 Mixer quad X

Dùng thứ tự motor giống PX4 và ArduPilot (cả hai đã kiểm tra trong mã nguồn đều dùng thứ tự này):

| Motor | Vị trí | Chiều quay (nhìn từ trên) | Throttle | Roll | Pitch | Yaw |
|---|---|---|---|---|---|---|
| M1 | Trước phải | CCW | +1 | −1 | +1 | +1 |
| M2 | Sau trái | CCW | +1 | +1 | −1 | +1 |
| M3 | Trước trái | CW | +1 | +1 | +1 | −1 |
| M4 | Sau phải | CW | +1 | −1 | −1 | −1 |

(Roll dương: tăng hai motor bên trái để cánh phải hạ. Pitch dương: tăng hai motor trước để mũi ngẩng. Yaw dương: tăng motor CCW, mô-men phản lực làm thân quay theo chiều kim đồng hồ.)

- **Chống bão hòa:** sau khi trộn, nếu motor lớn nhất > 1 thì trừ phần vượt cho tất cả; nếu motor nhỏ nhất < `motor_idle` thì cộng phần thiếu. Nếu khoảng chênh vẫn vượt quá [`motor_idle`, 1], thu nhỏ phần mô-men, **ưu tiên giữ roll/pitch hơn ga**. Báo cờ bão hòa ngược về vòng rate để chống tích phân (anti-windup).
- **Phải VERIFY bằng thử nghiệm khi tháo cánh quạt** (mốc M5): nghiêng tay thì đúng motor tăng tốc; đặt yaw thì đúng cặp motor tăng tốc.

### 7.7 Arm, disarm, failsafe

**Điều kiện arm** (tất cả phải đúng): công tắc ARM chuyển từ tắt sang bật; ga < 5%; RC hợp lệ; đã hiệu chuẩn xong; |roll|, |pitch| < 15°; không ở trạng thái failsafe; vòng điều khiển chạy đúng nhịp.

**Disarm khi:** tắt công tắc ARM; failsafe giai đoạn 2; phát hiện lật (|roll| hoặc |pitch| > 70° liên tục 0,3 s); hoặc lệnh `DISARM` từ GUI. **Không có lệnh ARM từ GUI.**

**Failsafe mất sóng tay cầm** (tham khảo Betaflight: giai đoạn 1 mặc định 1,5 s, sau đó "drop"; cần 0,5 s tín hiệu tốt để hồi phục):
- **Giai đoạn 1** (RC mất > 0,2 s): đặt roll và pitch về 0 (giữ thăng bằng), yaw rate = 0. Ga: nếu Alt Hold hợp lệ thì giữ độ cao, nếu không thì giữ ga cuối.
- **Giai đoạn 2** (sau 1,0 s): nếu ước lượng độ cao hợp lệ thì hạ cánh có kiểm soát ở 0,3 m/s và disarm khi ToF < 0,10 m; nếu không thì **disarm**.
- Sau khi failsafe đã disarm: phải tắt rồi bật lại công tắc ARM mới arm lại được.

**Khi khởi động hoặc sau khi IWDG reset:** luôn ở trạng thái **DISARMED**.

---

## 8. Tham số

- Bảng tham số có tên, kiểu float, giá trị mặc định, min và max. Ví dụ: `rate_p_rp, rate_i_rp, rate_d_rp, rate_ilim_rp, rate_p_y, rate_i_y, ang_p_rp, max_angle, max_rate_rp, max_yaw_rate, gyro_lpf_hz, dterm_lpf_hz, acc_lpf_hz, kf_sg, kf_sacc, kf_smag, kf_sa_z, kf_stof, alt_p, vz_p, vz_i, vz_max, thr_min, thr_max, motor_idle, mag_off_x/y/z, imu_axis_map, rc_map...`
- **Lưu vào flash sector 11** bằng lệnh `SAVE`, **chỉ khi đã disarm**. F405 chỉ có một bank flash, nên khi xóa sector CPU bị dừng khoảng 1–2 s (tra datasheet F405). Có số "magic", số phiên bản và CRC32. Nếu dữ liệu sai thì nạp giá trị mặc định.
- **Giá trị PID khởi điểm:** để nhỏ (P nhỏ, I = D = 0), và chỉnh theo quy trình ở mốc M7. Không bịa bộ hệ số "đẹp".

---

## 9. Telemetry và GUI Python

### 9.1 Giao thức (ASCII, mỗi dòng kết thúc bằng `\n`; không phụ thuộc lớp truyền: USB CDC hiện tại, UART/HC-05 sau này)

**Drone gửi lên PC:**
- `T,<t_ms>,<arm>,<mode>,<fs>,<roll>,<pitch>,<yaw>,<p>,<q>,<r>,<roll_sp>,<pitch_sp>,<yawr_sp>,<z>,<vz>,<z_sp>,<tof>,<thr>,<m1>,<m2>,<m3>,<m4>,<ctrl_us_max>,<jitter_us_max>,<h_terrain>,<baro_rel>,<alt_src>`
  (góc tính bằng độ, độ dài bằng m). Gửi ở **50 Hz**, khoảng 6 KB/s. USB Full-speed dư rất nhiều băng thông, nên có thể tăng lên 100 Hz khi chỉnh PID. Khi chuyển sang HC-05 (115200 baud, khoảng 11,5 KB/s), giữ 50 Hz.
- `P,<name>,<value>` (trả lời lệnh GET/LIST), `OK,<cmd>`, `ERR,<cmd>,<reason>`, `MSG,<text>`.

**PC gửi xuống drone:**
- `GET <name>`, `SET <name> <value>` (có kiểm tra min/max), `LIST`, `SAVE` (chỉ khi disarm), `DISARM`, `CALIB_MAG_START` / `CALIB_MAG_STOP` (chỉ khi disarm), `RAW <0|1>` (bật luồng dữ liệu thô của cảm biến để phân tích offline), `DUMP` (chỉ khi disarm: gửi hộp đen RAM dưới dạng các dòng `L,...` rồi `OK,DUMP`).

### 9.2 Lớp truyền USB CDC (`bsp/usb_cdc_port.c`, `app/telem_transport.c`)

- `telem_transport` có giao diện chung `tx(buf, len)` và `rx_available()` / `rx_read()`. Hiện cài bằng USB CDC; sau này thêm bản UART cho HC-05 mà không phải sửa `telem_proto`.
- **Nhận:** `CDC_Receive_FS` chạy **trong ngắt OTG_FS**. Trong đó chỉ chép dữ liệu vào ring buffer, gọi `osSignalSet(telemTaskHandle, SIG_USB_RX)`, rồi gọi lại `USBD_CDC_ReceivePacket`. Không phân tích lệnh trong ngắt.
- **Gửi:** chỉ `task_telem` được gọi `CDC_Transmit_FS`. Nếu hàm trả về `USBD_BUSY` thì giữ dữ liệu cho chu kỳ sau, **không vòng lặp chờ**. Nếu `hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED` (chưa cắm cáp hoặc PC chưa mở cổng) thì bỏ qua việc gửi.
- Mỗi gói USB tối đa 64 byte; `CDC_Transmit_FS` nhận bộ đệm dài hơn và tự chia gói. **Bộ đệm phải còn tồn tại cho đến khi gửi xong**, nên dùng bộ đệm tĩnh, không dùng biến cục bộ.
- Mất kết nối USB **không** kích hoạt failsafe bay. Lệnh `DISARM` từ GUI vẫn có hiệu lực khi đang kết nối.
- `printf` để gỡ lỗi: chuyển qua cùng ring buffer TX, chỉ dùng trong `task_telem` hoặc lúc khởi động. Không gọi trong ISR hay `task_ctrl`.

### 9.3 GUI (`gcs/`)

- Python 3, `pyserial`, `PySide6` hoặc `PyQt5`, `pyqtgraph`.
- Một luồng đọc serial đưa dữ liệu vào queue; GUI cập nhật bằng timer khoảng 30 Hz.
- Kết nối bằng cổng COM ảo của USB CDC (Windows thấy là "USB Serial Device"; tốc độ baud khai báo không có ý nghĩa với CDC). Sau này đổi sang cổng COM Bluetooth của HC-05 mà không cần sửa GUI.
- **Chức năng:** chọn cổng COM và kết nối; vẽ đồ thị realtime (cho chọn kênh; có nhóm sẵn "Rate roll: sp vs đo", "Góc", "Độ cao: z, z_sp, tof", "Motor"); bảng tham số (LIST/GET/SET/SAVE); **nút DISARM khẩn cấp lớn, màu đỏ**; nút **DUMP** để tải hộp đen và lưu thành CSV; ghi toàn bộ dòng nhận được ra CSV có timestamp; hiển thị trạng thái arm/mode/failsafe và thời gian vòng lặp.
- `tools/replay_kf.py`: đọc log RAW, chạy Kalman 1D và 2D **cùng công thức với firmware**, vẽ so sánh. Dùng để chỉnh σ offline.

---

## 10. An toàn (bắt buộc tuân thủ)

1. **Tháo cánh quạt** mỗi khi nạp code có thay đổi ở ESC, mixer, arm hoặc failsafe, và cho mọi lần thử motor đầu tiên.
2. Bay thử lần đầu trên **giá thử** hoặc **buộc dây**, trong không gian rộng, người đứng xa.
3. Kiểm tra failsafe (tắt tay cầm thì motor về idle hoặc dừng theo đúng mục 7.7) **trước** chuyến bay đầu tiên.
4. Kiểm tra IWDG: cố tình treo `task_ctrl` (bản build test) thì chip phải reset và khởi động lại ở trạng thái DISARMED.
5. Pin: tối đa 3S cho đến khi xác nhận ESC chịu được mức cao hơn.
6. **Cáp USB:** chỉ cắm khi drone ở trên **giá thử** hoặc đứng yên dưới đất. **Không bay tự do khi đang cắm cáp**: cáp kéo làm lệch tư thế và có thể giật đứt cổng. Trước khi cắm USB lúc pin đang nối, kiểm tra mạch có diode chống dòng ngược giữa VBUS và nguồn 5 V của mạch (`TBD`); nếu không chắc thì **rút pin trước khi cắm USB**, hoặc ngược lại.

---

## 11. Kế hoạch 3 ngày và các mốc

Ký hiệu: **[NGƯỜI DÙNG]** là việc phần cứng hoặc thao tác tay, **[CC]** là việc của Claude Code.

### Ngày 1: Hệ thống chạy được, chưa lắp cánh quạt

| Mốc | Việc | Tiêu chí hoàn thành |
|---|---|---|
| M0 | [NGƯỜI DÙNG] Cung cấp các mục `TBD` (mục 13), tạo `.ioc` theo mục 5. [CC] **Kiểm kê code cũ (mục 6.0, bước 1)** để lấy chân và thông số đã chạy được; kiểm tra lại `.ioc` cho khớp; dựng cấu trúc thư mục và build | Build thành công; LED nháy; PC nhận ra cổng COM ảo USB CDC; `printf` hiện trên terminal |
| M1 | [CC] Khung FreeRTOS 3 task (stub), DWT micros, GPIO debug, IWDG | Đo trên oscilloscope/logic analyzer: `task_slow` 100 Hz, `task_telem` 50 Hz; không tràn stack |
| M2 | [CC] Driver BMI323 (mục 6.1): SPI DMA, ngắt INT1 | CHIP_ID = 0x43, ERR_REG = 0; đếm được **3200 ± 2% lần ngắt mỗi giây**; đứng yên thì gyro ≈ 0, \|a\| ≈ 9,81 |
| M3 | [CC] Telemetry + GUI v0 + bộ lọc + hiệu chuẩn khi khởi động + **Kalman 1D roll/pitch**. [NGƯỜI DÙNG] Thử nghiêng | Nghiêng phải thì roll dương, ngẩng mũi thì pitch dương; sai số < 2° so với thước đo góc; đứng yên 5 phút trôi < 1° |
| M4 | [CC] **Đọc, đánh giá và chuyển code cũ** của tay cầm và ESC (mục 6.0, cả 4 bước; báo cáo bảng đánh giá cho người dùng trước khi viết lại bất kỳ module nào); viết mới arm/disarm, failsafe | GUI hiển thị đúng stick, giống code cũ; motor phản hồi giống code cũ (tháo cánh quạt); arm/disarm đúng điều kiện ở mục 7.7; tắt tay cầm thì failsafe kích hoạt |
| M5 | [CC] Mixer + vòng rate và góc với P nhỏ. [NGƯỜI DÙNG] **Tháo cánh**, thử hướng motor | Nghiêng tay sang phải thì M1 và M4 (bên phải) **tăng**; chiều quay khớp bảng 7.6; failsafe làm motor về idle hoặc dừng |

### Ngày 2: Bay ổn định, đồng thời chuẩn bị cho độ cao

| Mốc | Việc | Tiêu chí hoàn thành |
|---|---|---|
| M6 | [CC] Tham số + SET/GET/SAVE + lưu flash; GUI có bảng tham số và nút DISARM. **Hộp đen trong RAM**: ghi các trường telemetry chính vào ring buffer ở RAM CCM (64 KB) ở 50 Hz, giữ khoảng 20 s gần nhất, bắt đầu khi arm; lệnh `DUMP` (chỉ khi disarm) gửi toàn bộ qua USB để GUI lưu thành CSV | Đổi hệ số trong lúc chạy; lưu và khởi động lại vẫn giữ giá trị; bay xong, cắm USB, `DUMP` ra được log của chuyến bay |
| M7 | [NGƯỜI DÙNG] Chỉnh PID **trên giá thử, có cắm USB** để xem đồ thị realtime: rate P (tăng đến khi bắt đầu dao động rồi giảm 30–50%), sau đó D, sau đó I; tiếp theo vòng góc. Sau đó **rút USB** và bay tự do. [CC] Phân tích log (realtime khi trên giá thử, hộp đen khi bay tự do), đề xuất hệ số | Đáp ứng bậc rate không dao động kéo dài; **hover tay ổn định 30 s**; ghi lại `hover_thr` |
| M8 | [CC] Driver la bàn và **baro** (sau khi quét I2C và xác nhận chip), driver MTF-01P (MSP) | Đọc ToF đúng so với thước dây (sai < 3 cm trong khoảng 0,2–2 m); vector từ trường thay đổi khi xoay drone; nâng drone 1 m thì baro tăng khoảng 1 m (sai cỡ ±20 cm là bình thường) |
| M9 | [CC] **Kalman 2D độ cao + phát hiện bước nhảy (mục 7.4b) ở chế độ chỉ quan sát**, kèm `replay_kf.py` | Bay tay: z bám ToF, vz hợp lý (không lệch dần); che ToF thì `valid` chuyển sang false đúng thời điểm; **cầm drone đi ngang qua mặt bàn**: `h_terrain` nhảy đúng bằng chiều cao bàn, z gần như không đổi |

### Ngày 3: Giữ độ cao, sau đó yaw bằng la bàn

| Mốc | Việc | Tiêu chí hoàn thành |
|---|---|---|
| M10 | [CC] Chế độ Alt Hold (mục 7.4, 7.4b phần bước nhảy, 7.5) + failsafe hạ cánh | Ở độ cao 0,5–1 m, **giữ ±10 cm trong 30 s**; chuyển chế độ không giật; mất tay cầm thì hạ cánh có kiểm soát; bay ngang qua bàn thì **độ cao so với sàn không đổi quá ±10 cm** |
| M10b | [CC] Kiểm tra nhất quán dài hạn bằng baro + dự phòng khi mất ToF (mục 7.4b) | Che ToF trong 2 s khi đang giữ độ cao: độ cao giữ trong ±30 cm, GUI báo `alt_src = 1`; bỏ che thì quay về ToF không giật |
| M11 | [CC] Hiệu chuẩn hard-iron + Kalman 1D yaw với la bàn | Xoay 90° thì yaw ≈ 90° (sai < 10°); yaw không trôi trong 2 phút |
| M12 | [CC] Dọn code, cập nhật `PROGRESS.md`, ghi số đo: thời gian vòng lặp, jitter, sai số độ cao | Có bảng số liệu để đưa vào báo cáo |

### Thứ tự cắt bớt nếu trễ tiến độ

1. Bỏ yaw dùng la bàn (M11), yaw chỉ điều khiển bằng tốc độ góc.
2. Bỏ phần baro (M10b): mất ToF quá 0,5 s thì thoát Alt Hold như mục 7.4. **Giữ** phát hiện bước nhảy (không cần baro).
3. Bỏ failsafe hạ cánh tự động; giai đoạn 2 chuyển thành disarm.
4. GUI rút gọn: chỉ đồ thị và bảng tham số.
4. **Không cắt:** failsafe cơ bản, điều kiện arm, IWDG, thử nghiệm khi tháo cánh quạt.

---

## 12. Kiểm thử

- **`tests/` trên PC (gcc):**
  - Kalman 1D hội tụ khi phép đo là hằng số, và K ổn định đúng như giá trị tính tay.
  - Kalman 2D với dữ liệu giả lập (vận tốc hằng + nhiễu) bám đúng z, vz.
  - PID: chống tích phân khi bão hòa.
  - Mixer: dấu đúng theo bảng 7.6, xử lý bão hòa đúng.
  - Phát hiện bước nhảy: dữ liệu ToF giả lập bay qua bàn 0,7 m (có nhiễu và một gai nhiễu đơn lẻ) thì `h_terrain` = 0,7 ± 0,03 m, z không nhảy, gai đơn lẻ **không** bị coi là bước nhảy.
  - Bộ lọc: đáp ứng biên độ tại tần số cắt ≈ −3 dB.
- **Trên MCU:** đo bằng GPIO và logic analyzer gồm độ trễ từ ngắt IMU đến lúc xuất ESC, thời gian chạy lớn nhất của `task_ctrl`, và jitter. Gửi các số này qua telemetry.
- **Offline:** `replay_kf.py` trên log RAW để so firmware với Python (sai lệch cỡ sai số làm tròn float).

---

## 13. Thông tin cần người dùng cung cấp (`TBD`). Hỏi trước bước cần đến

| # | Thông tin | Cần cho mốc |
|---|---|---|
| 1 | Mạch F405: mạch tự làm hay mạch mua sẵn (tên gì)? **Sơ đồ chân** (SPI + CS + INT1 của IMU, các UART, I2C, 4 chân ESC, LED), **có thạch anh HSE không và tần số bao nhiêu** (USB bắt buộc cần HSE) | M0 |
| 1b | Cổng USB-C: D−/D+ có nối vào PA11/PA12 không; VBUS có nối PA9 không (để bật hoặc tắt VBUS sensing); có diode giữa VBUS và nguồn 5 V của mạch không | M0 |
| 2 | Công cụ build và nạp: STM32CubeIDE hay CMake + arm-none-eabi-gcc; ST-Link hay công cụ khác | M0 |
| 3 | Hướng gắn BMI323 so với khung (để lập ma trận đổi trục) | M3 |
| 0 | Chép code cũ vào **`legacy/`** (đã đọc được tay cầm, đã hiệu chuẩn ESC, điều khiển được motor). Phần lớn mục 1, 4 và 5 có thể lấy từ code này. Nếu `legacy/` trống hoặc thiếu file, hỏi người dùng | M0 |
| 4 | Giao thức tay cầm (PPM / SBUS / CRSF / iBUS), thứ tự kênh, công tắc nào là ARM và MODE (lấy từ code cũ nếu có) | M4 |
| 5 | ESC: firmware gì, giao thức hỗ trợ; tần số PWM và dải xung đang dùng (lấy từ code cũ); pin bao nhiêu cell | M4 |
| 6 | Motor đã lắp theo thứ tự và chiều quay nào (đối chiếu bảng 7.6) | M5 |
| 7 | Đã đặt MTF-01P sang **MSP** bằng MicoAssistant chưa | M8 |
| 8 | Kết quả quét I2C: chip la bàn và chip baro thật trên GEP-M10-DI là gì | M8 |
| 9 | (Tùy chọn) Khối lượng drone, motor, cánh quạt | M7 |

---

## 14. Tài liệu tham khảo (đã đọc mã nguồn khi lập kế hoạch)

| Nguồn | Tệp | Học được gì |
|---|---|---|
| BMI323 datasheet BST-BMI323-DS000-13 | §5.6, bảng thanh ghi, §8 (SPI) | ODR, bộ lọc trong chip, thanh ghi, trình tự khởi tạo SPI |
| Carbon Aeronautics (tài liệu của người dùng) | Chương Kalman 1D, Kalman 2D độ cao | Công thức Kalman (đã ghi chú 2 lỗi in ở mục 7.3) |
| Betaflight (GPLv3, chỉ tham khảo) | `src/main/drivers/rangefinder/rangefinder_lidarmt.c` | Gói MSP của MTF-01/01P, tầm 1200 cm, độ trễ flow khoảng 65 ms |
| | `src/main/msp/msp_protocol_v2_common.h`, `msp_serial.c` | ID 0x1F01/0x1F02, khung MSPv2, CRC8 DVB-S2 |
| | `src/main/sensors/rangefinder.c` | Nhân cos(nghiêng), loại bỏ khi nghiêng quá giới hạn |
| | `src/main/flight/imu.c` | Mahony, kiểm tra độ lớn gia tốc 0,9–1,1 g |
| | `src/main/flight/autopilot_multirotor.c` (`altitudeControl`) | PID độ cao, ga hover lấy lúc vào chế độ, bù nghiêng 1/max(cos, 0,5) |
| | `src/main/flight/position_estimator.c` | Phương án tham khảo: Kalman 3 trạng thái, gia tốc là phép đo, rangefinder R = 100 cm² |
| | `src/main/flight/pid.c`, `mixer_init.c`, `failsafe.c`, `sensors/gyro.h`, `flight/pid.h` | D theo giá trị đo, I relax, reset I khi ga thấp, failsafe 2 giai đoạn, mặc định bộ lọc |
| PX4 (BSD-3) | `src/lib/rate_control/rate_control.cpp` | Rate PID + feedforward, chống tích phân theo cờ bão hòa, giới hạn I |
| | `src/modules/ekf2/EKF/aid_sources/range_finder/range_finder_consistency_check.cpp`, `terrain_control.cpp` | So tốc độ đổi của rangefinder với vz ước lượng từ IMU để phát hiện địa hình thay đổi; ước lượng độ cao địa hình riêng |
| | `src/modules/mc_rate_control/MulticopterRateControl.cpp` | Vòng rate chạy trên work queue `rate_ctrl`, kích bởi dữ liệu gyro mới (đường nóng một chặng) |
| | `ROMFS/.../airframes/4001_quad_x` | Hình học quad X (M1 trước phải CCW, M2 sau trái CCW, M3 trước trái CW, M4 sau phải CW) |
| ArduPilot (GPLv3, chỉ tham khảo) | `libraries/AP_Motors/AP_MotorsMatrix.cpp` | Thứ tự quad X giống PX4 |
| | `libraries/AP_MSP/msp_sensors.h`, `AP_RangeFinder_MSP.cpp` | Xác nhận lại cấu trúc gói MSP rangefinder và flow |
| | `libraries/AC_AttitudeControl/AC_PosControl.cpp` | Phân tầng độ cao: P vị trí → vận tốc → gia tốc |
| MicoAir | github.com/micoair/MTF-01_USER_MANUAL | Giao thức hỗ trợ (Micolink/MSP/MAVLink), đổi giao thức bằng MicoAssistant |

**Lưu ý:** cả Betaflight, PX4 và ArduPilot hiện **chưa có driver BMI323**, nên driver phải viết theo datasheet.
