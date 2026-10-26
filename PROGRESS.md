# PROGRESS

## Trạng thái mốc

| Mốc | Trạng thái | Ghi chú |
|---|---|---|
| M0 | **Xong** (2026-10-13): người dùng nạp qua CubeIDE + ST-Link, luồng USB CDC chạy | |
| M1 | **Xong** (2026-10-13) | ctrl 1000 Hz, exec_max < 1 µs, jitter_max 23 µs; slow 100 Hz; telem 50 Hz; stack trống ctrl 968 / slow 974 / telem 852 word (trên 1024). Chưa thử IWDG (`APP_TEST_IWDG_HANG`) |
| M2 | **Xong** (2026-10-13) | init 0, CHIP_ID 0x1143, ERR 0, ACC/GYR_CONF đọc lại đúng; **3179 Hz** (−0,66 %), ovr 0, spierr 0; gyro đứng yên 1–5 mrad/s; \|a\| 9,71–9,74 m/s² (−1 %, chưa hiệu chuẩn); ctrl 1060 Hz, exec_max 2 µs, jitter_max 44 µs; stack trống ≥ 787 w. Đặt nằm ngang: acc_z ≈ +9,7 (trục z cảm biến hướng lên). Ngắt INT1 hoạt động ⇒ không cần FIFO |
| M3 | **Xong** (2026-10-13, người dùng xác nhận) | Số đo thử nghiêng/trôi: chờ người dùng gửi để ghi lại |
| M4 | **Xong** (2026-10-15, người dùng xác nhận) | SBUS DMA+IDLE 50 khung/s, 0 lỗi; failsafe bộ thu nhận qua CH3 < 100; arm chỉ bằng CH5 + ga thấp nhất; disarm (công tắc, cần, GUI, failsafe 2, lật, IMU, idle 15 s, watchdog ESC) đều thử đạt; ESC 400 Hz, 900–2000 µs, idle 1000 µs |
| M5 | **Xong** (2026-10-15, người dùng xác nhận) | Bảng motor M1=ESC2, M2=ESC3, M3=ESC4, M4=ESC1, chiều quay khớp §7.6; nghiêng/ngẩng bằng tay ⇒ đúng motor tăng; yaw: xoay phải ~100°/s ⇒ M3/M4 +114 µs mỗi motor (từ log). Yaw chỉ điều khiển tốc độ, không giữ hướng (giữ theo plan, người dùng chọn) |
| M6 | **Đang làm**: code + GUI xong (0 warning, test PC đạt), chờ nạp và thử | Xem "M6" bên dưới |
| M7–M12 | Chưa bắt đầu | |

---

## M6

### Đã làm (2026-10-16)
- GUI (theo yêu cầu người dùng): đường giá trị đặt nét đứt `roll_sp`, `pitch_sp` trên đồ thị góc; đồ thị mới tốc độ yaw `r` đo vs `yawr_sp`.
- `app/params.c` (C thuần, test PC): 17 tham số có tên/mặc định/min/max: `rate_p_rp, rate_i_rp, rate_d_rp, rate_p_y, rate_i_y, rate_ilim, ang_p_rp, max_angle (°), max_rate_rp (°/s), max_yaw_rate (°/s), gyro_lpf_hz, acc_lpf_hz, dterm_lpf_hz, kf_sg (°/s), kf_sacc (°), motor_idle_us, rc_deadband`. Mặc định = giá trị M5. 3 tham số bộ lọc chỉ đổi được khi disarm. `imu_axis_map`, `rc_map` (ví dụ ở §8) để hằng số: đổi lúc chạy dễ gây nhầm hướng.
- Đồng bộ (§3.4): `task_telem` là nơi ghi duy nhất (`g_params` rồi `g_params_seq`); `task_ctrl` chép bộ tham số ở đầu chu kỳ khi seq đổi; `task_slow` chép cho `rc_deadband`. Bộ lọc gyro/accel khởi tạo lại trong vùng găng (ISR IMU bị chặn).
- Ảnh flash: header {magic "DPRM", version, count, CRC32} + {FNV-1a(tên), giá trị}; nạp theo tên ⇒ thêm/bớt tham số không mất giá trị cũ; giá trị ngoài khoảng bỏ qua; ảnh hỏng ⇒ dùng toàn bộ mặc định. `bsp/flash_params.c`: xóa sector 11 (0x080E0000) + ghi word + đọc lại so sánh. `// VERIFY:` địa chỉ sector (RM0090) và thời gian xóa (DS8626).
- Lệnh: `GET <tên>`, `SET <tên> <giá trị>` (kiểm khoảng, kiểm disarm cho bộ lọc), `LIST`, `SAVE` (chỉ khi disarm và không có motor test), `DUMP`. Trả `P,<tên>,<giá trị>,<min>,<max>` (thêm min/max so với §9.1). Số thực đọc/in bằng `lib/numfmt.c` (không dùng `strtof`/`%f`).
- Hộp đen (`app/blackbox.c`): bản ghi 36 byte ở 50 Hz trong `task_telem`, 1500 bản ghi = 30 s, trong CCM (section NOLOAD `.ccm_bss` thêm vào linker ⇒ không chiếm flash). Arm ⇒ bắt đầu bản mới; ghi tiếp 2 s sau disarm. `DUMP` (khi disarm) gửi dòng `L,...` (số nguyên: cdeg, 0,1°/s, µs), tạm dừng dòng T. Cổng USB: ring TX 8 KB, mỗi lần gửi tối đa 2 KB.
- GUI tab "Tham số / Hộp đen": bảng LIST/SET (ô đã sửa nhưng chưa gửi tô vàng), SAVE (hỏi xác nhận), DUMP ⇒ `gcs/logs/blackbox_<thời gian>.csv`.
- Test PC thêm: CRC32 (giá trị chuẩn 0xCBF43926), đọc/in số, bảng tham số (khoảng, NaN, round trip ảnh, flash trống, CRC hỏng, ảnh cụt, giá trị ngoài khoảng), hộp đen (vòng tròn, định dạng = header). Đạt.
- Build 0 warning; FLASH 62,0 KB; RAM 53,3 KB; CCM 54,0 KB.
- **Phát hiện mất kiểm soát** (người dùng chọn, 2026-10-16; ý tưởng từ ArduPilot `crash_check`, code tự viết): khi motor đang SPOOL_UP/RUN mà max(|roll − roll_sp|, |pitch − pitch_sp|) > `lc_angle` (mặc định 30°) liên tục `lc_time_ms` (mặc định 1000 ms) ⇒ disarm `lost control`, phải gạt lại CH5. Không ngắt cứng ở 25° (người dùng hỏi; gió/vọt lố ngắn sẽ làm rơi). Phát hiện lật 70°/0,3 s giữ nguyên. Tham số mới: `lc_angle`, `lc_time_ms` (tổng 19). Test PC: không ngắt ở idle, không ngắt với 2 lần lệch 0,6 s, ngắt sau > 1 s, chốt gạt công tắc. Build 0 warning.
- **ESC về 50 Hz** (người dùng yêu cầu, 2026-10-16): `ESC_PWM_HZ` = 50. Dải 900–2000 µs, idle 1000 µs giữ nguyên. Lệch §5 (400 Hz).
- Log GCS 2026-10-16: người dùng `SET rate_p_rp 0.2`, `ang_p_rp 7` nhưng không `SAVE` ⇒ mỗi lần khởi động lại về mặc định. Chuyến bay tự do (lên tối đa ~1,7 m) không có trong log GCS (không cắm USB) ⇒ chưa xác định nguyên nhân; firmware không giới hạn ga (ga 1 ⇒ 2000 µs).
- Người dùng cho biết ESC là loại **40 A** (PROJECT_PLAN §2 ghi 30 A 2S–3S); chưa rõ hãng/firmware.
- GUI: tự kết nối lại cổng COM (10 lần, mỗi 1 s) khi gặp `serial error`, vì lúc SAVE xóa flash MCU dừng ~1–2 s làm Windows báo lỗi cổng (tham số vẫn lưu thành công: `OK,SAVE` đến trước lỗi).
- **Chốt ESC 50 Hz** (người dùng, 2026-10-16): ở 400 Hz drone không cất cánh, ở 50 Hz cất cánh được nhưng lắc. Lưu ý gây nhiễu kết luận: lần thử 400 Hz không cất cánh trùng lúc `rate_p_rp` = 0,4 và `rate_d_rp` = 0,007 đã được SAVE. Có thể ESC (30 A, 2S–3S) chỉ nhận PWM 50 Hz kiểu servo.
- Sự cố nạp 2026-10-16: CLI báo `Data mismatch at 0x0800B7F0 ... verification failed`, lần sau `Unable to get core ID` ⇒ flash có thể đang dở dang; đã nhắc người dùng không cắm pin khi có cánh cho tới khi nạp lại và verify OK.
- Trim tay cầm lệch (CH1 913, CH2 1051, CH4 884; ga thấp nhất 8 < 100 ⇒ firmware báo failsafe dù còn sóng). Đề xuất đang chờ duyệt: (1) nhận failsafe bằng "dấu vân tay" CH3 25–55 + CH1/2/4/5/6 = 1000 ± 3; (2) chỉ arm khi |roll|,|pitch|,|yaw| stick < 0,05.
- Người dùng cho biết **không có giá thử/dây buộc, sẽ bay tự do** (lệch §10.2 và quy trình M7 "chỉnh trên giá thử"). Đã khuyến nghị dây buộc tự chế + quy trình an toàn.
- Lưu ý: nạp bằng CubeIDE/CLI chỉ xóa các sector chứa chương trình (0–2) ⇒ tham số đã lưu ở sector 11 giữ nguyên; "full chip erase" sẽ xóa chúng.

---

## M5

### Bước 1: thử từng ESC để lập bảng motor (2026-10-15)
- Lệnh `MOTOR_TEST <1..4> CONFIRM` (giống "motor test" của ArduPilot): chỉ khi DISARMED; một kênh TIM3 ở 1120 µs (trên ngưỡng khởi động ~1070 µs) trong 2 s; arm hoặc lệnh DISARM hủy ngay; vẫn đi qua `pwm_esc_write_us` nên watchdog đầu ra ESC vẫn bảo vệ. GUI: nút ESC1–ESC4, hộp thoại "đã tháo cánh?" (hỏi 1 lần mỗi phiên).
- Build 0 warning, test PC đạt, nạp verify OK.
- Kết quả thử (người dùng): ESC1 sau-phải CW, ESC2 trước-phải CCW, ESC3 sau-trái CCW, ESC4 trước-trái CW ⇒ **M1 = ESC2, M2 = ESC3, M3 = ESC4, M4 = ESC1**, chiều quay cả 4 khớp §7.6 (không cần đảo dây). `CFG_MOTOR_TO_TIM_CH = {1, 2, 3, 0}`. Khớp với suy luận từ mixer code cũ.

### Bước 2: mixer + vòng rate + vòng góc (2026-10-15, code xong, chờ nạp + thử khi tháo cánh)
- `lib/mixer.c`: quad X theo bảng §7.6. Chống bão hòa: (1) roll/pitch quá khoảng [idle, 1] thì thu nhỏ cùng tỉ lệ; (2) yaw lấy phần còn lại (ưu tiên thấp nhất, chia đôi tìm hệ số); (3) dịch ga (không dịch mô-men) để mọi motor trong [idle, 1]. Trả cờ bão hòa theo hướng cho anti-windup.
- `lib/pid.c` (`pid_ctrl_t`): P + I (giới hạn, không tích phân thêm theo hướng mixer đang bão hòa, như PX4) + D theo giá trị đo qua PT1 80 Hz + feed-forward.
- `task_ctrl`: tốc độ góc thân = gyro đã lọc − bias, mỗi chu kỳ (~1060 Hz). Vòng góc: `angle_sp = stick × 25°`, `rate_sp = 4 × (angle_sp − angle)` giới hạn ±200°/s; yaw: `rate_sp = stick × 120°/s`. Rate PID → mixer → ESC (bảng M → kênh TIM3). Chỉ điều khiển khi ARMED và không ở IDLE; ở IDLE/disarm: motor idle/dừng, reset I. Chu kỳ không có mẫu IMU mới: giữ lệnh motor cũ.
- Hệ số khởi điểm (§8: P nhỏ, I = D = 0): góc P 4; rate P roll/pitch/yaw 0,05; I = D = 0. Ví dụ nghiêng 10° ⇒ chênh ~38 µs mỗi motor.
- Telemetry: `roll_sp`, `pitch_sp`, `yawr_sp`, `m1..m4` (thứ tự M1..M4). GUI ghi vị trí từng motor.
- Test PC: dấu mixer theo §7.6, tổng lực không đổi khi chỉ có mô-men, giữ roll khi ga tối đa/tối thiểu, thu nhỏ + cờ bão hòa, yaw bị cắt trước roll; PID: P, I + giới hạn, anti-windup + xả ngược chiều, D không bị giật khi đổi setpoint, D trên dốc. Đột biến (đảo dấu roll, đảo dấu yaw, bỏ anti-windup) đều bị bắt. Build 0 warning; FLASH 55,3 KB.
- Kiểm tra dấu bằng tay: nghiêng phải ⇒ M1, M4 tăng; ngẩng mũi ⇒ M1, M3 giảm; xoay theo chiều kim đồng hồ ⇒ M3, M4 (CW) tăng.
- Kết quả thử (2026-10-15, đã tháo cánh, ga ~36 %): người dùng xác nhận các dòng roll/pitch đúng. Yaw (phân tích log `gcs/logs/log_20261015_232159.csv`): xoay phải 73–75° ở 97–151°/s ⇒ cặp CW cao hơn cặp CCW 114–164 µs mỗi motor; xoay trái 108° ⇒ −103 µs. Chỉ lệch trong lúc đang xoay (đúng thiết kế rate-only).
- Phát hiện: khi arm + đẩy ga, CH4 (yaw) ≈ 840 thay vì 1000 ⇒ yaw stick −0,175 ⇒ yawr_sp −21°/s (ngón cái lệch khi đẩy ga hoặc trim yaw lệch). Vùng chết 3 % không che được. Theo dõi lại ở M7.
- Người dùng chọn **giữ yaw theo plan** (không giữ hướng bằng gyro; giữ hướng có ở M11 với la bàn).
- Lưu ý cho M7: về IDLE (ga thấp nhất) là tắt điều khiển tư thế; khi bay thật cần thêm phát hiện "đã hạ cánh" trước khi tắt (ArduPilot/PX4 dùng land detector).

---

## M4

### Đánh giá code cũ tay cầm + ESC (§6.0 bước 2, 2026-10-13)

| # | Kết quả | Dẫn chứng (code cũ `drone_stm32f405rgt6/Core/Src/`) |
|---|---|---|
| R1 | **Không đạt** (một phần) | Có kiểm byte đầu 0x0F và byte cuối 0x00 (`rc_sbus.c:24,32`). Không kiểm lỗi parity/framing của USART: byte lỗi parity vẫn được ghép vào khung (SBUS không có CRC, nên parity là lớp kiểm tra duy nhất) |
| R2 | **Không đạt** | Ghép khung theo chỉ số, không dựa vào khoảng lặng giữa các khung ⇒ mất 1 byte là lệch khung cho tới khi tình cờ gặp 0x0F đúng chỗ. Không có `HAL_UART_ErrorCallback` (`main.c`) ⇒ lỗi overrun (ORE) làm HAL dừng nhận ngắt vĩnh viễn |
| R3 | **Không đạt (an toàn)** | Không có timeout, không đọc byte cờ 23 (frame lost bit 2, failsafe bit 3). Mất sóng ⇒ giữ giá trị cuối |
| R4 | **Không đạt** | Ngắt từng byte (`rc_sbus.c:21,81`): 100 kbaud, ~4000 ngắt/s, ưu tiên 0. `.ioc` mới đã có DMA circular cho USART1_RX |
| R5 | **Không đạt** | `rc_channel[]`, `*_pwm` ghi trong ISR, đọc ở vòng lặp chính từng biến, không `volatile`, không chép nguyên khung |
| R6 | **Đạt một phần** | Chuẩn hóa tuyến tính từng đoạn min/giữa/max + clamp (`rc_sbus.c:42–77`). Vùng chết nằm ở `flight_control.c:176–178` (theo độ), không nằm trong module tay cầm. Chỉ giải mã CH1–CH4 ⇒ chưa có kênh ARM/MODE |
| E1 | **Đạt** | 50 Hz, dừng 900 µs, idle 950 µs, tối đa 2000 µs, clamp trên (`flight_control.c:283–300`) khớp dải đã hiệu chuẩn 900–2000 |
| E2 | **Đạt** | TIM3 PSC 83 → 1 µs/tick (`tim.c`) |
| E3 | **Đạt** | `HAL_TIM_PWM_ConfigChannel` bật OCxPE (đã kiểm `stm32f4xx_hal_tim.c`) |
| E4 | **Không đạt (an toàn)** | Không có arm: ga > 1050 là motor quay (`flight_control.c:271`). Khởi động với ga > 1800 ⇒ **tự vào hiệu chuẩn ESC, xuất 2000 µs** (`flight_control.c:96–102`) |
| E5 | **Không đạt** | Thứ tự motor cứng trong mixer (`flight_control.c:277–280`), suy ra M1 sau-phải, M2 trước-phải, M3 sau-trái, M4 trước-trái ≠ bảng 7.6. Chưa kiểm chứng |
| G1 | **Không đạt** | `HAL_Delay` trong `Motor_Init_And_Calibrate_By_RC` (chờ 4 s, vòng chờ ga) và vòng lặp chính `HAL_Delay(2)` |
| G2 | **Không đạt** | Mọi ngắt ưu tiên 0. (`.ioc` mới: USART1 = 7, DMA2_Stream2 = 6 → đã đúng) |
| G3 | Không áp dụng | Không giữ lại code tính toán cũ (`map()` dùng `long`) |
| G4 | **Không đạt** | Như R5 |

### Quyết định đề xuất (§6.0 bước 3), chờ người dùng duyệt

| Module | Quyết định | Lý do |
|---|---|---|
| Tay cầm SBUS | **Viết lại** | Không đạt R1–R5, trong đó R3 là tiêu chí an toàn; kiến trúc ngắt từng byte không hợp. Giữ nguyên: USART1 100000 8E2, thứ tự kênh AETR, bảng min/giữa/max từng kênh |
| ESC | **Giữ và sửa** | Giữ TIM3 1 MHz, 50 Hz (lần đầu), dừng 900 / idle 950 / tối đa 2000 µs, preload. Sửa: bỏ đường hiệu chuẩn tự động, chỉ xuất qua một hàm có clamp, disarm luôn 900 µs, bảng ánh xạ motor → kênh TIM3 |
| Arm/disarm, failsafe | Viết mới | Code cũ không có |

### Người dùng duyệt (2026-10-13)
- SBUS: **viết lại** (DMA + IDLE). ESC: **giữ và sửa**; bỏ hiệu chuẩn lúc bật nguồn, hiệu chuẩn sau này là lệnh GUI khi disarm + đã tháo cánh (§6.5).
- Arm/disarm: người dùng yêu cầu tham khảo ArduPilot (GPLv3: chỉ lấy ý tưởng, không chép code). Chọn **cả hai cách**: công tắc ARM (ưu tiên, khi tìm được kênh) + cần ga thấp nhất & yaw hết cỡ 3 s (dự phòng). Lệch §7.7 (plan chỉ có công tắc).

### M4 bước 1: driver SBUS + test tay cầm (2026-10-13, code xong, chờ nạp)
- `drivers/sbus.c` (C thuần): kiểm byte đầu 0x0F, byte cuối 0x00 hoặc SBUS2 (`& 0x0F == 0x04`), giải mã 16 kênh 11 bit, cờ frame lost (bit 2) và failsafe (bit 3) của byte 23. Test PC: giải mã khớp bộ đóng gói độc lập, cờ, từ chối byte đầu/cuối sai. Đạt.
- `bsp/rc_uart.c`: `HAL_UARTEx_ReceiveToIdle_DMA` trên USART1 (DMA circular 64 byte, tắt ngắt half-transfer). Mỗi ngắt IDLE: nếu đoạn từ lần IDLE trước dài đúng 25 byte và không có lỗi UART thì chép thành khung (ISR, ưu tiên 6–7). Lỗi parity/framing/overrun ⇒ bỏ khung; nếu HAL đã dừng nhận thì khởi động lại.
- `app/rc.c` (gọi trong `task_slow` ~100 Hz): chuẩn hóa theo min/giữa/max của code cũ (`config.h`), vùng chết 3 %, `valid = false` khi > 200 ms không có khung tốt hoặc bộ thu báo failsafe. Ghi `g_rc`.
- Telemetry thêm dòng `R,...` 10 Hz (lệch §9.1, chỉ dùng cho M4): trạng thái, khung/s, lỗi, 4 stick đã chuẩn hóa, 16 kênh thô. GUI có khung "Tay cầm": 16 thanh, kênh vừa đổi ≥ 40 đơn vị sáng vàng 1,5 s.
- Không đụng TIM3/ESC. Build 0 warning; FLASH 49,5 KB; RAM 47,1 KB.
- Cần kiểm chứng trên mạch: `HAL_UARTEx_ReceiveToIdle_DMA` ở chế độ circular tiếp tục nhận sau IDLE; lỗi parity có làm dừng DMA không (code xử lý được cả hai trường hợp).

### Kết quả test tay cầm MicroZone 6C-MINI (2026-10-13)
- Driver SBUS chạy: 50 khung/s, 0 lỗi khung, 0 lỗi UART.
- Hai công tắc góc trên **không phải kênh**: theo sách hướng dẫn là D/R (100 % ↔ 50 %) và CUSTOM (hành trình tùy chỉnh hoặc khóa kênh 2). Bảng min/max của code cũ (vd roll 627–1400) khớp với D/R 50 %.
- Đo lại ở D/R 100 %: CH1–CH4 min 200 / giữa 1000 / max 1800 → cập nhật `config.h`. **Luôn bay với D/R 100 %, không bật "khóa kênh 2".**
- CH5, CH6: công tắc 3 vị trí, thấp nhất 1800 / giữa 1000 / cao nhất 200.
- **Tắt tay cầm: bộ thu vẫn gửi khung, không bật cờ failsafe**; CH3 = 40 (~900 µs, đúng mặc định "runaway protection" trong sách), các kênh khác 1000. ⇒ thêm điều kiện mất sóng `CH3 < 120` (`CFG_RC_FS_THR_RAW`). Timeout 200 ms vẫn giữ cho trường hợp mất dây/bộ thu.
- Tay cầm đang bật **trộn kênh 1/2 (elevon)**: CH1 = a + e, CH2 = a − e (khớp số đo). Người dùng đã tắt MIX trên tay cầm.
- Sau khi tắt trộn: pitch và yaw đúng hướng; **roll ngược** (CH1 giảm khi sang phải) → `CFG_RC_ROLL_DIR = -1` trong firmware (không dùng REV trên tay cầm). Ga 168–1768; ngưỡng mất sóng hạ xuống CH3 < 100.
- Người dùng chọn: **ARM = CH5, vị trí lên (200) = ARMED**, giữa/xuống = DISARMED (khi mất sóng bộ thu đưa CH5 về 1000 = DISARMED). **MODE = CH6: lên = Angle, giữa = Alt Hold, xuống = dự phòng.** Ngưỡng: < 600 = lên, > 1400 = xuống (`config.h`).

### M4 bước 2: ESC + arm/disarm + failsafe (2026-10-15, code xong, CHƯA nạp, chờ người dùng xác nhận đã tháo cánh)
**Thay đổi ở ESC / arm / failsafe (người dùng cần đọc lại):**
- `bsp/pwm_esc.c`: nơi DUY NHẤT ghi TIM3 (đã grep). Lúc khởi động (`app_init`) nạp CCR = 900 µs rồi mới bật 4 kênh PWM ⇒ xung đầu tiên đã là 900. Mọi giá trị bị kẹp trong [900, 2000] µs. Giữ 50 Hz, tick 1 µs như code cũ. **Không có hiệu chuẩn ESC.**
- `esc_emergency_stop()` (ghi thẳng CCR = 900) được gọi trong HardFault/MemManage/BusFault/UsageFault, `Error_Handler`, `configASSERT`, hook tràn stack / hết heap ⇒ khi firmware lỗi, motor dừng rồi chip treo (thay cho IWDG đã bỏ).
- **Watchdog đầu ra ESC** (thay cho IWDG, không reset MCU): trong ngắt tick TIM7 (NVIC 0, không bị BASEPRI chặn), nếu motor đang > 900 µs mà `task_ctrl` không ghi PWM quá 50 ms ⇒ ép 900 µs, bật cờ ⇒ `task_ctrl` disarm (lý do "motor output watchdog") và yêu cầu gạt lại công tắc.
- `app/arming.c` (C thuần, test PC): 
  - **Arm duy nhất bằng: gạt CH5 lên (cạnh, ổn định 200 ms) khi cần ga ở thấp nhất (< 5 %)** (người dùng quyết định 2026-10-15, đã bỏ arm bằng cử chỉ yaw phải 3 s). Rời vị trí ARM ⇒ disarm ngay ở mọi mức ga. Vị trí công tắc lúc bật nguồn KHÔNG arm.
  - Điều kiện arm: RC hợp lệ, ga < 5 %, cần roll/pitch ở giữa (< 0,2), `imu_ok`, đã hiệu chuẩn, |roll|,|pitch| < 15°, không đang failsafe/hồi phục, watchdog ESC không bật, đã gạt công tắc về DISARM sau lần disarm tự động. Bị từ chối ⇒ `MSG,ARM refused (...)` kèm lý do.
  - Disarm: công tắc, cần (ga < 5 % + yaw trái 3 s), lệnh `DISARM` từ GUI, mất IMU (ngay lập tức), watchdog ESC, failsafe giai đoạn 2, lật > 70° trong 0,3 s, ga ở mức thấp nhất 15 s liên tục (`MSG,DISARMED: <lý do>`). Sau disarm do GUI/failsafe/lật/IMU/watchdog phải gạt công tắc về DISARM rồi lên lại.
  - Failsafe mất sóng (§7.7): mất < 0,2 s giữ lệnh cuối; 0,2–1 s giai đoạn 1 (roll/pitch/yaw = 0, giữ ga cuối); ≥ 1 s giai đoạn 2 ⇒ disarm (chưa có ước lượng độ cao). Hồi phục cần 0,5 s tín hiệu tốt. Giá trị failsafe của bộ thu (CH5 = 1000) bị bỏ qua khi RC không hợp lệ ⇒ không disarm sớm qua "công tắc".
  - Motor: STOP (900 µs) → IDLE khi vừa arm (950 µs) → SPOOL_UP (giới hạn ga tăng dần 0,5 s) khi ga > 5 % → RUN; ga < 3 % ⇒ về IDLE.
- `task_ctrl`: `motors_step()` chạy MỌI chu kỳ (kể cả khi IMU timeout). M4: cả 4 motor cùng giá trị ga (chưa mixer/PID). Ánh xạ motor → kênh TIM3 = đồng nhất (`CFG_MOTOR_TO_TIM_CH`), kiểm ở M5.
- `task_telem`: lệnh `DISARM` (`OK,DISARM`); dòng `T` có arm, mode, fs, thr, m1..m4. GUI: nhãn ARMED/DISARMED lớn, motor µs, chế độ, failsafe.
- Test PC (`tests/test_arming.c`, 10 nhóm): công tắc lúc bật nguồn không arm, cạnh công tắc, chống dội 100 ms, từng điều kiện arm, arm/disarm bằng cần (kể cả cấm khi công tắc ở DISARM và cấm disarm bằng cần khi ga cao), failsafe giai đoạn 1/2 + bắt buộc gạt lại công tắc, hồi phục 0,5 s, tự disarm, GUI/IMU/lật, spool-up. **Đạt.** Kiểm tra đột biến: 5 lỗi cố ý (vị trí lúc bật nguồn = cạnh, bỏ chốt gạt công tắc, bỏ giai đoạn 2, đọc công tắc khi mất RC, disarm bằng cần ở ga cao) đều bị test bắt.
- Build 0 warning; FLASH 53,0 KB; RAM 47,2 KB.
- Lệch plan: tự disarm 15 s, disarm bằng cần, watchdog đầu ra ESC thay IWDG, arming + failsafe chung `app/arming.c`.

### Kết quả thử M4 bước 2 trên mạch (2026-10-15, đã tháo cánh, pin + USB, kit WeAct)
- Nạp bằng `STM32_Programmer_CLI`, verify OK.
- Log GCS: `ARMED by switch`; `DISARMED: switch`; `ARM refused: throttle-not-low`; `FAILSAFE stage 1` → `DISARMED: RC failsafe stage 2`; `OK,DISARM` → `DISARMED: GCS command`; `DISARMED: idle throttle timeout` (2 lần); `DISARMED: stick (yaw left)`. Tất cả đúng luật.
- Người dùng xác nhận: 4 motor tăng đều theo ga giống code cũ; bài 7 đạt (phải gạt lại CH5 sau failsafe).
- **Idle 950 µs (của code cũ) không khởi động được motor từ đứng yên**; motor đang quay thì 950 µs vẫn giữ được. Ngưỡng khởi động đo được ~1070 µs ⇒ `CFG_MOTOR_IDLE` = 1090 µs.
- Đã thử 400 Hz + idle 1090 µs, rồi 50 Hz + idle 1000 µs. **Chốt (người dùng, 2026-10-15): ESC 400 Hz, idle 1000 µs.** `ESC_PWM_HZ` = 400 (ARR = 2499 đặt trong `pwm_esc_init`, `.ioc` vẫn ghi 19999), `CFG_MOTOR_IDLE` = 100/1100. Lưu ý: 1000 µs < ngưỡng khởi động đo ở 50 Hz (~1070 µs) ⇒ có thể motor không tự quay ngay khi vừa arm; người dùng chấp nhận.

### Tham khảo ArduPilot (đọc mã nguồn master, 2026-10-13)
- `RC_Channels::rudder_arm_disarm_check()`: |yaw| > 4000/4500 giữ 3000 ms; phải → arm, trái → disarm; arm cần ga = 0; disarm bằng cần bị từ chối nếu ga > 0; cần thấy yaw về giữa trước khi arm lại.
- `RC_Channel` ARMDISARM: tác động theo **cạnh** (đổi vị trí + ổn định 200 ms); vị trí lúc khởi động chỉ ghi nhận, **không** arm.
- `AP_Arming_Copter`: arm bị từ chối khi ga > 0, góc nghiêng vượt giới hạn, tư thế không hợp lệ, đang failsafe; disarm bằng cần bị từ chối khi chưa chạm đất (trừ chế độ ga tay).
- `AP_MotorsMulticopter::output_logic()`: máy trạng thái SHUT_DOWN → GROUND_IDLE (SPIN_ARM 0,10) → SPOOLING_UP (SPOOL_TIME) → THROTTLE_UNLIMITED → SPOOLING_DOWN; disarm ép ngay về SHUT_DOWN, xuất PWM_MIN.
- `crash_check()`: nghiêng > 15°, sai lệch góc > 30°, gia tốc < 3 m/s², liên tục 2 s → disarm. `auto_disarm_check()`: chạm đất + ga thấp trong `DISARM_DELAY` → disarm.
- Thiết kế áp dụng: trạng thái arm DISARMED/ARMED qua `arm_request(method)` có kiểm tra; trạng thái motor STOP (900 µs) / IDLE (950 µs, PID tắt, reset I) / SPOOL_UP (~0,5 s) / RUN; disarm khi: công tắc/cần, lệnh GUI, failsafe giai đoạn 2, lật > 70° trong 0,3 s (§7.7), tự disarm khi ga thấp nhất lâu sau khi arm.

---

## M3

### Đã làm (2026-10-13)
- `lib/filter`: PT1 và biquad LPF Butterworth (công thức RBJ cookbook, dạng DF2T), tự "mồi" bằng mẫu đầu tiên để không có quá độ khi khởi động. `lib/kalman1d`: predict/update theo §7.3, có bản update có bao góc [−π, π] cho yaw. `lib/mathx.h`.
- `app/config.h`: giá trị mặc định tới khi có bảng tham số (M6): phép đổi trục, LPF gyro 150 Hz, LPF accel 25 Hz, hiệu chuẩn 2,5 s (ngưỡng rung gyro 0,1 rad/s, |a| 1 m/s²), σ Kalman 4°/s, 3°, P0 = (2°)².
- `imu_pipeline` (ISR): giải mã → đổi trục sang FRD → biquad gyro + PT1 accel → `g_imu_sample` (chưa trừ bias).
- `app/calib`: trung bình gyro/accel 2,5 s; rung quá ngưỡng thì làm lại và báo `MSG,CALIB motion detected`. Kết quả: `gyro_bias`, `acc_mean`, `acc_up_bias0` (dùng ở M9).
- `app/attitude`: Kalman 1D roll/pitch (đo bằng accel khi 0,9 g < |a| < 1,1 g), yaw chỉ tích phân gyro (la bàn ở M11); khởi điểm góc lấy từ accel lúc hiệu chuẩn xong. Đổi tốc độ góc thân → Euler: có, mặc định tắt.
- `task_ctrl`: hiệu chuẩn rồi chạy KF mỗi 2 chu kỳ (~530 Hz), `dt` lấy từ timestamp DWT của mẫu; ghi `g_attitude` (có thêm `rate[3]` p, q, r đã trừ bias), `g_calib`.
- `app/telem_proto`: dòng `T,...` đủ 27 trường theo §9.1 (góc độ, rate độ/s), định dạng fixed-point không dùng `%f`. `task_telem` gửi 50 Hz; các trường của mốc sau = 0. Bỏ tự chụp trace lúc khởi động (vẫn có lệnh `trace`).
- `tests/test_lib.c` + `tests/run_tests.ps1` (gcc MSYS2): biquad/PT1 −3 dB tại fc, Kalman khớp tính tay + P ổn định khớp nghiệm giải tích, tích phân gyro, update có bao góc, định dạng dòng T (dấu âm, làm tròn, 27 trường). **Tất cả đạt.**
- `gcs/gcs.py` (PySide6 + pyqtgraph + pyserial): chọn cổng, đồ thị 20 s theo nhóm (Góc, Rate, Độ cao, Motor, Thời gian vòng lặp), khung log MSG/OK/ERR/TRC, ô gửi lệnh, nút DISARM đỏ (firmware chưa xử lý lệnh này tới M4/M6), ghi mọi dòng ra `gcs/logs/log_<thời gian>.csv`.
- Build: 0 warning; FLASH 48,2 KB; RAM 46,9 KB.

### Chỉnh theo phản hồi người dùng (GUI lag, khó nhìn)
- Firmware: `LOG_PERIODIC_STATS` (log.h, mặc định 0) tắt các dòng thống kê 1 Hz; chỉ còn dòng `T` 50 Hz + `MSG` sự kiện (CALIB, lỗi).
- GUI rút gọn: 1 đồ thị roll/pitch 20 s + số roll/pitch cỡ lớn; log chỉ hiện MSG/OK/ERR (TRC tùy chọn); vẽ lại 20 Hz bằng numpy, tắt antialias, chỉ vẽ khi có dữ liệu mới. Thử offscreen: 1000 dòng T đọc + vẽ trong 2,3 ms. Các nhóm đồ thị khác (rate, độ cao, motor) sẽ thêm lại khi tới mốc cần.

### Mất dòng `MSG,CALIB done` khi chưa mở cổng (2026-10-13)
- Nguyên nhân: khi PC chưa mở cổng, dòng `T` (~9 KB/s) làm đầy ring TX 4 KB trong ~0,5 s, các MSG sau đó bị bỏ.
- Sửa: đọc DTR (`CDC_SET_CONTROL_LINE_STATE`, wValue bit 0; `usbd_cdc.c` truyền gói setup vào `CDC_Control_FS`). Chỉ gửi `T` khi DTR = 1 và ring còn ≥ 1 KB. Mỗi lần host mở cổng, `task_telem` gửi lại banner + trạng thái CALIB; "CALIB done" chỉ do `task_telem` báo.

### Gộp task vào một file (người dùng yêu cầu, 2026-10-13)
- `task_ctrl.c/.h`, `task_slow.c/.h`, `task_telem.c/.h` → `App/app/tasks.c/.h` (nội dung không đổi; "CALIB done" chỉ do task_telem báo). Build 0 warning.

### Lệch so với plan
- `attitude_t` có thêm `rate[3]` (p, q, r đã trừ bias) để telemetry dùng.
- Bias gyro trừ ở `task_ctrl` thay vì trong ISR (bộ lọc có độ lợi DC = 1 nên kết quả như nhau, ISR không phải đọc dữ liệu do task ghi).

---

## Hướng gắn BMI323 (TBD #3, đo 2026-10-13, chuẩn bị cho M3)

| Tư thế | acc chip (mm/s²) X, Y, Z |
|---|---|
| Nằm ngang | −340, −90, +9670 |
| Mũi dựng lên | +1400, −9550, +350 |
| Cánh phải chúc xuống | +9600, −150, −250 |

- Người dùng xác nhận: mũi dựng lên → X_s ≈ +9600; tư thế Y_s ≈ −9550 là **cánh trái** chúc xuống (lúc đầu nhầm trái/phải). Bảng trên: dòng "Mũi dựng lên" thực ra là cánh trái chúc xuống, dòng "Cánh phải chúc xuống" thực ra là mũi dựng lên.
- Phép đổi trục chip → thân FRD: **x_b = +X_s, y_b = −Y_s, z_b = −Z_s** (định thức +1).
- Nằm ngang sau đổi trục: (−340, +90, −9670) ⇒ pitch_acc ≈ −2,0°, roll_acc ≈ −0,5° trên mặt bàn hiện tại.
- Gyro dùng cùng phép đổi trục (cùng chip). Kiểm tra lại bằng thử nghiêng ở M3.
- `LOG_RAW_MODE` trong `log.h`: 0 = log bình thường, 1 = accel X Y Z (mm/s²), 2 = gyro X Y Z (°/s), hệ trục chip, 5 Hz.

## M2

### Đã làm (2026-10-13)
- `bsp/spi_imu`: truyền SPI chờ (polling) dùng lúc init; khi stream: EXTI INT1 → ghi DWT → CS xuống → `HAL_SPI_TransmitReceive_DMA` 14 byte → callback DMA xong → CS lên → gọi xử lý. Báo overrun nếu cạnh INT1 đến khi burst trước chưa xong. CS ghi bằng BSRR.
- `drivers/bmi323`: init theo §6.1 (đọc giả 2 lần để chuyển sang SPI, kiểm CHIP_ID, soft reset 0xDEAF, đọc giả lại, ghi IO_INT_CTRL/INT_CONF/INT_MAP2/ACC_CONF/GYR_CONF, chờ 50 ms, kiểm ERR_REG = 0 **và đọc lại ACC_CONF/GYR_CONF**). Hàm delay được truyền vào để driver không phụ thuộc RTOS. Đổi đơn vị: gyro 16,384 LSB/(°/s) → rad/s; accel 4096 LSB/g → m/s².
- `app/imu_pipeline`: ISR DMA giải mã, ghi `g_imu_sample` (hệ trục cảm biến, chưa lọc, chưa trừ bias), mỗi 3 mẫu `osSignalSet(ctrlTaskHandle, SIG_IMU)`.
- `task_ctrl`: init IMU (lỗi thì không bật IWDG, đứng yên để telem báo lỗi); sau đó `osSignalWait(SIG_IMU, 5 ms)`; **chỉ cho ăn IWDG khi có dữ liệu IMU mới** ⇒ IMU im lặng > ~100 ms thì reset. Chu kỳ danh định 937 µs.
- `task_telem` mỗi giây in thêm dòng `MSG,imu init .. id .. err .. | .. Hz ovr .. spierr .. | gyro mrad/s .. | acc mm/s2 .. |a| ..`.
- Build: 0 warning; FLASH 37,2 KB; RAM 42,2 KB.

### Bộ ghi vết luồng chạy (theo yêu cầu người dùng, 2026-10-13)
- `bsp/trace.c`: `trace_mark(event)` ghi (mã sự kiện, DWT) vào bộ đệm 512 sự kiện, khóa bằng PRIMASK vài lệnh (~0,1 µs), gọi được từ mọi ISR/task. `trace_boot("...")` ghi mốc khởi động.
- Điểm ghi: ngắt EXTI IMU, DMA SPI xong, overrun/lỗi SPI, ngắt TIM7 tick, ngắt USB OTG_FS, `task_ctrl` xong chu kỳ / timeout, và **mỗi lần FreeRTOS chuyển task** qua macro `traceTASK_SWITCHED_IN` (USER CODE trong `FreeRTOSConfig.h`). Ngắt SysTick (tick RTOS) không ghi được vì do port FreeRTOS định nghĩa trực tiếp.
- `task_telem` in `MSG,BOOT ...` cho từng mốc khởi động; tự chụp 1 lần sau 3 s rồi in `TRC,<t> us +<dt>  <sự kiện> is running` (6 dòng/20 ms); gõ `trace` + Enter để chụp lại. Lệnh khác trả `ERR`.
- `printf` vẫn chỉ ở `task_telem`; ISR và `task_ctrl` chỉ ghi vết. RAM tăng ~4 KB (46,5 KB tổng).

### Quyết định bỏ IWDG (người dùng, 2026-10-13)
- Đã bỏ `MX_IWDG_Init`/`HAL_IWDG_Refresh` (IWDG vẫn "Activated" trong `.ioc` nhưng không được khởi động). Thay bằng cờ `ctrl_stats.imu_ok` (= 0 khi init lỗi hoặc ~20 ms không có dữ liệu IMU); **M4 phải bắt buộc `imu_ok = 1` để arm và cắt motor khi `imu_ok = 0`**.
- Lệch plan: §4.1 (task_ctrl cho ăn IWDG), §10.4, mục "Không cắt: IWDG" ở §11. Rủi ro còn lại: firmware treo khi motor đang quay thì TIM3 vẫn xuất xung cuối.
- `log_printf` → `mprintf`: bộ đệm 128 byte trên stack của task gọi, `vsnprintf`, ghi ring TX trong PRIMASK; gọi được từ mọi task (không từ ISR). Mỗi task tự in 1 dòng/giây `MSG,task_xxx is running: ...`.
- `APP_IMU_STREAM` (task_ctrl.c): 0 = chỉ init IMU, không chạy EXTI/DMA (để khoanh vùng lỗi).

### Sự cố "không hiện cổng COM" (2026-10-13): do nạp flash lỗi, không do code
- Nạp code cũ bằng `STM32_Programmer_CLI` lần 1 báo `Error: failed to erase memory`; option bytes bình thường (RDP 0, không WRP), VDD 3,28 V. Lần 2 (SWD 1 MHz) nạp + verify OK → code cũ hiện cổng COM ⇒ phần cứng/PC ổn.
- Kết luận: lần nạp trước từ CubeIDE nhiều khả năng đã lỗi giữa chừng (flash bị xóa một phần) nên mọi bản "nạp lại" đều không chạy. Từ nay kiểm tra `Download verified successfully` sau mỗi lần nạp.
- Người dùng đã khôi phục code về bản có IWDG vì tưởng code mới gây lỗi; sau khi tìm ra nguyên nhân, người dùng chọn **áp lại bản không IWDG** (mprintf, `imu_ok`, `APP_IMU_STREAM`). Đã nạp `build/Drone.elf` bằng CLI, verify OK.
- Lệnh nạp: `STM32_Programmer_CLI -c port=SWD freq=1000 mode=UR reset=HWrst -w build/Drone.elf -v -rst`.

### Lệch so với plan
- `imu_sample_t` dùng `t_cyc` (số chu kỳ DWT) thay cho `t_us`: DWT tràn sau 25,5 s nên chỉ dùng hiệu; `dt` = hiệu/168.
- Chưa có `docs/` (datasheet). Giá trị thanh ghi lấy từ §6.1 của plan, có `// VERIFY:` ở cấu trúc trường ACC_CONF/GYR_CONF.

---

## M1

### Đã làm (2026-10-13)
- `bsp/time_us`: DWT CYCCNT 168 MHz, chỉ dùng hiệu `uint32_t` (tràn sau ~25,5 s).
- `bsp/dbg_gpio.h`: macro DBG1 (mức cao trong thân `task_ctrl`), DBG2 (đảo mỗi lần `task_slow` chạy), DBG3 (đảo mỗi lần `task_telem` chạy). Ghi bằng BSRR (không RMW ODR). Chưa có chân → macro rỗng.
- `app/shared_state`: tín hiệu `SIG_IMU/SIG_TOF/SIG_USB_RX`, handle task, `shared_read/shared_write` chép cả struct trong `taskENTER_CRITICAL()`.
- `task_ctrl` (Realtime, 1024 w): stub, chờ `osSignalWait(SIG_IMU, 1)` nên chạy ~1 kHz theo tick cho đến M2. Đo thời gian chạy và jitter chu kỳ bằng DWT (bỏ 1000 chu kỳ đầu). **Khởi động IWDG (~100 ms) ở đầu task** (không phải lúc boot) và cho ăn mỗi chu kỳ; `__HAL_DBGMCU_FREEZE_IWDG()`. `APP_TEST_IWDG_HANG` (mặc định 0) để thử reset do IWDG.
- `task_slow` (High, 1024 w): stub, `osSignalWait(SIG_TOF, 10)` → ~100 Hz khi chưa có ToF.
- `task_telem` (BelowNormal, 1024 w, task CubeMX): `osDelayUntil` 20 ms; mỗi giây in `MSG,ctrl .. Hz exec_max .. jitter_max .. | slow .. Hz | telem .. Hz | stack free ...`; vẫn echo RX.
- `rtos_hooks.c`: hook stack overflow / malloc failed: tắt ngắt và treo → IWDG reset (đã kiểm tra bằng `nm` là bản này được link, không phải bản `__weak`).
- Không có thay đổi nào ở ESC/arm/failsafe/mixer (TIM3 vẫn chưa xuất xung).
- Build: 0 warning; FLASH 33,5 KB; RAM 42,1 KB.

### Ghi chú
- Không có chân debug ⇒ tiêu chí "đo bằng oscilloscope" tạm thay bằng số đếm DWT gửi qua USB.
- Khi treo trong hook/Error_Handler, PWM giữ giá trị CCR cuối cho tới lúc IWDG reset (≤ ~100 ms). Ở M4 sẽ cho các đường lỗi đặt CCR về mức dừng trước khi treo.

---

## M0

### Đã làm (2026-10-12)
- `CLAUDE.md`; cấu trúc `App/{bsp,drivers,lib,app}`; `App/app_main.c` gọi từ `main.c` (USER CODE 2 và USER CODE 5).
- `App/bsp/usb_cdc_port.c`: ring buffer TX 4 KB / RX 512 B. RX chép trong ngắt OTG_FS rồi `osSignalSet`; TX chỉ do một task gọi, không chờ bận, bộ đệm gửi là biến tĩnh. `_write` chuyển `printf` ra USB.
- `App/app/log.c`: `log_printf` dùng `vsnprintf` với bộ đệm tĩnh (không để newlib cấp phát bộ đệm stdout bằng malloc).
- `usbd_cdc_if.c` (USER CODE): gọi `usb_cdc_port_on_rx_isr`; thêm kiểm tra `pClassData == NULL` trong `CDC_Transmit_FS` (bản gốc sẽ HardFault nếu gửi trước khi enumerate).
- Linker `STM32F405RGTX_FLASH.ld`: FLASH còn 896 KB, dành sector 11 (`0x080E0000`) cho tham số.
- `.cproject`: thêm source `App` và include `../App` cho cả Debug/Release.
- `tools/build.ps1` + `tools/Makefile`: build dòng lệnh bằng GCC 14.3 trong CubeIDE. App/ dùng `-Wall -Wextra -Wdouble-promotion -Wshadow -Wformat=2`.
- Đã xác minh `osSignalSet` trong `cmsis_os.c` có nhánh `inHandlerMode()` → gọi `xTaskGenericNotifyFromISR` (an toàn trong ISR).
- Kết quả build: 0 warning; FLASH 31,9 KB / 896 KB; RAM 41,9 KB / 128 KB.

### Chương trình M0 làm gì
- Người dùng không cần LED: M0 chỉ kiểm tra luồng USB CDC (TX + RX).
- In banner khi khởi động (SYSCLK, nguyên nhân reset), sau đó in `alive N s` mỗi giây.
- **Echo**: gõ gì trên terminal thì drone gửi lại đúng như vậy (để thử đường RX).
- **Không** đụng tới TIM3/ESC: chưa gọi `HAL_TIM_PWM_Start`, chân ESC không có xung.

### Vấn đề còn mở
- Chưa có chân debug trong `.ioc` (cần cho M1). Nạp bằng ST-Link qua CubeIDE (đã xác nhận). USB CDC từng chạy với code cũ (đã xác nhận).
- Kênh ARM/MODE của tay cầm: người dùng chưa biết → xác định ở M4 bằng cách xem giá trị kênh trên GUI.
- `.ioc` cần sửa (bảng bên dưới) rồi generate lại.

---

## Rà soát `Drone.ioc` lần 3: **đạt toàn bộ §5** (trừ chân debug, chưa cần cho M0)

- IWDG Reload = 99; I2C1 400 kHz (Fast); NVIC: EXTI9_5 + DMA SPI = 5, UART/DMA UART/OTG_FS = 6, USART1 = 7, TIM7 = 0.
- Đã xóa `Drivers/CMSIS/{Core,Core_A,DAP,DSP,NN,RTOS,RTOS2,Documentation}` (theo yêu cầu người dùng); còn `Device`, `Include`, `LICENSE.txt`. Build lại: 0 warning, FLASH 32,3 KB, RAM 42,0 KB.

## Rà soát `Drone.ioc` lần 2 (sau khi người dùng sửa và Generate)

Đã đạt: sinh mã cặp .c/.h; FreeRTOS (vTaskDelayUntil, StackHighWaterMark, MallocFailedHook, `telemTask` BelowNormal/1024); SPI1 DMA (Stream0/Stream3, Normal); PD2 mặc định High; EXTI9_5 ưu tiên 5; bật ngắt USART1; USART6 8 bit; IWDG đã bật, prescaler 32, không tự gọi `MX_IWDG_Init`. Linker và `.cproject` vẫn giữ phần đã sửa tay.

Claude đã sửa: task chuyển sang `freertos.c` nên phải thêm lại `app_telem_task(argument)` vào USER CODE `StartTelemTask`. Build lại: 0 warning.

Còn thiếu:
- IWDG Reload = 4095 (≈4 s) → cần **99** (≈100 ms).
- I2C1 vẫn 100 kHz → 400 kHz.
- NVIC: OTG_FS, USART2, USART6 và các DMA của UART (DMA1_Stream5/6, DMA2_Stream2/6) đang ở 5 → đặt **6** (USART1 = 7 cũng được). TIM7 timebase đang 15 → **0**.
- `Drivers/CMSIS` vẫn còn 846 file .c (DSP, NN, DAP, RTOS...) sót lại từ lần generate trước; CubeIDE sẽ biên dịch cả chúng. Chỉ cần giữ `Drivers/CMSIS/Device` và `Drivers/CMSIS/Include`.
- Chưa có chân debug (chưa cần cho M0).

## Rà soát `Drone.ioc` lần 1 so với PROJECT_PLAN §5

| Mục | Hiện tại | Yêu cầu | Đạt? |
|---|---|---|---|
| RCC | HSE 8 MHz, PLL M8 N336 P2 Q7; AHB 168, APB1 42, APB2 84; CSS tắt | như vậy | ✅ |
| SYS | SWD, timebase TIM7 | như vậy | ✅ |
| TIM7 (timebase) NVIC | ưu tiên 15 | 0 | ❌ |
| Code gen | tất cả trong `main.c` (`CoupleFile=false`) | cặp .c/.h cho từng ngoại vi | ❌ |
| Copy thư viện | chép toàn bộ CMSIS (846 file .c: DSP, NN, DAP...) | chỉ chép file cần thiết | ❌ (CubeIDE sẽ biên dịch cả các template này → lỗi build) |
| FreeRTOS | CMSIS_V1, heap 25000, stack overflow = 2, newlib reentrant | như vậy | ✅ |
| FreeRTOS include | `vTaskDelayUntil` = 0 | **1** (cần cho `osDelayUntil`) | ❌ |
| FreeRTOS include | `uxTaskGetStackHighWaterMark` = 0 | 1 | ❌ |
| FreeRTOS hook | `USE_MALLOC_FAILED_HOOK` = 0 | 1 | ❌ |
| FreeRTOS task | `FlightTask`, Normal, 512 word | đổi thành `telemTask`, `osPriorityBelowNormal`, 1024, hàm `StartTelemTask` (task_ctrl/task_slow tạo trong `app_main`) | ❌ |
| USB_OTG_FS | Device_Only, VBUS sensing tắt, CDC | như vậy (PA9 đang là USART1_TX nên phải tắt VBUS sensing) | ✅ |
| USB NVIC | ưu tiên 5 | 6 | ❌ |
| SPI1 | mode 0, /16 = 5,25 MHz, NSS phần mềm | như vậy | ✅ |
| SPI1 DMA | chưa có | RX: DMA2 Stream0 Ch3, TX: DMA2 Stream3 Ch3 (Stream2 đã dùng cho USART1_RX). Mode **Normal** (không Circular), Byte/Byte, Memory increment bật, FIFO tắt. Ưu tiên RX Very High, TX High; NVIC cả hai stream = 5. `// VERIFY: RM0090 bảng DMA2 request mapping` | ❌ |
| IMU_CS (PD2) | mức ban đầu **Low** | **High** | ❌ |
| IMU_INT (PB8) | EXTI cạnh lên, nhưng NVIC EXTI9_5 **chưa bật** | bật, ưu tiên 5 | ❌ |
| Chân debug | không có | 2–3 chân debug (label `DBG1`, `DBG2`, `DBG3`), cần cho M1. Không cần LED | ❌ (cần chân từ người dùng) |
| USART2 (MTF-01P) | 115200 8N1, RX DMA circular | như vậy | ✅ |
| USART2 NVIC | IRQ 5, DMA 5 | 6 | ❌ |
| USART1 (SBUS) | 100000, 9 bit (8 + parity chẵn), 2 stop, RX DMA circular | như vậy | ✅ |
| USART1 NVIC | **USART1 global interrupt chưa bật**; DMA2_Stream2 = 5 | bật (cần cho ngắt IDLE), 6; DMA 6 | ❌ |
| USART6 (HC-05) | 9600, **9 bit, không parity** | 8N1 (sau này 115200) | ❌ (9 bit không parity là sai) |
| USART3 (GPS) | 115200 | ngoài phạm vi, để nguyên cũng không sao | — |
| I2C1 | 100 kHz | 400 kHz (kiểm tra điện trở kéo lên) | ❌ |
| TIM3 (ESC) | PSC 83 → 1 MHz, ARR 19999 → **50 Hz**, ARR preload bật | giữ 50 Hz như code cũ cho lần chạy đầu (§6.0 bước 4); thử 400 Hz sau | ✅ |
| TIM2 | 32 bit chạy tự do, chưa dùng | không cần (dùng DWT) | — |
| IWDG | chưa bật | bật, prescaler 32, reload 99 (≈100 ms với LSI 32 kHz). Project Manager → Advanced Settings → tick "Do Not Generate Function Call" cho IWDG để app tự gọi `MX_IWDG_Init()` sau khi task_ctrl chạy (M1) | ❌ |
| Linker | | sector 11 dành cho tham số | ✅ (đã sửa tay) |

---

## Kiểm kê code cũ (`drone_stm32f405rgt6/`, §6.0 bước 1)

Toàn bộ là **HAL**, vòng lặp chính (superloop) + `HAL_Delay(2)`, không RTOS. Mọi ngắt ở **ưu tiên 0**.

| Module | File | Tóm tắt |
|---|---|---|
| main | `Core/Src/main.c` | Superloop: Flow → GPS → Baro → Mag → IMU → PID; telemetry mỗi ~50 vòng qua USART6 và USB CDC |
| Tay cầm | `Core/Src/rc_sbus.c` | SBUS, xem bên dưới |
| ESC + PID + mixer | `Core/Src/flight_control.c`, `tim.c` | Xem bên dưới |
| IMU | `Core/Src/sensor_imu.c` | BMI323 SPI1 polling (blocking), ACC_CONF = GYR_CONF = 0x7008 (ODR 100 Hz, ±2 g, ±125 °/s), complementary filter 0,98. Sẽ viết lại theo §6.1 |
| La bàn | `Core/Src/sensor_gps_mag.c` | I2C1 địa chỉ **0x0E**, ghi 0x0A = 0x0B, đọc 6 byte từ 0x03 → khớp IST8310 (vẫn phải quét I2C ở M8) |
| Baro | `Core/Src/sensor_baro.c` | I2C1 địa chỉ **0x76**, bản đồ thanh ghi kiểu DPS310/DPS368. Lỗi: oversampling 16× nhưng không bật P_SHIFT (thanh ghi 0x09) |
| ToF/flow | `Core/Src/sensor_flow.c` | USART2 DMA circular, phân tích MSPv2 nhưng **không kiểm CRC**, chỉ nhận `<`, bỏ byte cao của cmd/size. Viết lại theo §6.3 |
| GPS | `Core/Src/sensor_gps_mag.c` | USART3 ngắt từng byte, NMEA GGA. Ngoài phạm vi |
| USB CDC | `USB_DEVICE/` | Không sửa gì so với bản CubeMX sinh; gọi `CDC_Transmit_FS` thẳng từ vòng lặp chính |

### Chân và ngoại vi (code cũ, giống `.ioc` mới)

| Chức năng | Chân | Ngoại vi |
|---|---|---|
| ESC 1..4 | PA6, PA7, PB0, PB1 | TIM3 CH1..CH4 |
| SBUS | PA10 (RX), PA9 (TX, không dùng) | USART1 |
| MTF-01P | PA2 (TX), PA3 (RX) | USART2 |
| GPS | PC10, PC11 | USART3 |
| HC-05 | PC6, PC7 | USART6 |
| UART4 (để dành) | PA0, PA1 | UART4 (chỉ cấu hình chân) |
| IMU SPI | PB3 SCK, PB4 MISO, PB5 MOSI; CS = PD2; INT1 = PB8 | SPI1 |
| La bàn + baro | PB6 SCL, PB7 SDA | I2C1 |
| USB | PA11, PA12 | OTG_FS |
| HSE | 8 MHz | |

### Tay cầm (code cũ)
- **Giao thức:** SBUS trên USART1 RX (PA10), 100000 baud, 8E2. STM32F4 không đảo được mức RX → trên mạch phải có mạch đảo, hoặc bộ thu xuất SBUS không đảo. Code cũ đọc được nên phần cứng đã ổn.
- **Cách nhận:** `HAL_UART_Receive_IT` từng byte, phân tích trong callback ngắt (`HAL_UART_RxCpltCallback` trong `main.c`). DMA cho USART1_RX có cấu hình nhưng không dùng. Ưu tiên ngắt 0.
- **Khung:** chờ byte đầu 0x0F, đủ 25 byte thì kiểm tra byte cuối == 0x00. **Không** đọc byte cờ (byte 23: frame lost, failsafe). **Không** có timeout mất tín hiệu.
- **Kênh** (chỉ giải mã CH1..CH4, thứ tự AETR):

| Kênh | Chức năng | Min | Giữa | Max | Ghi chú |
|---|---|---|---|---|---|
| CH1 | Roll | 627 | 1017 | 1400 | trái → phải |
| CH2 | Pitch | 737 | 1126 | 1400 | 737 = đẩy cần **lên trước** |
| CH3 | Throttle | 200 | — | 1768 | |
| CH4 | Yaw | 601 | 976 | 1376 | trái → phải |

  Mỗi kênh được ánh xạ tuyến tính từng đoạn (min→1000, giữa→1500, max→2000) rồi giới hạn trong [1000, 2000].
- **Không có công tắc ARM / MODE** trong code cũ.
- Biến chia sẻ (`rc_channel[]`, `throttle_pwm`...) ghi trong ISR, đọc ở vòng lặp chính, không `volatile`, không chép nguyên khung.

### ESC (code cũ)
- **Timer:** TIM3 CH1..CH4 (PA6, PA7, PB0, PB1). Clock timer APB1 84 MHz, PSC 83 → **tick 1 MHz (1 µs)**, ARR 19999 → **50 Hz**.
- **Dải xung:** lúc mới khởi động CCR = 0 (không có xung); **dừng = 900 µs**; khi ga > 1050 thì sàn **950 µs** (idle); **tối đa 2000 µs**. Ga tay cầm 1050–2000 được ánh xạ sang 950–2000 µs.
- **Hiệu chuẩn ESC đã làm với dải 900 µs (min) – 2000 µs (max)** (`Motor_Init_And_Calibrate_By_RC`: xuất 2000, chờ kéo ga xuống, xuất 900). ⇒ Driver mới **giữ dừng = 900 µs, max = 2000 µs**, không phải 1000 µs như ví dụ trong §6.5. Không chạy lại hiệu chuẩn.
- **Preload CCR:** có (`HAL_TIM_PWM_ConfigChannel` tự bật OCxPE, đã kiểm tra trong `stm32f4xx_hal_tim.c`); ARR preload bật.
- **An toàn:** không có khái niệm arm; đẩy ga > 1050 là motor quay. Khởi động mà ga > 1800 thì vào chế độ hiệu chuẩn ESC. Dùng `HAL_Delay` chờ 4 s.
- **Mixer cũ** (`out_m1..m4` ↔ CH1..CH4): `m1 = T + P − R − Y`, `m2 = T − P − R + Y`, `m3 = T + P + R + Y`, `m4 = T − P + R − Y`, với quy ước cũ "roll phải +, pitch **chúi mũi** +". Nếu dấu của code cũ đúng thì suy ra: M1 sau‑phải, M2 trước‑phải, M3 sau‑trái, M4 trước‑trái. **Chỉ là suy luận, chưa kiểm chứng** — sẽ hỏi và thử ở M5.
- PID cũ: vòng góc trực tiếp (không phân tầng), Kp 8, Kd 3 (đơn vị µs/độ). Sẽ thay bằng PID phân tầng §7.5.

### Telemetry cũ
- USB CDC hoạt động với HSE 8 MHz (code cũ in dữ liệu tay cầm qua USB) → xác nhận HSE 8 MHz và D−/D+ nối PA11/PA12 (cần người dùng xác nhận đã thấy cổng COM với code cũ).

---

### [2026-10-22] - Cài đặt Kiro-Antigravity Bridge
- **Tích hợp kênh giao tiếp song song (kiro-antigravity-bridge):**
  - Thiết lập thư mục .agents/ và .kiro/ tại gốc dự án Drone nhằm chia sẻ ngữ cảnh và điều phối nhiệm vụ 2 chiều giữa Kiro và Antigravity IDE.
  - Cập nhật thông tin kiến trúc Drone vào .agents/context/project.md và sẵn sàng cơ chế hook tự động tải nhiệm vụ tồn đọng.
