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
| M6 | **Xong** (2026-10-28, người dùng chuyển giai đoạn) | Tham số SET/GET/LIST/SAVE, hộp đen + DUMP, ACRO, CAPTURE, liên kết ESP-NOW. Xem "M6" bên dưới |
| M7 | **Đang làm**: chỉnh rate trên giá xong (P 0,24, I 0,1, D 0,00112, lọc gyro 90 / D 50). Còn: siết ốc trục cánh (người dùng tìm thấy ốc lỏng) rồi capture lại ở ga ~0,48, SAVE, chỉnh `ang_p_rp`, hover tay 30 s, ghi `hover_thr` | Bo mạch đang bắt cứng vào khung (không đệm giảm rung). 2026-10-28: bay tự do Angle trôi **lùi + phải** (theo mũi, mọi hướng) ⇒ lệch gắn trên drone; thêm `trim_roll`/`trim_pitch` (độ, ±5, cộng vào góc đặt Angle, không ảnh hưởng Acro; tổng 30 tham số; có trong ô chỉnh nhanh GUI). Chưa nạp. Cùng ngày: lỗi ký tự lạ + "mất dữ liệu" (log 19:41) là **vòng phản hồi**: RX USART6 nghe lại chính TX của drone (dòng P/T quay về thành lệnh `ERR,P,...unknown`), echo + ERR khuếch đại vô hạn. Sửa `poll_commands`: chỉ echo nguồn USB; dòng có dấu phẩy hoặc byte không in được bị bỏ, không trả ERR. Nguồn gốc phần cứng (PC7 hở/ESP không cấp nguồn) chưa xác định |
| M8 | **ToF xong** (2026-10-28, người dùng xác nhận sai số < 3 cm). **La bàn + baro: code xong, chưa thử** (người dùng đổi ý, yêu cầu viết luôn; hiệu chuẩn sau): `drivers/ist8310.c` (đo đơn mỗi 20 ms, CNTL3 16 bit, AVGCNTL 16×, PDCNTL normal, 1320 LSB/G), `drivers/dps310.c` (soft reset, chờ COEF_RDY+SENSOR_RDY, P 32 Hz ×16 + P_SHIFT, T 8 Hz ×1 nguồn theo COEF_SRCE, liên tục; công thức bù PX4) — thanh ghi/giá trị theo driver PX4 (BSD, đọc 2026-10-28), ID 0x10 = DPS310 (DPS368 = 0x11). `app/i2c_sensors.c` trong task_slow: `g_mag` (thân FRD, trừ `mag_off_*`; `CFG_MAG_AXIS_*` theo PX4 {x,y,−z}, cần VERIFY), `g_baro` (p0 = trung bình 64 mẫu đầu, alt_rel PT1 2 Hz → trường `baro_rel` của dòng T), thử init lại mỗi 2 s chỉ khi disarm. Lệnh `SENSORS`, `MAGCAL [s]` (min/max → in 3 lệnh SET). Tham số mới `mag_off_x/y/z` (tổng 25). Test PC (dấu, hệ số, bù, giải mã IST8310) đạt; build 0 warning. Chi tiết: | |
| M8 (chi tiết) | (bước 1, 2026-10-28): người dùng xác nhận GPS module (GEP-M10-DI) dùng USART3 (PC10/PC11) + I2C1 (PB6/PB7), MTF-01P ở USART2 (PA2/PA3). Lệnh `I2CSCAN` (task_slow, chỉ khi disarm) và `FLOWRAW` (1 s thống kê byte USART2: B/s, số header `$X`/`$M`, 24 byte đầu); `bsp/flow_uart.c` RX DMA vòng của CubeMX, đọc theo NDTR, tự khởi động lại sau lỗi UART. Code xong, chưa nạp. Người dùng: MTF-01P đã đặt MSP; GPS (USART3) chưa dùng, module chỉ dùng la bàn + baro qua I2C1. Kết quả: I2C1 có **0x0E** và **0x76** (lỗi bus 0); USART2 3233 B/s, 202 `$X`/s (MSPv2 ~2 × 100 Hz). Gói mẫu: `24 58 3C 00 | 02 1F | 09 00 | 3E FE FF FF FF 01 00 00 00 | 45` = cmd 0x1F02, 9 B payload (u8 + 2 × i32). Lệnh qua sóng chạy lại. Thêm: `I2CSCAN` đọc thanh ghi ID ứng viên (0x0E/0x00, 0x76/0x0D, 0xD0, 0x00…) để xác nhận chip; mprintf giữ `\r\n` khi cắt, 192 ký tự. ID: 0x0E/0x00 = 0x10 (**IST8310**), 0x76/0x0D = 0x10 (**DPS310/DPS368**), 0x76/0xD0 và 0x00 = 0x00 (không phải BMP). **Driver MTF-01P xong** (chưa nạp): `drivers/mspv2.c` (parser + crc8_dvb_s2, CRC kiểm bằng gói thật), `drivers/mtf01p.c` (0x1F01 range, 0x1F02 flow), `g_tof` (task_slow, hợp lệ khi trong tầm và < 50 ms), trường `tof` của dòng T (−1 khi không hợp lệ), `FLOWRAW` in thêm dòng `MSG,TOF`. Test PC đạt; build 0 warning. Đánh giá code cũ: baro DPS310 PRS_CFG 0x24 (4 Hz, 16×) / TMP_CFG 0xA0 (4 Hz, ngoài) / MEAS_CFG 0x07, không chờ COEF_RDY, không ghi CFG_REG (P_SHIFT khi oversampling > 8, cần VERIFY), không kiểm nguồn nhiệt độ COEF_SRCE, 4 Hz < 32–64 Hz của plan; la bàn IST8310 ghi 0x0B vào 0x0A, đọc 0x03..0x08 LE, không kiểm DRDY, không đổi trục sang thân. ⇒ Dùng địa chỉ/thanh ghi làm gợi ý, phải đối chiếu datasheet trước khi viết driver | |
| M9 | **Code xong, chưa thử trên mạch** (2026-10-28; người dùng tạm để M8 la bàn/baro chưa kiểm). `lib/alt_kf.c` (Kalman [z, vz] §7.4, cổng outlier 9), `app/alt_est.c` (bù nghiêng ToF, bỏ khi nghiêng > 30°, phát hiện bậc NORMAL/CONFIRM §7.4b(1): ngưỡng `alt_step_th`, 3 mẫu ±3 cm, 100 ms; bậc rồi về mức cũ = nhiễu; snap |h| < 3 cm; mất ToF > 0,5 s ⇒ valid = 0, có lại ⇒ kiểm tra bậc trước khi dùng; mất > 2 s ⇒ khởi tạo lại với h = 0 vì chưa có baro (thêm so với plan); 10 lần bị cổng loại liên tiếp ⇒ neo lại h (thêm so với plan, cho dốc thoải)). `task_ctrl` tính a_up trừ `acc_up_bias0`, trung bình 10 chu kỳ (`g_acc_up`); `task_slow` chạy 100 Hz (`g_alt`), báo `MSG,ALT ...`. Tham số `alt_sigma_a` 0,3, `alt_sigma_tof` 0,05, `alt_step_th` 0,10 (tổng 28). Dòng T điền z/vz/h_terrain/alt_src, thêm `alt_valid` (31 trường). Lệnh `ALTRAW 0/1` ⇒ dòng `A` 100 Hz; `tools/replay_kf.py` chép lại thuật toán, so với firmware trên dữ liệu giả: lệch ≤ 0,9 mm. GUI tab "Độ cao". Test PC (leo 1 m, bàn 0,7 m lên/xuống, nhiễu 1 mẫu, mất ToF, nghiêng 40°) đạt; build 0 warning. **Thử trên mạch 2026-10-28/29 (chưa bật ALTRAW):** cầm tay: z = ToF + h đúng, nhưng h_terrain bị đặt sai −0,1…−0,3 m (bậc giả khi nghiêng/lắc tay, "neo lại" giữ lỗi mãi) ⇒ z thấp hơn thật 15–20 cm; khi arm (cả ga idle trên đất) vz ≈ −2 m/s, z/h_terrain trôi tới −40 m ⇒ a_up lệch ≳ −1 m/s² khi motor quay (nghi bão hòa accel ±8 g, bo bắt cứng). **Sửa 2026-10-29 (người dùng chọn, CHƯA thêm trạng thái bias vào Kalman):** (1) accel ±16 g `ACC_CONF 0x703C`, 2048 LSB/g (Bosch bmi3_defs.h v2.4.0, BSD-3; **lệch PLAN §6.1** 0x702C/±8 g), đếm mẫu gần full-scale (|raw| ≥ 32000) ⇒ `MSG,IMU CLIP ...` mỗi 1 s khi có; (2) bỏ "neo lại" ⇒ 10 lần bị cổng loại liên tiếp thì khởi tạo lại z, vz từ ToF, giữ h (`MSG,ALT ... z restarted`); bậc chỉ chấp nhận khi nghiêng < 10° (nghiêng hơn: bỏ khung đó); disarm + ToF về mức nằm sàn (±3 cm, 0,5 s) ⇒ h = 0 (`MSG,ALT on the floor again`); lúc arm ⇒ z tham chiếu lại sàn dưới drone, h = 0 (`MSG,ALT armed`). Dòng `A` thêm cột armed (16 trường), replay_kf.py cập nhật. Thử PC: lệch accel b ⇒ z sai ổn định ≈ b/6 (−0,6 ⇒ 10 cm; −1 ⇒ tối đa 21 cm, 8 lần đồng bộ / 12 s; −2 ⇒ tối đa 24 cm) ⇒ vẫn cần trạng thái bias (hoặc hết bão hòa) trước M10 (±10 cm). **Log 18:10 (ALTRAW, cầm tay nhấp 0,55–1,1 m ~1 Hz, chúc mũi 30–48° ở điểm thấp):** a_up ±10 m/s² là thật (0,27 m × (2π)²); lỗi do logic: (a) sau mỗi khoảng mất ToF (nghiêng > 30°) sai số nhỏ của z bị ghi vào h_terrain (8–17 cm) ⇒ chênh < `alt_step_th` không còn là bậc; (b) khung nghiêng 10–30° bị bỏ mà không cập nhật mốc so sánh ⇒ dây chuyền bỏ khung, kẹt CONFIRM, z chỉ chạy bằng accel tới 2 s (lệch 1,9 m) ⇒ khung bị bỏ vẫn làm mốc, CONFIRM luôn hết hạn 100 ms; (c) không đồng bộ lại z khi nghiêng > 10°. Replay log đó: \|z − ToF\| TB 12,3 → 3,6 cm, p95 60,7 → 8,2 cm, h luôn 0 (còn 1 khung ToF đọc sai 2,11 m bị bỏ đúng; 1 đoạn 0,5 s lắc mạnh lệch > 15 cm rồi tự về). Test PC mới: nhấp tay 1 Hz nghiêng 40° | |
| M10 | **Bước 1 code xong, chưa thử** (2026-10-29, người dùng yêu cầu: bay treo lần tới phải có giữ độ cao; chỉ bay sau khi log thử motor trên mặt đất đạt). `app/alt_hold.c` (C thuần, test PC): z P → vz PI → ga, chia max(cosφcosθ, 0,5) (§7.5); CH6 giữa = bật; lúc bật z_sp = z, ga gốc = ga cần lúc đó; **lệch §7.5:** vùng chết cần ga ±0,1 quanh vị trí cần lúc bật (không phải 40–60 %, vì hover 0,41 nằm sát mép 0,4); ngoài vùng chết leo/hạ tới `vz_max`, z_sp không vượt z ± 0,5 m, không dưới 0,15 m. An toàn: ga kẹp trong ga gốc ± `alt_thr_rng`; từ chối bật khi z < 0,3 m hoặc không hợp lệ; tự thoát khi `alt_valid` = 0 hoặc \|z − z_sp\| > 1 m (phải gạt CH6 ra vào lại mới bật lại); thoát thì ga chuyển dần về cần trong 0,5 s; trong Alt Hold cần ga thấp không đưa motor về IDLE (không tự disarm idle, không disarm bằng cần; công tắc ARM vẫn disarm); failsafe giai đoạn 1 giữ độ cao nếu đang bật. Tham số `alt_p` 1,0, `vz_p` 0,10, `vz_i` 0,10, `vz_max` 0,5, `alt_thr_rng` 0,15 (tổng 35; chọn từ hover 0,41 ⇒ ~24 m/s² mỗi đơn vị ga; mô hình không trễ: gió 0,5 m/s² lệch tối đa 10 cm). Dòng T: `mode` = chế độ thật (1 chỉ khi đang giữ), `z_sp`, `thr` = ga ra mixer. GUI: tab Độ cao có z đặt, ô chỉnh nhanh thêm alt_p/vz_p/vz_i. **Tham khảo `s450drone/` (người dùng thêm 2026-11-10, Arduino STM32 + ICM42688 + cùng MTF-01P, không ghi giấy phép ⇒ chỉ lấy ý tưởng):** (a) tự học ga treo (TB trượt chậm khi treo yên) ⇒ `alt_hold.c`: khi giữ yên (cần trong vùng giữ, |vz| < 0,1, |z − z_sp| < 0,1, nghiêng < 15°, ga không chạm giới hạn) `hover` ← ga × cos nghiêng, τ 3 s; lúc bật dùng `hover` đã học (hoặc tham số `hover_thr` 0,41; 0 = theo cần như cũ) làm ga gốc; thoát/hạ cánh in `SET hover_thr …` nếu lệch ≥ 0,01; (b) expo cần ga ⇒ `thr_curve()` (lib/mathx.h, kiểu Betaflight thr_mid/thr_expo, điểm cố định 0/mid/1), `thr_mid` 0,41, `thr_expo` 0 (= tuyến tính như cũ); áp cho ga Angle và cần ga đưa vào Alt Hold (tổng 39 tham số); (c) flow: cặp trục x~roll, y~pitch, K_FLOW 0,2735 (chỉnh theo dịch chuyển) nhưng bù xoay chỉ 0,3 × gyro (dò tay) ⇒ hai hệ số không khớp (≈3,66 so với ≈1,10 đơn vị / rad/s), họ chưa khớp flow với gyro ⇒ `flow_cal.py` in so sánh; (d) bù điện áp pin (chưa làm: mạch chưa có cầu chia áp). **Giữ hướng (2026-11-10, người dùng: yaw trôi rõ với P yaw 0,15, phải đẩy lên 0,7):** bias gyro yaw sau CALIB chỉ ~1 mrad/s (3–4°/phút) ⇒ trôi là do yaw chỉ giữ tốc độ (lệch do nhiễu không được kéo về). `app/yaw_hold.c` (C thuần, test PC): cần yaw ở giữa (= 0 trong vùng chết, cả failsafe) ⇒ phanh tới |r| < 10°/s rồi nhớ hướng (yaw tích phân gyro), yaw đặt = `yaw_hold_p` × wrap(hướng nhớ − yaw), giới hạn 60°/s; cần lệch ⇒ tốc độ từ cần như cũ; reset khi chưa bay. `yaw_hold_p` 2,5 (0 = tắt, tổng 36 tham số), ô chỉnh nhanh GUI. Chỉnh PID cùng ngày (capture, ga ~0,39): D 0,00052 ⇒ roll vượt 126% + dập dềnh ~5 Hz; D 0,0011 / I 0,15 / notch 65–125 ⇒ trễ 30 ms, vượt 128%, vẫn dập dềnh ~5 Hz, rung 81 Hz giảm ~40%; lọc gyro/D đang 70/40 (trước 90/50) ⇒ đề xuất trả 90/50, sau đó P 0,21 nếu cần. **LEVELCAL (2026-11-10, ý người dùng: úp drone lên 4 nắp chụp motor = mặt phẳng cánh, vì chân đỡ không đều):** `app/levelcal.c` (C thuần, test PC) lấy trung bình gia tốc ~2 s; úp ngược trên sàn phẳng thì lực đo được đúng bằng âm lực khi đứng thẳng với mặt cánh nằm ngang ⇒ đảo dấu (không phụ thuộc trục lật hay hướng mũi), tính góc như `attitude_acc_angles` ⇒ in 2 dòng `SET trim_roll/trim_pitch` (kiểu MAGCAL); từ chối khi |a| dao động > 0,3 m/s² hoặc lệch > 10°; chỉ khi disarm. Đo bằng điện thoại trên thân (sau khi đổi dấu quy ước app): IMU lệch thân chỉ +0,08° roll / +0,37° pitch; trim chỉnh khi bay (+0,72 / −0,4) chênh 0,6–0,8° ⇒ phần chênh sinh ra khi bay (lực đẩy/rung/gió). **Đổi liên kết vô tuyến sang HC-05 (người dùng, 2026-11-09)**, cùng USART6: `UART_LINK_BAUD` 460800 → 115200 (người dùng đặt HC-05 `AT+UART=115200,0,0`), `TX_CHUNK_MAX` 200 B (~17 ms, xong trước lần flush 20 ms kế; ~10 KB/s). `link_write_to`/`link_tx_free_to`/`mprintf_to` theo cổng: dòng T qua UART mỗi 3 nhịp (~17 Hz), R 2 Hz, USB giữ 50/10 Hz; DUMP/CAPTURE/ALTRAW/FLOWLOG chỉ gửi về cổng đã ra lệnh (UART 2 dòng/nhịp ≈ 7,5 KB/s). Sketch ESP-NOW giữ lại, không dùng. **Tắt phát hiện bậc ToF (người dùng, 2026-10-29: chỉ bay trên sàn phẳng; lỗi độ cao chủ yếu do bậc giả khi bay):** `alt_step_th` = 0 (mặc định mới, min 0) ⇒ không CONFIRM, mọi khung ToF vào Kalman qua cổng, bị loại liên tục 0,1 s thì đồng bộ lại z (cả khi nghiêng). Người dùng chuyển sang giữ vị trí bằng optical flow (**ngoài phạm vi PLAN §1.1**, người dùng quyết định). **Cần pitch đảo chiều (người dùng, 2026-10-29: bay tự do tiến/lùi bị ngược)** `CFG_RC_PITCH_DIR` +1 → −1. **CH6 đổi (người dùng, 2026-10-29):** gạt lên = Alt Hold, giữa và dưới = Angle; **bỏ Acro** (code Acro xóa khỏi task_ctrl; trường `acro` của dòng T giữ, luôn 0; `mode` 0 Angle / 1 Alt Hold đang giữ). **Hạ cánh bằng cần ga trong Alt Hold** (người dùng yêu cầu): dưới 0,5 m hạ tối đa 0,3 m/s; z_sp xuống tới −0,2 m; phát hiện chạm đất (cần ra lệnh hạ, z < 0,25 m, \|vz\| < 0,1, ga ở giới hạn dưới, liên tục 1 s; ý tưởng ArduPilot land_detector) ⇒ `MSG,ALTHOLD landed`, motor về idle, giữ idle tới khi cần ga < 0,05 (không tự quay lại). Test PC: hạ từ 0,8 m, chậm nhất dưới 0,4 m > −0,4 m/s; không nhận nhầm khi treo thấp 0,35 m. **Chưa làm:** cất cánh bằng Alt Hold (người dùng hỏi; đề xuất sau khi Alt Hold trên không đã thử đạt), failsafe giai đoạn 2 hạ cánh (vẫn disarm như cũ), M10b baro | |
| Giữ vị trí (optical flow) | **Bước 1 code xong, chưa nạp** (2026-10-29; ngoài phạm vi PLAN §1.1, người dùng quyết định; thay M11 la bàn). Đơn vị motion_x/y của MTF-01P **không có tài liệu**: ArduPilot `AP_OpticalFlow_MSP.cpp` chỉ cộng dồn × dt rồi đảo dấu, INAV dùng `opflow_scale` do người dùng hiệu chuẩn ⇒ hiệu chuẩn bằng thử tay. Lệnh `FLOWLOG 0/1` (dòng `F,t_ms,quality,mx,my,p,q,r (mrad/s),roll,pitch (cdeg),tof_mm` mỗi khung flow ~100 Hz), ô tick trong tab Độ cao; `tools/flow_cal.py`: ghép trục flow ↔ gyro, dấu, hệ số k, độ trễ (khi lắc roll/pitch tại chỗ), quãng dịch chuyển (khi trượt tới/phải). Thử trên dữ liệu giả: tìm đúng k −249/+300, trễ 50 ms, quãng 0,49 m. Kế hoạch: (2) ước lượng vận tốc ngang (flow − gyro) × z, Kalman với gia tốc ngang, bù trễ; (3) vị trí (hệ phòng, yaw từ gyro) P → vận tốc PI → góc đặt (giới hạn ~10°), CH6 lên = giữ độ cao + vị trí, chỉ dùng flow khi quality đủ và ToF hợp lệ | |
| Giữ vị trí: hiệu chuẩn flow | **Xong** (2026-11-10, log 19:16 và 19:27, cầm tay, độ cao ~1,05 m, quality 85–89, min 67): lắc ⇒ `motion_x` ↔ roll rate k −5,09 / −4,75, `motion_y` ↔ pitch rate k −4,79 / −4,80 (đơn vị / (rad/s)), trễ 20 ms, r −0,94, trục chéo r < 0,3 ⇒ chốt **k = −4,8, trễ 20 ms**. Trượt có thước 1,2 m (tới, về, phải, về): flow cho 1,16 / 1,15 / 1,09 / 1,24 m (TB −3%) ⇒ k từ bài lắc đúng, không cần hệ số dò tay. Dấu: **v_tới = +h·(my/k − q), v_phải = −h·(mx/k − p)** (hai trục ngược dấu, như quan hệ LOS của ArduPilot EKF: x ↔ +vy/h, y ↔ −vx/h). Cặp trục giống s450drone; hệ số của họ (3,66 dịch chuyển / 1,10 bù xoay) không khớp nhau | |
| Giữ vị trí: bước 2 vận tốc ngang | **Code xong, chưa thử trên mạch** (2026-11-10). `app/flow_est.c` (C thuần, test PC): Kalman 1D mỗi trục trong hệ heading (ngang, theo yaw), dự báo bằng gia tốc ngang (`attitude_acc_horizontal`: R_y(θ)R_x(φ)·f, TB khối 10 chu kỳ cùng a_up trong `g_acc_up`) + quay theo yaw rate, cập nhật bằng vận tốc flow theo công thức hiệu chuẩn (gyro trễ 2 khung, so với ước lượng của 2 khung trước), σ = `flow_sigma` × h, cổng 4σ; chỉ dùng khi quality ≥ `flow_qmin`, ToF hợp lệ, h ≥ 0,2 m, nghiêng < 30°; hợp lệ khi có khung dùng được trong 200 ms; mất > 1 s thì khởi tạo lại từ khung kế. Vị trí tích phân trong hệ phòng (yaw gyro), lệnh `POSZERO`. Tham số `flow_k` −4,8, `flow_scale` 1, `flow_qmin` 50, `flow_sigma` 0,2 (tổng 43). Dòng T thêm vel_fwd, vel_right, pos_x, pos_y, flow_q, flow_valid (**37 trường**, GUI mới bắt buộc). GUI tab "Vị trí": vận tốc theo thời gian + quỹ đạo x/y, nút POSZERO. Test PC: lắc ±3 rad/s tại chỗ ⇒ |v| < 0,15 m/s; trượt tới 1,5 m / phải 0,5 m ⇒ vị trí sai < 0,1 m; quality thấp 1,5 s rồi có lại ⇒ không dùng, khởi tạo lại đúng (mô hình cảm biến đếm nguyên + nhiễu ±0,5 đơn vị). **Thử cầm tay 2026-11-10 (người dùng xác nhận):** lắc tại chỗ ⇒ vị trí lệch ±0,1–0,2 m; lùi ⇒ x âm, sang trái ⇒ y âm (dấu đúng); hai nhánh ~1,5 m (dọc) / ~1,3 m (ngang), quay về điểm xuất phát sai ~0,1–0,2 m sau > 5 m đường đi | |
| Giữ vị trí: bước 3 điều khiển | **Code xong, chưa bay** (2026-11-10). `app/pos_hold.c` (C thuần, test PC): bật khi Alt Hold đang giữ + flow hợp lệ + z ≥ 0,3 m + `pos_p` > 0 (CH6 lên); cần roll/pitch ⇒ vận tốc (tới = −pitch_cmd × `vel_max`, phải = roll_cmd × `vel_max`); thả cần ⇒ phanh (v đặt 0) tới \|v\| < 0,15 m/s rồi thả neo (vị trí hệ phòng); neo ⇒ sai số xoay về hệ heading × `pos_p`, giới hạn 0,5 m/s; vận tốc PI ⇒ góc (pitch = −nghiêng tới, roll = +nghiêng phải), giới hạn `pos_ang_max`, I ≤ 5°, chống tích phân; vào/ra trộn với góc theo cần trong 0,5 s (`attitude_rate_control` nhận góc + trọng số; trim vẫn cộng). Failsafe: cần = 0 ⇒ giữ vị trí. `mode` 2 = giữ độ cao + vị trí; `MSG,POSHOLD on/anchored/off (lý do)`. Tham số `pos_p` 0,8 (0 = tắt), `vel_p` 10°/(m/s), `vel_i` 3°/m, `vel_max` 1 m/s, `pos_ang_max` 12° (tổng 48). Mô hình (trễ nghiêng 0,15 s, trễ flow 30 ms, nhiễu): phanh từ 0,8 m/s neo sau ~0,53 m, nghiêng ≤ 6,3°; gió 0,5 m/s² lệch ≤ 0,27 m, về < 0,1 m sau ~5 s; cần nửa ⇒ 0,5 m/s đúng hướng mũi; xoay yaw 1 rad/s khi giữ ⇒ lệch < 0,15 m. GUI: chế độ "GIỮ ĐỘ CAO + VỊ TRÍ", ô chỉnh nhanh pos_p/vel_p/vel_i | |
| Cất cánh tự động + expo ga | **Code xong, chưa bay** (2026-11-12). Log 20:02 (người dùng: "cất cánh Angle có lúc vọt lên"): ga đứng yên 0,48–0,50, ga treo học được 0,451 (đầu chuyến ~0,40, pin sụt) ⇒ thừa ~11% lực ⇒ tăng tốc đều ~1 m/s², lên 1,45 m/s rồi quán tính tới 1,7 m; không có bước nhảy ga ⇒ giới hạn tốc độ đổi ga không sửa được. (1) Mặc định `hover_thr` 0,45, `thr_mid` 0,45, `thr_expo` 0,4. (2) `alt_hold.c`: CH6 lên + đã arm + z < 0,3 m (và `takeoff_alt` > 0, có ga treo) ⇒ ga giới hạn `ALT_HOLD_GROUND_THR` 0,25 (cả lúc motor đang spool-up); cần ga ≥ 0,5 ⇒ bật giữ độ cao, z_sp leo `vz_max` tới `takeoff_alt` (mặc định 0,8 m, 0 = tắt ⇒ từ chối như cũ), không vượt z + 0,5 m; giữ vị trí tự vào ở 0,3 m; kéo cần dưới vùng giữ ⇒ hủy, hạ, phát hiện chạm đất. `MSG,ALTHOLD take-off …`. Tổng 49 tham số. Test PC: chờ trên sàn với cần 0,4 ⇒ không nhấc; cần 0,6 ⇒ lên 0,8 m (max < 0,9 m, vz max < 0,75 m/s); hủy giữa chừng ⇒ hạ và nhận chạm đất | |
| M10b baro + failsafe hạ cánh + LAND | **Code xong, chưa bay** (2026-11-13). Thử baro tay (motor tắt, log 09:39): nhiễu ±2–3 cm, trôi ~6 cm/40 s, bàn 0,88 m đọc 0,85–0,93 m (ToF chỉ thấy mặt bàn 0,02 m). (A) `alt_est.c`: `baro_ref` = baro − z lấy lần đầu khi ToF dùng được (đứng yên lúc khởi động / lúc arm), học chậm τ 5 s khi ToF dùng được, z > 0,5 m, không chờ bậc, h = 0; ToF vắng > 100 ms và z > 0,5 m ⇒ Kalman cập nhật bằng baro − baro_ref (σ `alt_sigma_baro` 0,3, cổng 4σ), `src` = baro, hợp lệ tối đa 3 s (20 s khi đang hạ cánh); không bao giờ dưới 0,5 m; ToF quay lại sau quãng baro ⇒ không khởi tạo lại; bậc phải vượt cả `alt_step_th` lẫn 3σ của z (không ghi sai số sau quãng mất ToF vào h_terrain; replay_kf.py cập nhật, baro không replay). (B) Failsafe giai đoạn 2: nếu motor đang chạy và độ cao hợp lệ ⇒ không disarm mà hạ cánh: `alt_hold` nhận `land` ⇒ bật ở mọi độ cao, hạ 0,3 m/s bất kể cần, giữ vị trí nếu flow hợp lệ (cần = 0), chạm đất ⇒ `DISARM_LANDED`; hạ cánh phải thoát (mất độ cao / lệch > 1 m) ⇒ không bật lại, failsafe ⇒ disarm (§7.7); idle trên sàn ⇒ disarm như cũ. (C) Lệnh `LAND` + nút "HẠ CÁNH" (GUI): chỉ khi đang bay và độ cao hợp lệ; hủy khi gạt CH6, disarm, hoặc hạ cánh thất bại; `mode` 3 = hạ cánh. Tổng 50 tham số. Test PC: baro nối 2,8 s mất ToF khi đang hạ ⇒ z sai ≤ 0,25 m, vẫn hợp lệ, ToF quay lại không khởi tạo lại; không dùng baro dưới 0,5 m; hạ cánh bắt buộc từ 1 m với cần 0,9 ⇒ hạ ≤ 0,4 m/s, chạm đất; mất độ cao giữa chừng ⇒ không bật lại; arming: giai đoạn 2 + land_ok ⇒ không disarm, land_done ⇒ disarm | |
| G1 GPS (ngoài PLAN, người dùng quyết định 2026-11-14) | **Code xong, chưa thử trên mạch** (2026-11-14). GEP-M10DI (u-blox M10, la bàn IST8310 + baro DPS310 trên module, đã dùng qua I2C1); GPS nối như code cũ: USART3 PC10 TX → GPS RX, PC11 RX ← GPS TX, 115200. `drivers/ubx.c` (khung UBX, NAV-PVT 92 B, CFG-VALSET; hằng số/key theo PX4-GPSDrivers BSD, key NAV-PVT UART1 = I2C + 1 như cfgValsetPort), `bsp/gps_uart.c` (DMA1_Stream1 kênh 4 vòng, VERIFY RM0090 bảng 42). `task_slow`: 1 s sau khởi động và mỗi 2 s khi chưa có NAV-PVT (chỉ khi disarm) gửi VALSET lớp RAM: UBX bật, NAV-PVT UART1 = 1, RATE_MEAS 100 ms, NMEA tắt; `g_gps`; MSG khi đổi trạng thái (không byte / có byte không NAV-PVT / chưa fix / 3D fix). Dòng `G` 5 Hz (radio 1 Hz). GUI tab GPS (trạng thái, sai số, vận tốc NED, quỹ đạo Bắc/Đông theo mét). Test `test_ubx` (parser với NMEA/checksum sai/quá dài, NAV-PVT, VALSET byte-by-byte). Kế hoạch tiếp: G2 la bàn (nhiễu motor, MAGCAL, bù nghiêng, trộn yaw), G3 ước lượng + `nav_mode`, G4 giữ vị trí GPS. | |
| Dời cảm biến flow + `flow_rot` | **Code xong, chưa bay** (2026-11-14). Calib tay 3 lần sau khi dời: 16:51 xoay 90° (chất lượng TV 60, k −6,06/−5,49 ở 0,54 m), 17:16 hướng cũ (chất lượng 40, 83% khung < 50: bỏ), **17:27 xoay 180°** (giữ cố định: chất lượng 68, 0% < 50, k −4,46/−4,27 ở 0,9 m, trượt tới/phải ~+1,05 m đúng dấu). Gốc 10/11: chất lượng 87. Tham số mới `flow_rot` (0/90/180/270, mặc định 180, chỉ đổi khi disarm): đếm flow quay về hướng lắp gốc trước công thức vận tốc (`flow_est.c`); `flow_k` mặc định −4,4. `flow_cal.py` in luôn `SET flow_rot … SET flow_k …` và quãng trượt theo đúng công thức firmware. Test `test_mounts` (4 hướng + trường hợp đặt sai). | |
| Chạm đất mới + bù trôi sát đất | **Failsafe tự hạ cánh: bay thử ĐẠT** (2026-11-13): mất RC → LAND từ 0,8 m → giữ vị trí tắt dưới 0,2 m → chạm đất z 16 mm → DISARMED landed. Gợi ý từ lần bay: trim_roll 0,40 / trim_pitch −1,48 (18 s treo), hover_thr 0,454. Chưa thử: hạ bằng cần ga, nút LAND. Code (2026-11-13): Log 20:02 (10/11): nằm trên sàn 2,1 s mà không nhận chạm đất: vz nảy ±0,19 m/s (ngưỡng 0,1), ga chỉ trôi 0,44 → 0,34 (cần ≤ 0,28, bộ I ~0,03/s). (2) Bộ mới (`alt_hold.c`): có thể đã chạm đất = đang đòi hạ + z < z_sàn (ToF lúc arm, `alt_out.z_floor`) + 0,05 + z_sp < z − 0,1 + \|vz lọc 0,2 s\| < 0,15 ⇒ ga giảm 0,5/s về idle ngay, tắt điều khiển ngang; giữ 1 s ⇒ chạm đất. Chạy lại trên log đó: "có thể" 0,13 s sau chạm, xác nhận sau ~1 s (cũ: không bao giờ). Hạ cánh mà mất độ cao dưới 0,5 m ⇒ hạ mù 0,9 × ga gốc 3 s rồi coi là chạm đất (ý người dùng). (4.1) `POS_HOLD_MIN_Z` 0,3 → 0,2 m (flow hợp lệ tới ~0,2 m). (4.2) Bộ I vòng vận tốc ngang không xoá khi tắt giữ vị trí; dưới ngưỡng flow / lúc cất cánh (giữ độ cao đang bật, chưa chạm đất) cộng cố định vào góc đặt (`add_roll/pitch`, tổng phần I không đổi khi trộn ra), giữ qua các lần bay (dùng lại khi cất cánh); tắt khi có thể đã chạm đất. Log 20:02: dưới 0,3 m drone trôi lùi 0,15–0,2 m/s; vận tốc chỉ từ gia tốc trên sàn trôi ~0,3 m/s mỗi giây ⇒ không dùng gia tốc sát đất (4.4 để sau). (4.3) Khi giữ vị trí neo và \|v\| < 0,1 m/s: TB bộ I (τ 2 s); ≥ 5 s ⇒ lúc tắt in `MSG,POSHOLD trim learnt … SET trim_roll … / SET trim_pitch …` | |
| Rung sau thay phần cứng + bias gia tốc trong `flow_est` | **Code xong, chưa bay** (2026-11-16). Rung: capture 14/11 đỉnh 55 Hz roll (D) → người dùng sửa cơ khí + giảm D (0.0007) hết 55 Hz; còn ~6 Hz cả roll/pitch (đo/đặt ×4,3) → `rate_p_rp` 0.21 → 0.17 đỡ rõ; capture 11:39 có thêm đỉnh 44 Hz pitch. Module MTF-01 mới đo ToF thiếu ~0,4–0,5 m → quay về module cũ. FLOWLOG tay 11:33 (qua HC-05, không rớt khung): rot 180, k −4,62, trượt 1,2 m đọc 1,05/1,10 ⇒ `flow_scale` ~1,12. Trôi phải khi giữ vị trí: FLOWLOG lúc bay 11:49 ⇒ ước lượng = 0,76×flow − 0,096 m/s ở trục phải (tương quan 0,98), trục trước +0,01 ⇒ bias gia tốc ngang ~0,6 m/s² (~3,5°) bị bộ lọc chỉ-vận-tốc biến thành trôi đều. Sửa: `flow_est` thêm trạng thái bias gia tốc mỗi trục (KF 2 trạng thái v, b; σ_b 0,05 m/s²/√s, b0 0,5, kẹp ±1,5), giữ bias khi reseed; `flow_out_t.acc_bias`; MSG `FLOW accel bias learnt` khi POSHOLD off. Test PC `test_acc_bias` (bias −0,3/+0,6 ⇒ v trung bình < 0,015 m/s, b đúng ±0,1); `test_mounts` ca sai flow_rot chấm theo trung bình 1 chu kỳ. Build 0 warning, FLASH 97752 B. Nguyên nhân gốc phần cứng (rung/IMU sau khi thay) chưa xác định. Bay thử 12:10: v_right trung bình −0,09 → −0,03 m/s, bias học −1,5…−4,1° (dao động theo lắc roll 0,2–0,3 Hz, lắc này đã có ở log 11:49 trước khi đổi firmware); 7 lần mất flow (chất lượng TB 56, 6% < 50); phát hiện bậc báo sai giữa không (h_terrain −197 mm). ⇒ `FLOW_SIGMA_B` 0,05 → 0,005 (hằng thời gian bias ~2 s → ~20 s), đề nghị `flow_qmin 40`, `alt_step_th 0`. Bay 12:27/12:31 (vel_i 1,5): bias ổn định −3,1°, v_right trung bình 0,00; còn lắc vị trí trục phải ±35 cm, chu kỳ ~4,7 s (đang tìm nguyên nhân). **Gai ToF** (log 12:24–12:31: +9…+25 cm, < 60 ms, vài lần/phút; cổng 3σ ~16 cm nên phần lớn lọt qua, mỗi gai đẩy z +1–2 cm): `alt_est` giữ lại frame lệch > `ALT_SPIKE_M` 6 cm so với z dự đoán, tối đa `ALT_SPIKE_N` 8 frame (~80 ms); lệch lâu hơn thì là thay đổi thật, đi qua Kalman như cũ. `tools/replay_kf.py` cập nhật theo. Test `test_short_spikes_flat_floor` (step_th 0: gai 1–5 frame 12/25 cm ⇒ z lệch < 1,2 cm; thay đổi 10 cm kéo dài vẫn theo; bỏ phần giữ thì test fail 2,1 cm). Build 0 warning, FLASH 97804 B. Bay 14:19 (firmware có lọc gai ToF, FLOWLOG + capture): z−z_sp std 2,1 cm (trước 2,7–5,1), gai ToF vẫn có (26 lần) nhưng z gần như không theo; giữ vị trí đoạn 63 s: lệch khỏi điểm neo RMS 6,4 cm, tối đa 16 cm (12:31: 21 / 39 cm); vận tốc ước lượng chỉ trễ ~50 ms so với flow thô (bộ lọc không gây trễ đáng kể). Capture: hết đỉnh 44 Hz pitch; còn dao động 5,4 Hz ở cả roll và pitch (p/p_sp ≈ 3, ga 0,45 với pin mới). **Sửa `flow_est`**: dưới đất (không có frame dùng được > `FLOW_RESEED_MS`) v và pos vẫn tích phân theo gia tốc ⇒ log 14:11 v −11 m/s, pos −264/−412 m trước khi cất cánh; nay giữ v = 0 và đóng băng pos (`coast_s`), frame tốt đầu tiên khởi động lại như cũ. Test `test_floor_no_drift`. Build 0 warning, FLASH 97848 B. Bay 14:35 (trim 0,44/−2,26; drone vừa khởi động nên điểm neo đầu x 12 / y −8 cm — KHÔNG chứng minh bản sửa trôi dưới đất: log 14:35 và 14:47 sau khi hạ cánh v vẫn tăng, pos vẫn trôi ⇒ drone chưa nạp bản sửa); giữ vị trí 4 đoạn 13–46 s: RMS 3–9 cm, tối đa 25 cm; flow thô và ước lượng trung bình ≈ 0 cả hai trục (hết trôi); bias gia tốc học được ~0° (fwd 0,2°, phải −0,1°) ⇒ lệch −3° trước đây là do trim/trọng tâm chứ không phải IMU; firmware đề xuất trim 0,15 / −1,09, hover_thr 0,482; z−z_sp std 2,8 cm, gai ToF 11 lần. Lần 14:35 bay với `rate_p_rp 0.15` (14:19: 0,17): trên T line (16 Hz) dải 4,5–6,5 Hz của p 4,05 → 2,44 °/s, q 3,07 → 1,65 °/s, góc roll/pitch ở dải đó 0,13/0,15 → 0,06/0,05° (ga 0,47 vs 0,45) ⇒ giữ 0,15. Bay dài 14:47 (~4 phút, rate_p_rp 0,15, 3 capture + FLOWLOG): 9 đoạn giữ vị trí 6–39 s, RMS 4,6–8,7 cm, tối đa 17 cm, không xấu đi theo thời gian; trong các đoạn neo flow thô trung bình |v| ≤ 0,02 m/s, khớp ước lượng (không trôi bền); trim học 0,01/−1,15 (14:35: 0,15/−1,09), hover_thr 0,493, bias ~0; z−z_sp std 2,4 cm. Capture 5 Hz: p 3,55/2,35/3,75, q 2,58/1,10/1,29 (14:21 P 0,17: 4,59/4,37); roll vẫn ~3× setpoint ở 5 Hz. Điểm neo đầu tiên x −4788 m: trôi dưới đất giữa hai chuyến (firmware cũ). Bay 15:07 (đã nạp bản sửa): trước cất cánh và sau hạ cánh v = 0, pos đứng yên ⇒ bản sửa trôi dưới đất chạy đúng; lần cất cánh đầu ở Angle bằng ga tay (lên 1,39 m rồi rơi, IMU clip 78, ALT resync 731 mm) — lỗi thao tác. Giữ vị trí RMS 5,6–9,7 cm; trim −0,06/−0,89, hover_thr 0,501. **Lắc sau khi thả cần** (±15 cm, chu kỳ ~4 s, 2–3 lần): mô hình fit từ 3 log (dv/dt theo góc đo và v) cho "lực cản" ÂM +0,65…+1,08 /s ⇒ góc ước lượng bị gia tốc kế kéo về 0 khi drone nghiêng để tăng tốc (multirotor: gia tốc kế thấy lực đẩy dọc thân), hằng thời gian = kf_sacc/kf_sg = 3/4 = 0,75 s; sai số góc tỉ lệ v ⇒ mất phần lớn giảm chấn của vòng vận tốc. Mô phỏng (scratchpad relsim.py) với τ 0,75 tái tạo vượt 17 cm, 2 lần qua lại; τ 3 s: 9 cm, không qua lại. Đề nghị thử `kf_sacc 12` (τ 3 s). **Phát hiện bậc không bắt được bậc thật** (16:02, cầm tay qua vật ~28 cm, `alt_step_th` bật): ToF đổi dần 0,95 → 0,67 m trong ~0,1 s (~3 cm/frame, vệt tia) ⇒ không frame nào nhảy > ngưỡng ⇒ 13 lần "z restarted from ToF", h_terrain = 0. Sửa `alt_est`: chuỗi thay đổi nhanh (frame bắt đầu: cả ToF lẫn ToF − vz·dt > `ALT_STEP_RATE` 1 m/s cùng chiều; tiếp tục khi 3 frame gần nhất còn ≥ 0,5 lần tốc độ đó) cộng dồn, tổng > `alt_step_th` ⇒ CONFIRM; khi chuỗi còn chạy, ToF bị giữ (khi bật bậc) và CONFIRM chỉ đếm 3 frame ổn định sau khi chuỗi dừng. Thử so innovation trực tiếp/cửa sổ 250 ms: hỏng test bias gia tốc (vz lệch ~1 m/s khi ToF phẳng ⇒ bậc giả tới −23 m) nên bỏ. `alt_step_th 0`: đường xử lý y như cũ. `replay_kf.py` cập nhật. Test `test_gradual_edge` (bậc 28 cm dốc 50–200 ms, lên và xuống, ngưỡng 0,15). Build 0 warning, FLASH 98136 B. |
| M11–M12 | Chưa bắt đầu | |

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
- Người dùng đã có **giá thử gimbal 3 trục** (roll/pitch/yaw quay tự do). ESC xác định là Hobbywing Skywalker V2 40A (nhà bán ghi 50–432 Hz).

### ACRO + ESC 400 Hz (người dùng yêu cầu, 2026-10-26; code xong, chưa nạp)
- **ESC 400 Hz** (`ESC_PWM_HZ` = 400, chu kỳ 2500 µs, còn 500 µs mức thấp sau xung 2000 µs). `_Static_assert` trong `pwm_esc.c`: chu kỳ ≥ `ESC_MAX_US` + 300 µs (chặn 500 Hz). Lần "không cất cánh ở 400 Hz" trước trùng P 0,4 / D 0,007 nên chưa kết luận được về ESC.
- **ACRO trên CH6 "dưới"** (`CFG_RC_MODE_ACRO`): cần roll/pitch ⇒ `rate_sp = cần × max_rate_rp` (200°/s) đưa thẳng vào vòng rate (giống "rate mode" của Carbon Aeronautics); yaw như cũ. Không tự cân bằng. Failsafe / mất RC luôn quay về Angle. Trong ACRO tắt phát hiện mất kiểm soát (không có góc đặt); phát hiện lật 70°/0,3 s vẫn hoạt động. CH6 giữa (Alt Hold, chưa có) chạy Angle.
- Telemetry: dòng `T` thêm 3 trường cuối `p_sp, q_sp` (°/s, setpoint vòng rate ở cả hai chế độ) và `acro` ⇒ 30 trường (lệch §9.1). GUI: đồ thị rate p/q/r đo (liền) vs đặt (đứt), checkbox ẩn từng trục; nhãn "ARMED ACRO". Lưu ý telemetry 50 Hz nên đồ thị rate chỉ thấy đáp ứng chậm (< 25 Hz).
- Test PC (dòng T 30 trường) đạt; build 0 warning.

### Liên kết vô tuyến ESP-NOW thay HC-05 (người dùng yêu cầu, 2026-10-27; code xong, chưa thử trên mạch)
- Người dùng dùng 2 ESP32-C3 Super Mini (ESP-NOW) thay HC-05. Sketch người dùng gửi (`txespnow.ino`, `RXespnow.ino` ở thư mục gốc) không dùng được: chỉ nhận JSON (drone gửi dòng văn bản `T,...`), một chiều (mất DISARM/SET), 2 Hz, chân GPIO17/18 (flash/USB trên C3). Đã viết lại.
- `esp/espnow_bridge/espnow_bridge.ino`: một sketch cho cả 2 board, vai trò chọn theo MAC (drone AC:A7:04:BB:71:0C, GCS AC:A7:04:BB:1F:30). Cầu nối trong suốt hai chiều: UART 460800 (GPIO20 RX ← PC6, GPIO21 TX → PC7) ↔ ESP-NOW ↔ USB. Gói ≤ 250 B (header magic + seq), gom theo khoảng lặng 1 ms / tối đa 10 ms, thử lại 3 lần, bỏ gói trùng theo seq; báo `MSG,LINK ...` (mất/trùng/bỏ gói, RSSI) ở ranh giới dòng khi có lỗi. Kênh 1, 8,5 dBm, tắt power save. Biên dịch sạch với core 2.0.17 (`CDCOnBoot=cdc`).
- Firmware: `bsp/uart_link.c` (USART6 460800 đặt lúc chạy, `.ioc` vẫn 9600; TX ring 4 KB → DMA2_Stream6 từng khối 768 B; RX bằng ngắt RXNE, đọc DR trước `HAL_UART_IRQHandler`). `app/link.c`: mọi đầu ra ghi vào USB (khi có DTR) và UART; lệnh nhận từ cả hai (bộ đệm dòng riêng). Lệnh mới `HELLO` (in lại banner + trạng thái hiệu chuẩn). Build 0 warning; RAM +5 KB.
- GUI: mở cổng với DTR bật, RTS tắt (RTS có thể reset C3); khi kết nối gửi `HELLO` + `LIST`; nhãn "MẤT DỮ LIỆU" khi > 0,5 s không có dòng T.
- **Ghi tần số cao (CAPTURE)** (người dùng duyệt, 2026-10-27; code xong, chưa thử trên mạch): `app/capture.c` (C thuần, test PC) ghi mỗi 2 chu kỳ task_ctrl (~530 Hz) 1280 bản ghi × 40 B (~2,4 s, 51 KB SRAM ⇒ RAM 83,8 %): p/q/r đo, p/q/r đặt, khâu P/I/D roll & pitch, P/I yaw (`pid_ctrl_t` thêm `out_p`, `out_d`; FF không ghi), ga, 4 motor µs. Lệnh `CAPTURE` → `OK,CAPTURE,started`; ghi đầy ⇒ `task_telem` gửi `MSG,CAPTURE sending` + dòng `C,...` (tạm dừng dòng T như DUMP) + `OK,CAPTURE,done`, rồi giải phóng bộ đệm. Không ảnh hưởng điều khiển (chỉ đọc).
- GUI: nút "Ghi tần số cao" + "Mở capture cũ"; lưu `gcs/logs/capture_*.csv`; cửa sổ phân tích: chọn trục, đồ thị đặt/đo, khâu P/I/D + tổng, motor (vạch 2000 µs), phổ FFT (log) của rate đo và sai số; chỉ số mỗi trục: RMS sai số, trễ (tương quan chéo ≤ 100 ms), đỉnh đo/đặt %, đỉnh dao động mạnh nhất ≥ 15 Hz. Thử với dữ liệu giả (trễ 20 ms + dao động 45 Hz): phát hiện đúng 45 Hz.
- Người dùng đã sửa được kết nối IMU và khôi phục tasks.c/log.c/log.h/uart_link.c về bản trước đợt gỡ lỗi IMU (bỏ IMUDIAG, init thử lại, lọc byte rác, pull-up PC7, sửa cắt dòng mprintf).
- CAPTURE qua ESP-NOW lần đầu (2026-10-27): 1280 dòng gửi 40 dòng/tick (~150 KB/s vào UART) ⇒ ESP drone `tx dropped 545 overflow 123`, GCS `rx lost 293 dup 162`; 1067 dòng tới nơi, 26 dòng hỏng ⇒ GUI không mở được. Sửa: DUMP/CAPTURE 6 dòng/tick (~22 KB/s, capture ~4,3 s); dòng `C` thêm số thứ tự mẫu `i` đầu dòng; ESP `MAX_TRIES` 10; GUI bỏ dòng hỏng, bỏ dòng trùng, nội suy mẫu mất theo `i`, tự thêm tiêu đề nếu mất, báo số mẫu mất/dòng hỏng. Thử: file hỏng thật mở được; dữ liệu giả mất 15 % + 2 % dòng cụt vẫn tìm đúng 45 Hz.
- Người dùng xác nhận (2026-10-27): CAPTURE qua ESP-NOW đã ổn sau khi giảm tốc độ gửi.
- 2026-10-28 "CAPTURE đôi khi không có phản hồi": log GCS có dòng `CAPTURE 4T,97490,...` ⇒ drone echo lệnh nhưng thiếu `\n` (byte mất trên USART6 RX: ngắt RXNE ở 460800 baud, 1 byte/22 µs, bị ISR khác chặn lâu hơn ⇒ ghi đè). Sửa: USART6 RX dùng DMA2 Stream1 kênh 5 vòng tròn 512 B (cấu hình thanh ghi trong `uart_link_init`, `// VERIFY: RM0090 Table 43`), đọc theo NDTR; `uart_link_irq` để trống. GUI: mỗi lệnh gửi kèm `\n` đầu (xóa dòng dở trong bộ đệm lệnh drone); báo nếu 1,5 s không có `OK/ERR,CAPTURE`.
- CAPTURE dài hơn (người dùng: 2,4 s quá ngắn): RAM không đủ thêm bản ghi (83,8 %), nên `CAPTURE [n]` lấy **trung bình** n chu kỳ task_ctrl mỗi bản ghi (n 1–8, mặc định 4 ≈ 265 Hz, 4,8 s); lấy trung bình thay vì bỏ mẫu để rung nhanh không gập xuống thành dao động chậm giả. `capture_add(cap_in_t float, dt)` thay `capture_due/store`. GUI có ô chọn 2,4 / 3,6 / 4,8 / 7,2 s. Test PC (trung bình, dt, thứ tự, định dạng) đạt; build 0 warning.
- **Tune vòng rate trên giá gimbal (2026-10-28, chế độ Angle, ang_p_rp 2, ga ~0,34):** P 0,42 + D 0,002 ⇒ motor giật 1000↔2000 µs ở ~75 Hz (do hệ số). Chỉnh trọng tâm trên giá (pitch kẹt ~9°), trim yaw (CH4 ~1070), `gyro_lpf_hz` 150→90, `dterm_lpf_hz` 80→50, `rate_i_rp` 0,1, yaw P 0,07 / I 0,05. Diễn tiến roll (trễ / đỉnh làm mượt): P 0,15 → 98 ms / ~106 %; P 0,18 → 83 ms / ~137 %; P 0,19 D 0,0005 → 75 ms / 113 %; P 0,25 D 0,0012 → 68 ms / 111 % nhưng motor chạm ~1013 µs, yaw bị cắt. **Bộ người dùng chọn: P 0,24, D 0,00112** → 75 ms / 110 %, motor σ ~95 µs (min 1005). Rung cơ khí ~75–90 Hz tăng theo ga (cánh/motor) là giới hạn chính; trễ còn lại chủ yếu do motor/cánh. Còn lại: xác nhận pitch, SAVE, chỉnh ang_p_rp. GUI: đỉnh đo/đặt tính trên tín hiệu làm mượt 30 ms.
- Sau khi siết ốc trục cánh (ga 0,525): rung roll 92 Hz 6,8°/s, D roll σ 0,08, motor 1033–1980 µs ⇒ chưa cải thiện. **Notch gyro theo motor** (người dùng duyệt, 2026-10-28; code xong, chưa thử): `biquad_notch_*` (RBJ) trong `lib/filter`, đặt trước LPF gyro trong ISR IMU; tâm = `notch_lo_hz` + (`notch_hi_hz` − `notch_lo_hz`) × trung bình lệnh 4 motor, cập nhật ~100 Hz khi đổi ≥ 0,5 Hz (giữ trạng thái lọc); tham số mới `notch_lo_hz` 55 (0 = tắt), `notch_hi_hz` 120, `notch_q` 2,5 (tổng 22). Đáp ứng tại 85 Hz: 10 Hz −0,0 dB / trễ 0,76 ms; 75 & 95 Hz −5,5 / −6,2 dB. Test PC (suy giảm tâm < −30 dB, mép −3 dB, 10 Hz, đổi tâm 2 Hz) đạt; build 0 warning.
- GUI chỉnh vòng rate: khung chỉnh nhanh 6 hệ số trên tab Bay (Enter = SET, ô vàng = chưa gửi), nút tạm dừng đồ thị (zoom/kéo trục thời gian), tự co trục Y cho cả đồ thị rate, sai số bám RMS 5 s cho p/q/r (chỉ lúc ARMED và ga > 5 %), bảng tay cầm ẩn mặc định.

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
