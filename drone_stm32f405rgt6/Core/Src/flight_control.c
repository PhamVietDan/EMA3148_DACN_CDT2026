/*
 * flight_control.c
 *
 *  Created on: Aug 13, 2026
 *      Author: ADMIN
 */
#include "flight_control.h"
#include "tim.h"
#include "rc_sbus.h"    // Lấy throttle_pwm, rc_channel, map()
#include "sensor_imu.h" // Lấy pitch, roll, gy_dps, gx_dps, gz_dps

// Gọi biến góc yaw tuyệt đối từ sensor_imu.c sang
extern float yaw;

// BỔ SUNG: Gọi 3 biến PWM đã được chuẩn hóa (1000-2000) từ rc_sbus.c sang
extern uint16_t roll_pwm;
extern uint16_t pitch_pwm;
extern uint16_t yaw_pwm;

// Khai báo hệ số PID
// GIÁ TRỊ KHỞI ĐIỂM (chưa bay thật) cho khung F450 cỡ ~1.2-1.5kg, motor ~900-1000KV,
// props 10x4.5. Đây CHỈ LÀ ĐIỂM XUẤT PHÁT AN TOÀN ĐỂ TEST, không phải giá trị cuối.
// Quy trình tune bắt buộc trước khi bay thật:
//   1) Tháo cánh quạt, cầm tay nghiêng máy, xem out_m1..m4 phản ứng đúng chiều chưa.
//   2) Lắp cánh, buộc dây an toàn/đứng trên giá đỡ, tăng ga nhẹ, tăng dần Kp cho tới
//      khi thấy dao động (rung lắc nhanh) thì giảm Kp xuống ~70% giá trị đó.
//   3) Thêm Kd từ từ để dập dao động còn sót (nghe/nhìn cánh tay đòn rung là dấu hiệu
//      Kd đang thấp hoặc lọc gyro chưa đủ).
//   4) Thêm Ki cuối cùng, chỉ đủ để triệt tiêu lệch góc tĩnh (drone không tự nghiêng
//      khi giữ yên); Ki quá lớn gây dao động chậm ("bồng bềnh").
//   5) Test bay tay ở độ cao thấp, có người sẵn sàng cắt ga khẩn cấp.
float Kp_pitch = 8.0f, Ki_pitch = 0.0f, Kd_pitch = 3.0f;
float Kp_roll  = 8.0f, Ki_roll  = 0.0f, Kd_roll  = 3.0f;
float Kp_yaw   = 0.0f, Ki_yaw   = 0.0f, Kd_yaw   = 0.0f; // Vòng trong: rate (deg/s)

// Vòng ngoài GIỮ HƯỚNG (heading-hold): chạy trên góc yaw tuyệt đối (đã fuse la bàn
// trong sensor_imu.c), CHỈ được kích hoạt khi cần yaw ở giữa (đã về deadband).
// Output của vòng này là 1 "yaw_rate mong muốn" nạp ngược vào vòng trong Kp_yaw/Ki_yaw
// phía trên, KHÔNG thay thế nó. Vì vậy PHẢI tune xong Kp_yaw/Ki_yaw (vòng trong) rồi
// mới bật/tune Kp_yaw_hold (vòng ngoài) - tune ngược thứ tự sẽ không hội tụ được.
// Bắt đầu với Kp_yaw_hold nhỏ và Ki_yaw_hold = 0, chỉ thêm Ki nếu sau khi tune Kp vẫn
// còn lệch hướng tĩnh không tự triệt tiêu (thường do la bàn calib chưa tốt).
float Kp_yaw_hold = 3.0f, Ki_yaw_hold = 0.0f;
#define YAW_HOLD_MAX_RATE 60.0f // Giới hạn tốc độ yaw (deg/s) mà vòng giữ hướng được phép đòi, chống lồng nếu la bàn nhiễu đột ngột

int16_t out_m1 = 1000, out_m2 = 1000, out_m3 = 1000, out_m4 = 1000;

// Các biến phục vụ tính toán PID
static float integral_pitch = 0.0f;
static float integral_roll = 0.0f;
static float integral_yaw = 0.0f;

// Chiều bão hòa của output PID ở vòng lặp trước: +1 = kẹt trần trên,
// -1 = kẹt trần dưới, 0 = không bão hòa. Dùng để chống windup thật sự
// (dừng cộng dồn tích phân cùng chiều đang kẹt).
static int8_t pitch_saturated = 0;
static int8_t roll_saturated = 0;
static int8_t yaw_saturated = 0;

static uint32_t last_pid_time = 0;

// Trạng thái vòng giữ hướng (heading-hold)
static float target_yaw = 0.0f;          // Hướng đang bị "khóa" khi cần yaw ở giữa
static float integral_yaw_hold = 0.0f;   // Tích phân sai số hướng (chỉ dùng nếu Ki_yaw_hold > 0)

/*=============================================================================
 * HÀM KHỞI ĐỘNG VÀ CALIBRATE ESC BẰNG TAY CẦM
 *===========================================================================*/
/* =========================================================================
 * HÀM KHỞI ĐỘNG VÀ CALIBRATE ESC (CÓ CHỐNG NHIỄU SBUS & KHÓA AN TOÀN)
 * =========================================================================*/
void Motor_Init_And_Calibrate_By_RC(void) {
    // 1. ÉP XUNG VỀ 0V ĐỂ CHỜ ESC
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

    // 2. CHỜ TÍN HIỆU SBUS ỔN ĐỊNH THẬT SỰ (Debounce 200ms)
    uint8_t valid_sbus_count = 0;
    while (valid_sbus_count < 20) {
        if (throttle_pwm > 800 && throttle_pwm < 2200) {
            valid_sbus_count++; // Tăng biến đếm nếu tín hiệu tốt
        } else {
            valid_sbus_count = 0; // Reset nếu có niễu
        }
        HAL_Delay(10);
    }

    // 3. ĐỌC CẦN GA VÀ QUYẾT ĐỊNH
    if (throttle_pwm > 1800) {
        // ================= CALIBRATE MODE =================
        // Bắn xung 2000us ra. ESC sẽ kêu bíp bíp.
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 2000);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 2000);

        // Vòng lặp chờ bạn kéo ga xuống MIN (Chống nhiễu!)
        uint8_t min_count = 0;
        while (1) {
            // Ga phải nằm dưới 1200 và trên 800 (tức là mức MIN hợp lệ)
            if (throttle_pwm < 1200 && throttle_pwm > 800) {
                min_count++;
                if (min_count > 30) break; // Phải giữ MIN liên tục 300ms mới thoát!
            } else {
                min_count = 0; // Nếu bị nhiễu văng lên, reset đếm lại từ đầu
            }
            HAL_Delay(10);
        }

        // Bạn đã kéo ga xuống MIN thật sự. Chốt xung 1000us.
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 900);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 900);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 900);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 900);

        HAL_Delay(4000); // Chờ ESC kêu tít tít hoàn thành
    }
    else {
        // ================= NORMAL MODE =================
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 900);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 900);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 900);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 900);

        HAL_Delay(4000);
    }

    // ====================================================================
    // 4. KHÓA AN TOÀN KÉP (Tuyệt đối không cho bay nếu chưa hạ ga)
    // ====================================================================
    while (throttle_pwm > 1200) {
        // Nếu vì lý do gì đó ga vẫn cao, giam code ở đây và ép xung 1000us
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 900);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 900);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 900);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 900);
        HAL_Delay(10);
    }

    // 5. Khởi động Timer PID
    HAL_TIM_Base_Start(&htim2);
    last_pid_time = __HAL_TIM_GET_COUNTER(&htim2);
}

void FlightControl_Update_PID(void) {
    // ===================================================================
    // 1. Tính toán thời gian dt bằng micro-giây (Dùng TIM2)
    // ===================================================================
    uint32_t current_time = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t dt_ticks = current_time - last_pid_time;
    float dt = dt_ticks / 1000000.0f; // Chuyển us sang giây
    last_pid_time = current_time;

    if (dt < 0.0005f) dt = 0.0005f;
    if (dt > 0.01f) dt = 0.01f;

    // ===================================================================
    // 2. Chuyển đổi tín hiệu tay cầm (RC) sang Setpoint (ĐÃ SỬA DÙNG PWM CHUẨN)
    // ===================================================================
    // Dải PWM chuẩn từ rc_sbus.c là 1000 (Min) đến 2000 (Max).
    float setpoint_roll  = (float)map(roll_pwm, 1000, 2000, -30, 30);

    // Giữ nguyên logic đảo chiều của Pitch và Yaw như code cũ của bạn
    // (1000 -> dương, 2000 -> âm)
    float setpoint_pitch = (float)map(pitch_pwm, 1000, 2000, 30, -30);
    float yaw_rate_cmd   = (float)map(yaw_pwm, 1000, 2000, 100, -100);

    // Deadband lọc nhiễu điểm giữa
    if (setpoint_roll > -2.0f && setpoint_roll < 2.0f) setpoint_roll = 0.0f;
    if (setpoint_pitch > -2.0f && setpoint_pitch < 2.0f) setpoint_pitch = 0.0f;
    if (yaw_rate_cmd > -15.0f && yaw_rate_cmd < 15.0f) yaw_rate_cmd = 0.0f;

    // ===================================================================
    // 3. VÒNG NGOÀI GIỮ HƯỚNG (Heading-hold)
    // ===================================================================
    if (yaw_rate_cmd != 0.0f || throttle_pwm <= 1050) {
        // Người dùng đang chủ động xoay yaw, hoặc chưa cất cánh: liên tục cập
        // nhật hướng khóa theo hướng hiện tại. Nhờ vậy lúc thả cần yaw về giữa,
        // hướng khóa = hướng vừa xoay tới -> không bị giật lại do sai số dồn.
        target_yaw = yaw;
        integral_yaw_hold = 0.0f;
    } else {
        // Cần yaw đã về giữa: giữ nguyên target_yaw, tính sai số hướng có xử lý
        // vòng tròn (wrap-around qua mốc 180/-180), giống cách fuse la bàn.
        float heading_error = target_yaw - yaw;
        if (heading_error > 180.0f) heading_error -= 360.0f;
        else if (heading_error < -180.0f) heading_error += 360.0f;

        integral_yaw_hold += heading_error * dt;
        if (integral_yaw_hold > 50.0f) integral_yaw_hold = 50.0f;
        else if (integral_yaw_hold < -50.0f) integral_yaw_hold = -50.0f;

        // Output vòng ngoài = yaw_rate mong muốn để đưa vào vòng trong (rate PID)
        yaw_rate_cmd = (Kp_yaw_hold * heading_error) + (Ki_yaw_hold * integral_yaw_hold);

        if (yaw_rate_cmd > YAW_HOLD_MAX_RATE) yaw_rate_cmd = YAW_HOLD_MAX_RATE;
        else if (yaw_rate_cmd < -YAW_HOLD_MAX_RATE) yaw_rate_cmd = -YAW_HOLD_MAX_RATE;
    }

    // ===================================================================
    // 4. Tính toán Sai số (Error)
    // ===================================================================
    float error_pitch = setpoint_pitch - pitch;
    float error_roll  = setpoint_roll - roll;
    float error_yaw = yaw_rate_cmd - gz_dps;

    float pid_pitch = 0.0f, pid_roll = 0.0f, pid_yaw = 0.0f;

    // ===================================================================
    // 5. Tính toán bộ điều khiển PID
    // ===================================================================
    if (throttle_pwm > 1050) {
        // Anti-windup: chỉ cộng dồn tích phân nếu vòng trước KHÔNG bão hòa,
        // hoặc nếu sai số đang kéo output đi ngược chiều đang kẹt (giúp thoát nhanh).
        if (pitch_saturated == 0 || (pitch_saturated > 0 && error_pitch < 0.0f)
                                  || (pitch_saturated < 0 && error_pitch > 0.0f)) {
            integral_pitch += error_pitch * dt;
        }
        if(integral_pitch > 400.0f) integral_pitch = 400.0f;
        else if(integral_pitch < -400.0f) integral_pitch = -400.0f;

        if (roll_saturated == 0 || (roll_saturated > 0 && error_roll < 0.0f)
                                 || (roll_saturated < 0 && error_roll > 0.0f)) {
            integral_roll += error_roll * dt;
        }
        if(integral_roll > 400.0f) integral_roll = 400.0f;
        else if(integral_roll < -400.0f) integral_roll = -400.0f;

        if (yaw_saturated == 0 || (yaw_saturated > 0 && error_yaw < 0.0f)
                                || (yaw_saturated < 0 && error_yaw > 0.0f)) {
            integral_yaw += error_yaw * dt;
        }
        if(integral_yaw > 400.0f) integral_yaw = 400.0f;
        else if(integral_yaw < -400.0f) integral_yaw = -400.0f;

        pid_pitch = (Kp_pitch * error_pitch) + (Ki_pitch * integral_pitch) - (Kd_pitch * gy_dps);
        pid_roll  = (Kp_roll * error_roll)   + (Ki_roll * integral_roll)   - (Kd_roll * gx_dps);
        pid_yaw   = (Kp_yaw * error_yaw)     + (Ki_yaw * integral_yaw);
    } else {
        integral_pitch = 0.0f;
        integral_roll = 0.0f;
        integral_yaw = 0.0f;
        pitch_saturated = 0;
        roll_saturated = 0;
        yaw_saturated = 0;
    }

    // Clamp output VÀ ghi nhớ CHIỀU bão hòa cho vòng lặp kế tiếp
    if(pid_pitch > 400.0f) { pid_pitch = 400.0f; pitch_saturated = 1; }
    else if(pid_pitch < -400.0f) { pid_pitch = -400.0f; pitch_saturated = -1; }
    else pitch_saturated = 0;

    if(pid_roll > 400.0f) { pid_roll = 400.0f; roll_saturated = 1; }
    else if(pid_roll < -400.0f) { pid_roll = -400.0f; roll_saturated = -1; }
    else roll_saturated = 0;

    if(pid_yaw > 400.0f) { pid_yaw = 400.0f; yaw_saturated = 1; }
    else if(pid_yaw < -400.0f) { pid_yaw = -400.0f; yaw_saturated = -1; }
    else yaw_saturated = 0;

    // ===================================================================
    // 6 & 7. Xử lý Ga, Motor Mixing & Bảo vệ hành trình
    // ===================================================================
    if (throttle_pwm > 1050) {
        // Dùng hàm map() ép dải ga của tay cầm (1050 -> 2000)
        // Khớp với dải ga thực tế của ESC (950 -> 2000).
        // 950 là tốc độ Idle để motor quay nhẹ, không bị giật.
        float base_throttle = (float)map(throttle_pwm, 1050, 2000, 950, 2000);

        out_m1 = base_throttle + pid_pitch - pid_roll - pid_yaw;
        out_m2 = base_throttle - pid_pitch - pid_roll + pid_yaw;
        out_m3 = base_throttle + pid_pitch + pid_roll + pid_yaw;
        out_m4 = base_throttle - pid_pitch + pid_roll - pid_yaw;

        // Chặn Idle: Ép không cho motor nào tụt dưới 950us để chống chết máy trên không
        if (out_m1 < 950) out_m1 = 950;
        if (out_m2 < 950) out_m2 = 950;
        if (out_m3 < 950) out_m3 = 950;
        if (out_m4 < 950) out_m4 = 950;

    } else {
        // Khi hạ ga kịch sàn (throttle_pwm < 1050) -> TẮT ĐỘNG CƠ (900us)
        out_m1 = 900;
        out_m2 = 900;
        out_m3 = 900;
        out_m4 = 900;
    }

    // Chặn Max: Không cho vượt quá 2000us
    if (out_m1 > 2000) out_m1 = 2000;
    if (out_m2 > 2000) out_m2 = 2000;
    if (out_m3 > 2000) out_m3 = 2000;
    if (out_m4 > 2000) out_m4 = 2000;

    // ===================================================================
    // 8. Xuất xung PWM ra phần cứng
    // ===================================================================
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, out_m1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, out_m2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, out_m3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, out_m4);
}
