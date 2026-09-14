#include "rc_sbus.h"
#include "usart.h"

uint16_t rc_channel[16];

// Khai báo 4 biến chứa giá trị PWM đã được map chuẩn
uint16_t roll_pwm = 1500;
uint16_t pitch_pwm = 1500;
uint16_t throttle_pwm = 1000;
uint16_t yaw_pwm = 1500;

uint8_t sbus_byte;
uint8_t sbus_buffer[25];
uint8_t sbus_index = 0;

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void SBUS_Init(void) {
    HAL_UART_Receive_IT(&huart1, &sbus_byte, 1);
}
void SBUS_Parse(void) {
    if (sbus_index == 0 && sbus_byte != 0x0F) {
        // Sai byte Header, bo qua
    } else {
        sbus_buffer[sbus_index++] = sbus_byte;

        if (sbus_index == 25) {
            sbus_index = 0;
            // Kiem tra byte End
            if (sbus_buffer[24] == 0x00) {
                // Đọc giá trị thô 4 kênh cơ bản
                rc_channel[0] = (sbus_buffer[1] | sbus_buffer[2] << 8) & 0x07FF; // CH1: Roll
                rc_channel[1] = (sbus_buffer[2] >> 3 | sbus_buffer[3] << 5) & 0x07FF; // CH2: Pitch
                rc_channel[2] = (sbus_buffer[3] >> 6 | sbus_buffer[4] << 2 | sbus_buffer[5] << 10) & 0x07FF; // CH3: Throttle
                rc_channel[3] = (sbus_buffer[5] >> 1 | sbus_buffer[6] << 7) & 0x07FF; // CH4: Yaw

                // ---------------------------------------------------------
                // 1. Map ROLL (Trái: 627 | Giữa: 1017 | Phải: 1400)
                // ---------------------------------------------------------
                if (rc_channel[0] < 1017) {
                    roll_pwm = map(rc_channel[0], 627, 1017, 1000, 1500);
                } else {
                    roll_pwm = map(rc_channel[0], 1017, 1400, 1500, 2000);
                }
                if (roll_pwm < 1000) roll_pwm = 1000;
                if (roll_pwm > 2000) roll_pwm = 2000;

                // ---------------------------------------------------------
                // 2. Map PITCH (Trước: 737 | Giữa: 1126 | Sau: 1400)
                // ---------------------------------------------------------
                if (rc_channel[1] < 1126) {
                    pitch_pwm = map(rc_channel[1], 737, 1126, 1000, 1500);
                } else {
                    pitch_pwm = map(rc_channel[1], 1126, 1400, 1500, 2000);
                }
                if (pitch_pwm < 1000) pitch_pwm = 1000;
                if (pitch_pwm > 2000) pitch_pwm = 2000;

                // ---------------------------------------------------------
                // 3. Map THROTTLE (Dưới: 200 | Trên: 1768)
                // ---------------------------------------------------------
                throttle_pwm = map(rc_channel[2], 200, 1768, 1000, 2000);
                if (throttle_pwm < 1000) throttle_pwm = 1000;
                if (throttle_pwm > 2000) throttle_pwm = 2000;

                // ---------------------------------------------------------
                // 4. Map YAW (Trái: 601 | Giữa: 976 | Phải: 1376)
                // ---------------------------------------------------------
                if (rc_channel[3] < 976) {
                    yaw_pwm = map(rc_channel[3], 601, 976, 1000, 1500);
                } else {
                    yaw_pwm = map(rc_channel[3], 976, 1376, 1500, 2000);
                }
                if (yaw_pwm < 1000) yaw_pwm = 1000;
                if (yaw_pwm > 2000) yaw_pwm = 2000;
            }
        }
    }
    HAL_UART_Receive_IT(&huart1, &sbus_byte, 1);
}
