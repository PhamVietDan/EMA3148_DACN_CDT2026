/*
 * sensor_gps_mag.c
 *
 *  Created on: Aug 13, 2026
 *      Author: ADMIN
 */
#include "sensor_gps_mag.h"
#include "usart.h"
#include "i2c.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "usbd_cdc_if.h" // Cho CDC_Transmit_FS() - gửi qua USB Type-C ảo hóa COM port

// GPS Variables
volatile uint8_t gps_byte;
static volatile char gps_buffer[128];
static volatile uint8_t gps_index = 0;
static volatile char gps_sentence[128];
static volatile uint8_t gps_sentence_ready = 0;

float gps_lat = 0.0f, gps_lon = 0.0f;
int gps_sats = 0;

int16_t mag_x = 0, mag_y = 0, mag_z = 0;
float heading = 0.0f;
// Mag Variables
static float mag_offset_x = 0.0f, mag_offset_y = 0.0f; // Bù nhiễu hard-iron (offset tâm)
static float mag_scale_x = 1.0f, mag_scale_y = 1.0f;   // Bù méo soft-iron (đơn giản, theo trục)

void GPS_Init(void) {
    HAL_UART_Receive_IT(&huart3, &gps_byte, 1);
}

void GPS_Parse_Interrupt(void) {
    if (gps_index < 127) {
        gps_buffer[gps_index++] = gps_byte;
        if (gps_byte == '\n') {
            gps_buffer[gps_index] = '\0';
            // Copy tay thay vì strcpy() vì gps_buffer/gps_sentence là volatile
            // (strcpy không nhận con trỏ volatile một cách an toàn/chuẩn)
            for (uint8_t i = 0; i <= gps_index; i++) {
                gps_sentence[i] = gps_buffer[i];
            }
            gps_sentence_ready = 1;
            gps_index = 0;
        }
    } else {
        gps_index = 0;
    }
    HAL_UART_Receive_IT(&huart3, &gps_byte, 1);
}

// Chuyển định dạng NMEA ddmm.mmmm / dddmm.mmmm sang độ thập phân (decimal degrees)
// is_lon = 1 nếu là kinh độ (3 chữ số độ), 0 nếu là vĩ độ (2 chữ số độ)
static float NMEA_To_Decimal(float raw, uint8_t is_lon) {
    float deg_int;
    if (is_lon) {
        deg_int = (float)((int)(raw / 100.0f));      // dddmm.mmmm -> ddd
    } else {
        deg_int = (float)((int)(raw / 100.0f));      // ddmm.mmmm  -> dd
    }
    float minutes = raw - deg_int * 100.0f;
    return deg_int + (minutes / 60.0f);
}

void GPS_Process_Sentence(void) {
    if (gps_sentence_ready == 1) {
        // Copy sang buffer local KHÔNG volatile để strtok/strncmp hoạt động
        // đúng chuẩn (strtok cần ghi đè lên chuỗi, không dùng được trên volatile).
        char local_sentence[128];
        for (uint8_t i = 0; i < 128; i++) {
            local_sentence[i] = gps_sentence[i];
            if (gps_sentence[i] == '\0') break;
        }
        gps_sentence_ready = 0; // Nhả cờ ngay sau khi copy xong, cho ISR ghi tiếp

        if (strncmp(local_sentence, "$GNGGA", 6) == 0 || strncmp(local_sentence, "$GPGGA", 6) == 0) {
            char *token = strtok(local_sentence, ",");
            int field_count = 0;

            float raw_lat = 0.0f, raw_lon = 0.0f;
            char ns = 'N', ew = 'E';

            while (token != NULL) {
                if (field_count == 2) raw_lat = atof(token);
                if (field_count == 3 && token[0] != '\0') ns = token[0];       // N hoặc S
                if (field_count == 4) raw_lon = atof(token);
                if (field_count == 5 && token[0] != '\0') ew = token[0];       // E hoặc W
                if (field_count == 7) gps_sats = atoi(token);
                token = strtok(NULL, ",");
                field_count++;
            }

            if (raw_lat != 0.0f) {
                float lat = NMEA_To_Decimal(raw_lat, 0);
                gps_lat = (ns == 'S') ? -lat : lat;
            }
            if (raw_lon != 0.0f) {
                float lon = NMEA_To_Decimal(raw_lon, 1);
                gps_lon = (ew == 'W') ? -lon : lon;
            }
        }
    }
}

// Đọc raw mag_x, mag_y trực tiếp từ cảm biến (dùng nội bộ cho calib)
static void Mag_Read_Raw(int16_t *mx, int16_t *my) {
    uint8_t mag_data[6];
    if (HAL_I2C_Mem_Read(&hi2c1, (0x0E << 1), 0x03, I2C_MEMADD_SIZE_8BIT, mag_data, 6, 100) == HAL_OK) {
        *mx = (int16_t)(mag_data[1] << 8 | mag_data[0]);
        *my = (int16_t)(mag_data[3] << 8 | mag_data[2]);
    }
}

void Mag_Init(void) {
    uint8_t mag_cfg = 0x0B;
    HAL_I2C_Mem_Write(&hi2c1, (0x0E << 1), 0x0A, I2C_MEMADD_SIZE_8BIT, &mag_cfg, 1, 100);
    HAL_Delay(50);
}

// ============================================================================
// HIỆU CHUẨN LA BÀN (Hard-iron / Soft-iron đơn giản theo trục)
// Gọi hàm này TÁCH RIÊNG (ví dụ qua lệnh từ tay cầm hoặc 1 lần lúc lắp máy),
// KHÔNG gọi tự động mỗi lần khởi động. Trong lúc chạy, xoay drone đủ 360°
// quanh trục thẳng đứng (giữ máy ngang) để lấy hết các hướng.
// ============================================================================
void Mag_Calibrate(uint16_t duration_ms) {
    int16_t mx, my;
    float min_x = 32767, max_x = -32768;
    float min_y = 32767, max_y = -32768;

    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < duration_ms) {
        Mag_Read_Raw(&mx, &my);

        if (mx < min_x) min_x = mx;
        if (mx > max_x) max_x = mx;
        if (my < min_y) min_y = my;
        if (my > max_y) max_y = my;

        HAL_Delay(20); // ~50Hz lấy mẫu trong lúc xoay
    }

    // Hard-iron offset: tâm dịch chuyển của hình elip đo được
    mag_offset_x = (max_x + min_x) / 2.0f;
    mag_offset_y = (max_y + min_y) / 2.0f;

    // Soft-iron scale đơn giản: cân bán kính 2 trục về bằng nhau
    float delta_x = (max_x - min_x) / 2.0f;
    float delta_y = (max_y - min_y) / 2.0f;
    float avg_delta = (delta_x + delta_y) / 2.0f;

    if (delta_x > 1.0f) mag_scale_x = avg_delta / delta_x; else mag_scale_x = 1.0f;
    if (delta_y > 1.0f) mag_scale_y = avg_delta / delta_y; else mag_scale_y = 1.0f;

    // In kết quả ra USB (Type-C, hiện thành COM port ảo) để COPY-PASTE THẲNG
    // vào code (thay 4 dòng "static float mag_offset_x = 0.0f..." ở đầu file
    // bằng 4 dòng này), rồi xóa lệnh gọi Mag_Calibrate() khỏi main.c.
    char calib_msg[180];
    int len = sprintf(calib_msg,
        "\r\n===== MAG CALIB DONE - COPY 4 DONG DUOI DAY =====\r\n"
        "static float mag_offset_x = %.6ff;\r\n"
        "static float mag_offset_y = %.6ff;\r\n"
        "static float mag_scale_x  = %.6ff;\r\n"
        "static float mag_scale_y  = %.6ff;\r\n"
        "==================================================\r\n",
        mag_offset_x, mag_offset_y, mag_scale_x, mag_scale_y);

    // CDC_Transmit_FS trả về USBD_BUSY nếu buffer USB trước đó chưa gửi xong,
    // nên thử lại vài lần (chỉ chạy 1 lần lúc calib nên không ảnh hưởng hiệu năng).
    for (uint8_t retry = 0; retry < 20; retry++) {
        if (CDC_Transmit_FS((uint8_t*)calib_msg, len) == USBD_OK) break;
        HAL_Delay(20);
    }
}

void Mag_Read_Heading(void) {
    uint8_t mag_data[6];
    if (HAL_I2C_Mem_Read(&hi2c1, (0x0E << 1), 0x03, I2C_MEMADD_SIZE_8BIT, mag_data, 6, 100) == HAL_OK) {
        mag_x = (int16_t)(mag_data[1] << 8 | mag_data[0]);
        mag_y = (int16_t)(mag_data[3] << 8 | mag_data[2]);
        mag_z = (int16_t)(mag_data[5] << 8 | mag_data[4]); // Z chua su dung de tinh 2D

        // Áp dụng bù hard-iron (trừ offset tâm) và soft-iron (cân tỉ lệ trục)
        float mx_cal = ((float)mag_x - mag_offset_x) * mag_scale_x;
        float my_cal = ((float)mag_y - mag_offset_y) * mag_scale_y;

        heading = atan2(my_cal, mx_cal) * 57.2958f;
        if (heading < 0.0f) heading += 360.0f;
    }
}
