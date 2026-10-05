#include "sensor_imu.h"
#include "spi.h"   // Đổi thư viện i2c thành spi
#include "main.h"  // Chứa định nghĩa các chân GPIO
#include <math.h>

// --- ĐỊNH NGHĨA SPI & THANH GHI ---
// BMI323 không cần địa chỉ I2C nữa, chỉ cần địa chỉ thanh ghi
#define BMI323_REG_CHIP_ID  0x00
#define BMI323_REG_ACC_CONF 0x20
#define BMI323_REG_GYR_CONF 0x21
#define BMI323_REG_ACC_X    0x03 // Địa chỉ bắt đầu của dữ liệu trục X gia tốc

// Định nghĩa chân CS (Chip Select) là PD2
#define BMI323_CS_PORT      GPIOD
#define BMI323_CS_PIN       GPIO_PIN_2

#define DIR_PITCH_ACC  -1.0f // Đã lật để gia tốc ra số Dương khi chúi mũi
#define DIR_ROLL_ACC   1.0f

// Lật DIR_PITCH_GYR thành 1.0f để ĐỒNG TÌNH với Gia tốc. Hết đánh nhau!
#define DIR_PITCH_GYR  1.0f
#define DIR_ROLL_GYR   1.0f
#define DIR_YAW_GYR   -1.0f

// --- BIẾN TOÀN CỤC ---
uint8_t bmi323_chip_id = 0;
float acc_offset[3] = {0.0f, 0.0f, 0.0f}; // Cố định bằng 0, không tự động tính lúc cắm pin
float gyr_offset[3] = {0.0f, 0.0f, 0.0f};

float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
float ax_raw, ay_raw, az_raw;
float gx_raw, gy_raw, gz_raw;
float gx_dps = 0.0f, gy_dps = 0.0f, gz_dps = 0.0f;
float calc_acc_offset[3] = {0.0f, 0.0f, 0.0f};

// Biến tĩnh dùng cho bộ lọc LPF
static float ax_filt = 0.0f, ay_filt = 0.0f, az_filt = 1.0f;
static float gx_filt = 0.0f, gy_filt = 0.0f, gz_filt = 0.0f;

static uint32_t prev_time = 0;
static uint8_t is_first_loop = 1; // Cờ báo hiệu vòng lặp tính toán đầu tiên


// =====================================================================
// HÀM TRỢ GIÚP GIAO TIẾP SPI CHO BMI323
// =====================================================================

// Hàm GHI dữ liệu qua SPI
static void BMI323_SPI_Write(uint8_t reg, uint8_t *data, uint16_t len) {
    // Để GHI, bit số 7 của địa chỉ thanh ghi phải là 0
    uint8_t tx_reg = reg & 0x7F;

    HAL_GPIO_WritePin(BMI323_CS_PORT, BMI323_CS_PIN, GPIO_PIN_RESET); // Kéo CS xuống LOW
    HAL_SPI_Transmit(&hspi1, &tx_reg, 1, 100);                        // Gửi địa chỉ
    HAL_SPI_Transmit(&hspi1, data, len, 100);                         // Gửi dữ liệu
    HAL_GPIO_WritePin(BMI323_CS_PORT, BMI323_CS_PIN, GPIO_PIN_SET);   // Kéo CS lên HIGH
}

// Hàm ĐỌC dữ liệu qua SPI (Có xử lý Dummy Byte của Bosch)
static void BMI323_SPI_Read(uint8_t reg, uint8_t *data, uint16_t len) {
    // Để ĐỌC, bit số 7 của địa chỉ thanh ghi phải là 1
    uint8_t tx_reg = reg | 0x80;
    uint8_t dummy = 0x00;

    HAL_GPIO_WritePin(BMI323_CS_PORT, BMI323_CS_PIN, GPIO_PIN_RESET); // Kéo CS xuống LOW
    HAL_SPI_Transmit(&hspi1, &tx_reg, 1, 100);                        // Gửi địa chỉ

    // Cảm biến Bosch YÊU CẦU đọc 1 byte rác (Dummy byte) trước khi trả dữ liệu thật
    HAL_SPI_Receive(&hspi1, &dummy, 1, 100);

    HAL_SPI_Receive(&hspi1, data, len, 100);                          // Nhận dữ liệu thật
    HAL_GPIO_WritePin(BMI323_CS_PORT, BMI323_CS_PIN, GPIO_PIN_SET);   // Kéo CS lên HIGH
}

// =====================================================================
// CÁC HÀM XỬ LÝ CHÍNH
// =====================================================================

void IMU_Init_And_Calibrate(void) {
    uint8_t id_data[2] = {0}; // BMI323 trả về 16-bit (2 byte) cho thanh ghi 0x00

    // 1. Chờ cảm biến có đủ thời gian lên nguồn
    HAL_Delay(100);
    // Đảm bảo chân CS đang ở mức cao trước khi làm việc
    HAL_GPIO_WritePin(BMI323_CS_PORT, BMI323_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(10);

    // 2. Thử đọc ID tối đa 5 lần
    for(int retry = 0; retry < 5; retry++) {
        BMI323_SPI_Read(BMI323_REG_CHIP_ID, id_data, 2);

        bmi323_chip_id = id_data[0]; // Byte thấp (LSB) chứa Chip ID

        if (bmi323_chip_id == 0x43) break; // Đọc đúng ID thì thoát vòng lặp ngay
        HAL_Delay(50);
    }

    // 3. Nếu đọc thành công ID mới tiến hành cấu hình
    if (bmi323_chip_id == 0x43) {
        HAL_Delay(10);

        // Cấu hình gia tốc (Ghi 2 byte LSB và MSB)
        uint8_t acc_cfg[2] = {0x08, 0x70};
        BMI323_SPI_Write(BMI323_REG_ACC_CONF, acc_cfg, 2);

        // Cấu hình con quay (Ghi 2 byte LSB và MSB)
        uint8_t gyr_cfg[2] = {0x08, 0x70};
        BMI323_SPI_Write(BMI323_REG_GYR_CONF, gyr_cfg, 2);

        // Chờ màng lọc bên trong cảm biến ổn định trước khi Calib
        HAL_Delay(1000);

        // ========================================================
        // CALIBRATION: CHỈ ĐO NHIỄU GYRO, KHÔNG TỰ ĐỘNG ĐO ACCEL
        // ========================================================
        uint8_t raw_calib[12];
        int calib_count = 500;

        // Reset offset về 0 trước khi cộng dồn
        gyr_offset[0] = 0; gyr_offset[1] = 0; gyr_offset[2] = 0;

        for (int i = 0; i < calib_count; i++) {
            BMI323_SPI_Read(BMI323_REG_ACC_X, raw_calib, 12);

            // Chỉ tính toán bù trừ cho Gyro (Trục X, Y, Z của Gyro nằm ở index 6 tới 11)
            int16_t gx = (int16_t)(raw_calib[7] << 8 | raw_calib[6]);
            int16_t gy = (int16_t)(raw_calib[9] << 8 | raw_calib[8]);
            int16_t gz = (int16_t)(raw_calib[11] << 8 | raw_calib[10]);

            gyr_offset[0] += (gx / 262.4f);
            gyr_offset[1] += (gy / 262.4f);
            gyr_offset[2] += (gz / 262.4f);

            HAL_Delay(2);
        }

        for(int i = 0; i < 3; i++) {
            gyr_offset[i] /= calib_count; // Tính trung bình nhiễu tĩnh Gyro
        }

        // Cố định acc_offset bằng 0 (Giữ nguyên gốc để luôn biết mặt đất nằm đâu)
        acc_offset[0] = -0.00714025879f;
        acc_offset[1] = 0.0123070683f;
        acc_offset[2] = -0.0119230747f;

        prev_time = HAL_GetTick();
        is_first_loop = 1; // Khởi tạo cờ cho vòng lặp đầu tiên
    }
}

void IMU_Read_And_Calculate_Angles(void) {
    if (bmi323_chip_id != 0x43) return;

    uint8_t raw_data[12];

    // Đọc liên tiếp 12 byte bắt đầu từ thanh ghi ACC_X
    BMI323_SPI_Read(BMI323_REG_ACC_X, raw_data, 12);

    // 1. Đọc dữ liệu raw
    int16_t acc_x = (int16_t)(raw_data[1] << 8 | raw_data[0]);
    int16_t acc_y = (int16_t)(raw_data[3] << 8 | raw_data[2]);
    int16_t acc_z = (int16_t)(raw_data[5] << 8 | raw_data[4]);
    int16_t gyr_x = (int16_t)(raw_data[7] << 8 | raw_data[6]);
    int16_t gyr_y = (int16_t)(raw_data[9] << 8 | raw_data[8]);
    int16_t gyr_z = (int16_t)(raw_data[11] << 8 | raw_data[10]);

    // 2. Chuyển đổi đơn vị và trừ offset
    float ax_g_raw = (acc_x / 16384.0f) - acc_offset[0];
    float ay_g_raw = (acc_y / 16384.0f) - acc_offset[1];
    float az_g_raw = (acc_z / 16384.0f) - acc_offset[2];

    float gx_dps_raw = (gyr_x / 262.4f) - gyr_offset[0];
    float gy_dps_raw = (gyr_y / 262.4f) - gyr_offset[1];
    float gz_dps_raw = (gyr_z / 262.4f) - gyr_offset[2];

    ax_raw = ax_g_raw; ay_raw = ay_g_raw; az_raw = az_g_raw;
    gx_raw = gx_dps_raw; gy_raw = gy_dps_raw; gz_raw = gz_dps_raw;

    // 3. Áp dụng Low-Pass Filter (LPF) dập nhiễu
    float alpha_acc = 0.15f;
    float alpha_gyr = 0.15f;

    ax_filt = ax_filt + alpha_acc * (ax_g_raw - ax_filt);
    ay_filt = ay_filt + alpha_acc * (ay_g_raw - ay_filt);
    az_filt = az_filt + alpha_acc * (az_g_raw - az_filt);

    gx_filt = gx_filt + alpha_gyr * (gx_dps_raw - gx_filt);
    gy_filt = gy_filt + alpha_gyr * (gy_dps_raw - gy_filt);
    gz_filt = gz_filt + alpha_gyr * (gz_dps_raw - gz_filt);

    // Gán lại cho các biến toàn cục để PID sử dụng
    gx_dps = DIR_ROLL_GYR * gx_filt;
    gy_dps = DIR_PITCH_GYR * gy_filt;
    gz_dps = DIR_YAW_GYR * gz_filt;

    // 4. Tính toán thời gian dt
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - prev_time) / 1000.0f;
    if (current_time > prev_time) {
        prev_time = current_time;
    } else {
        dt = 0.002f; // Tránh lỗi loop nhỏ hơn 1ms
    }
    if(dt > 0.1f) dt = 0.002f; // Tránh lỗi khi vừa khởi động lên bị cộng dồn góc

    // 5. Tính góc Pitch và Roll từ Accel (Gia tốc kế)
    // ÁP DỤNG QUY ƯỚC: Trục X hướng trước. Roll Phải (+), Pitch Chúi (+), Yaw Phải (+)
    float pitch_acc = atan2(DIR_PITCH_ACC * ax_filt, az_filt) * 57.2958f;
    float roll_acc = atan2(DIR_ROLL_ACC * ay_filt, sqrt(ax_filt*ax_filt + az_filt*az_filt)) * 57.2958f;

    float pitch_rate = gy_dps;
    float roll_rate  = gx_dps;
    float yaw_rate   = gz_dps;

    // 6. Áp dụng Complementary filter (Xử lý chống trễ ở vòng lặp đầu tiên)
    if (is_first_loop) {
        // Gán thẳng góc tuyệt đối từ gia tốc vào để Drone nhận diện đúng tư thế ngay lập tức
        pitch = pitch_acc;
        roll = roll_acc;
        // Yaw mặc định luôn xuất phát từ 0 độ
        yaw = 0.0f;

        is_first_loop = 0; // Xóa cờ, từ giờ sẽ lọc bình thường
    } else {
        // Bộ lọc bù Complementary Filter tiêu chuẩn
        pitch = 0.98f * (pitch + pitch_rate * dt) + 0.02f * pitch_acc;
        roll  = 0.98f * (roll + roll_rate * dt)   + 0.02f * roll_acc;
    }

    // 7. Xử lý tính toán Yaw (Chỉ lấy từ Gyro)
    if (fabs(yaw_rate) > 0.5f) {
        yaw += (yaw_rate * dt);
    }

    // Đưa góc Yaw về dải [-180, +180] độ để dễ đưa vào bộ điều khiển PID
    if (yaw > 180.0f) {
        yaw -= 360.0f;
    } else if (yaw < -180.0f) {
        yaw += 360.0f;
    }
}

