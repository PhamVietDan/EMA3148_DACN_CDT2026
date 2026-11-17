// SPI1 link to the IMU: blocking transfers for init, INT1(EXTI)-triggered DMA bursts for streaming.
#pragma once

#include <stdint.h>

typedef enum {
    SPI_IMU_BURST_OK,
    SPI_IMU_BURST_OVERRUN,  // data-ready edge arrived while the previous burst was still in flight
    SPI_IMU_BURST_ERROR,
} spi_imu_burst_status_t;

// Runs in DMA-complete / EXTI ISR context (NVIC priority 5). rx is NULL unless status is OK.
typedef void (*spi_imu_burst_cb_t)(uint32_t t_cyc, const uint8_t *rx, spi_imu_burst_status_t status);

int  spi_imu_xfer_blocking(const uint8_t *tx, uint8_t *rx, uint16_t len);  // 0 on success
void spi_imu_stream_start(const uint8_t *tx, uint8_t *rx, uint16_t len, spi_imu_burst_cb_t cb);
