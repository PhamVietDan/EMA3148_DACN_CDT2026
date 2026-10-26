// SPI1 link to the IMU: blocking transfers for init, INT1(EXTI)-triggered DMA bursts for streaming.
#include "board_hardware_layer/spi_imu.h"

#include <stdbool.h>

#include "board_hardware_layer/time_us.h"
#include "board_hardware_layer/trace.h"
#include "main.h"
#include "spi.h"

static const uint8_t *stream_tx;
static uint8_t *stream_rx;
static uint16_t stream_len;
static spi_imu_burst_cb_t stream_cb;
static volatile bool streaming;
static volatile bool burst_busy;
static uint32_t burst_t_cyc;

static inline void cs_low(void)  { IMU_CS_GPIO_Port->BSRR = (uint32_t)IMU_CS_Pin << 16; }
static inline void cs_high(void) { IMU_CS_GPIO_Port->BSRR = IMU_CS_Pin; }

int spi_imu_xfer_blocking(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (streaming) {
        return -1;
    }
    cs_low();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, len, 10);
    cs_high();
    return (st == HAL_OK) ? 0 : -1;
}

void spi_imu_stream_start(const uint8_t *tx, uint8_t *rx, uint16_t len, spi_imu_burst_cb_t cb)
{
    stream_tx = tx;
    stream_rx = rx;
    stream_len = len;
    stream_cb = cb;
    burst_busy = false;
    // Edges seen while INT1 was unconfigured/floating must not start a burst.
    __HAL_GPIO_EXTI_CLEAR_IT(IMU_INT_Pin);
    streaming = true;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != IMU_INT_Pin || !streaming) {
        return;
    }
    uint32_t t = time_cycles();
    trace_mark(TR_IMU_EXTI, 0);
    if (burst_busy) {
        trace_mark(TR_IMU_OVERRUN, 0);
        stream_cb(t, NULL, SPI_IMU_BURST_OVERRUN);
        return;
    }
    burst_busy = true;
    burst_t_cyc = t;
    cs_low();
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)stream_tx, stream_rx, stream_len) != HAL_OK) {
        cs_high();
        burst_busy = false;
        trace_mark(TR_IMU_SPI_ERR, 0);
        stream_cb(t, NULL, SPI_IMU_BURST_ERROR);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi1 || !streaming) {
        return;
    }
    cs_high();
    burst_busy = false;
    trace_mark(TR_IMU_DMA_DONE, 0);
    stream_cb(burst_t_cyc, stream_rx, SPI_IMU_BURST_OK);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi1 || !streaming) {
        return;
    }
    cs_high();
    burst_busy = false;
    trace_mark(TR_IMU_SPI_ERR, 0);
    stream_cb(burst_t_cyc, NULL, SPI_IMU_BURST_ERROR);
}
