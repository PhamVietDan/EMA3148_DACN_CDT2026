// USART1 SBUS receive: circular DMA, frames delimited by the UART IDLE gap between SBUS frames.
#include "board_hardware_layer/rc_uart.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"

#define DMA_BUF_LEN 64u  // > 2 frames; power of two not required

static uint8_t dma_buf[DMA_BUF_LEN];
static uint16_t last_pos;
static volatile bool burst_corrupt;  // UART error seen since the last IDLE

// Written in the USART1/DMA ISRs (NVIC 6-7, masked by taskENTER_CRITICAL), read by task_slow.
static uint8_t frame_buf[SBUS_FRAME_LEN];
static volatile uint32_t frame_seq;
static volatile rc_uart_stats_t stats;

static void start_rx(void)
{
    last_pos = 0;
    burst_corrupt = false;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, dma_buf, DMA_BUF_LEN) == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);  // only IDLE matters; fewer interrupts
    }
}

void rc_uart_start(void)
{
    start_rx();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &huart1 || HAL_UARTEx_GetRxEventType(huart) != HAL_UART_RXEVENT_IDLE) {
        return;
    }
    uint16_t pos = (uint16_t)(size % DMA_BUF_LEN);
    uint16_t n = (uint16_t)((pos + DMA_BUF_LEN - last_pos) % DMA_BUF_LEN);
    if (n == SBUS_FRAME_LEN && !burst_corrupt) {
        for (uint16_t i = 0; i < SBUS_FRAME_LEN; i++) {
            frame_buf[i] = dma_buf[(last_pos + i) % DMA_BUF_LEN];
        }
        frame_seq++;
        stats.frames++;
    } else if (n != 0u) {
        stats.bad_len++;
    }
    last_pos = pos;
    burst_corrupt = false;
}

// SBUS has no CRC: a parity/framing error is the only sign of a corrupted byte, so the burst is dropped.
// HAL aborts DMA reception on some errors (always on overrun): restart if it is no longer running.
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1) {
        return;
    }
    stats.uart_errs++;
    burst_corrupt = true;
    if (huart->RxState != HAL_UART_STATE_BUSY_RX) {
        start_rx();
    }
}

bool rc_uart_get_frame(uint8_t frame[SBUS_FRAME_LEN], uint32_t *seq)
{
    bool fresh = false;
    taskENTER_CRITICAL();
    if (frame_seq != *seq) {
        memcpy(frame, frame_buf, SBUS_FRAME_LEN);
        *seq = frame_seq;
        fresh = true;
    }
    taskEXIT_CRITICAL();
    return fresh;
}

void rc_uart_get_stats(rc_uart_stats_t *st)
{
    taskENTER_CRITICAL();
    st->frames = stats.frames;
    st->bad_len = stats.bad_len;
    st->uart_errs = stats.uart_errs;
    taskEXIT_CRITICAL();
}
