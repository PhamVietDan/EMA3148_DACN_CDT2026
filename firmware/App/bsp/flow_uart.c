// MTF-01P optical flow / ToF on USART2 (PA2 TX, PA3 RX), 115200 8N1 from the .ioc.
#include "bsp/flow_uart.h"

#include "usart.h"

#define RX_BUF_SIZE 512u  // ~44 ms at 115200 baud; task_slow polls every ~10 ms

static uint8_t rx_buf[RX_BUF_SIZE];  // DMA target: SRAM, not CCM
static uint32_t rx_tail;
static uint32_t restarts;

void flow_uart_start(void)
{
    rx_tail = 0u;
    // DMA_CIRCULAR in the .ioc: runs forever. HAL's error handling stops it on ORE/FE (see flow_uart_read).
    (void)HAL_UART_Receive_DMA(&huart2, rx_buf, RX_BUF_SIZE);
}

uint32_t flow_uart_restarts(void)
{
    return restarts;
}

size_t flow_uart_read(void *dst, size_t max_len)
{
    if (huart2.RxState != HAL_UART_STATE_BUSY_RX) {
        restarts++;
        (void)HAL_UART_AbortReceive(&huart2);
        flow_uart_start();
        return 0u;
    }
    uint8_t *out = dst;
    uint32_t head = RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    if (head >= RX_BUF_SIZE) {
        head = 0u;
    }
    size_t n = 0;
    while (rx_tail != head && n < max_len) {
        out[n++] = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1u) % RX_BUF_SIZE;
    }
    return n;
}
