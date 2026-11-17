// MTF-01P optical flow / ToF on USART2 (PA2 TX, PA3 RX), 115200 8N1 from the .ioc. RX by circular DMA
// (DMA1_Stream5, set up by CubeMX); the reader polls the DMA position, so no RX callback is needed.
#pragma once

#include <stddef.h>
#include <stdint.h>

void     flow_uart_start(void);                    // task_slow
size_t   flow_uart_read(void *dst, size_t max_len);  // task_slow only; restarts RX after a UART error
uint32_t flow_uart_restarts(void);
