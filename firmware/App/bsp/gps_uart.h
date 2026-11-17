// GEP-M10DI GPS on USART3 (PC10 TX -> GPS RX, PC11 RX <- GPS TX; wiring as the legacy code), 115200 8N1 from
// the .ioc. RX by circular DMA set up here at register level (DMA1_Stream1, no interrupt), polled by task_slow.
#pragma once

#include <stddef.h>
#include <stdint.h>

void gps_uart_start(void);                        // task_slow
size_t gps_uart_read(void *dst, size_t max_len);  // task_slow only
// Blocking (~0.09 ms per byte): only for the short configuration frames, while disarmed.
void gps_uart_write(const void *data, size_t len);
