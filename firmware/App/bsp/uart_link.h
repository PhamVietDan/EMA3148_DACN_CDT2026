// Ground-station radio link on USART6 (PC6 TX, PC7 RX): HC-05 Bluetooth (2026-11-09; was the ESP-NOW bridge at
// 460800). Same text protocol as USB.
#pragma once

#include <stddef.h>
#include <stdint.h>

// 115200 baud = the HC-05 setting (AT+UART=115200,0,0): ~11.5 KB/s, so task_telem sends the radio a reduced
// stream (T lines at ~17 Hz, R at 2 Hz, DUMP/CAPTURE slower); USB keeps the full rate.
#define UART_LINK_BAUD 115200u

void   uart_link_init(void);  // before the scheduler: sets the baud rate (overrides the .ioc), enables RX

// TX side: write from any task; flush only from task_telem (the single UART transmitter).
size_t uart_link_write(const void *data, size_t len);  // returns bytes queued; excess is dropped
void   uart_link_flush(void);                          // never blocks; starts one DMA chunk if idle
size_t uart_link_tx_free(void);

size_t uart_link_read(void *dst, size_t max_len);

// Called first in USART6_IRQHandler (stm32f4xx_it.c), before HAL_UART_IRQHandler.
void   uart_link_irq(void);
