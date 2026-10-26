// USART1 SBUS receive: circular DMA, frames delimited by the UART IDLE gap between SBUS frames.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sensor_and_radio_driver/sbus.h"

typedef struct {
    uint32_t frames;     // complete 25-byte bursts captured
    uint32_t bad_len;    // bursts that were not 25 bytes (sync loss, noise)
    uint32_t uart_errs;  // parity / framing / noise / overrun
} rc_uart_stats_t;

void rc_uart_start(void);
// Task context. Copies the newest frame if it is newer than *seq; returns true when it did.
bool rc_uart_get_frame(uint8_t frame[SBUS_FRAME_LEN], uint32_t *seq);
void rc_uart_get_stats(rc_uart_stats_t *st);
