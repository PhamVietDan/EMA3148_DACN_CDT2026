// Lightweight execution trace: any context records (event, DWT time) in ~0.1 µs; task_telem prints it.
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    TR_IMU_EXTI,      // IMU data-ready ISR
    TR_IMU_DMA_DONE,  // SPI DMA complete ISR
    TR_IMU_OVERRUN,
    TR_IMU_SPI_ERR,
    TR_TICK_ISR,      // TIM7 HAL tick ISR
    TR_USB_ISR,       // OTG_FS ISR
    TR_TASK_IN,       // scheduler switched to task `arg` (trace_task_id_t)
    TR_CTRL_DONE,
    TR_CTRL_TIMEOUT,
    TR_COUNT
} trace_event_t;

typedef enum { TRACE_TASK_OTHER, TRACE_TASK_CTRL, TRACE_TASK_SLOW, TRACE_TASK_TELEM } trace_task_id_t;

typedef struct {
    uint32_t t_cyc;
    uint8_t event;
    uint8_t arg;
} trace_rec_t;

#define TRACE_CAPTURE_LEN 512u
#define TRACE_BOOT_LEN    24u

void trace_mark(trace_event_t ev, uint8_t arg);
void trace_boot(const char *static_text);  // one-time start-up milestone; text must be a string literal

// Called by the FreeRTOS traceTASK_SWITCHED_IN hook (inside the kernel, interrupts may be masked).
void trace_task_switched_in(void *tcb);
void trace_set_task_handles(void *ctrl, void *slow, void *telem);

// Capture control: task_telem only.
void trace_capture_arm(void);
bool trace_capture_done(void);
uint32_t trace_capture_count(void);
const trace_rec_t *trace_capture_buf(void);
void trace_capture_release(void);

uint32_t trace_boot_count(void);
const char *trace_boot_text(uint32_t i);
