// USB CDC (virtual COM) port: ring buffers between the OTG_FS ISR and the single TX task.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "cmsis_os.h"

void   usb_cdc_port_set_rx_notify(osThreadId thread, int32_t signal);
bool   usb_cdc_port_configured(void);
bool   usb_cdc_port_host_open(void);   // DTR set: a terminal/GCS has the COM port open
size_t usb_cdc_port_tx_free(void);

// TX side: write from any task; flush only from task_telem (the single USB transmitter).
size_t usb_cdc_port_write(const void *data, size_t len);  // returns bytes queued; excess is dropped
void   usb_cdc_port_flush(void);                          // never blocks; resumes next call if USB busy

size_t usb_cdc_port_rx_available(void);
size_t usb_cdc_port_read(void *dst, size_t max_len);

// Called from CDC_Receive_FS / CDC_Control_FS (OTG_FS ISR context).
void   usb_cdc_port_on_rx_isr(const uint8_t *buf, uint32_t len);
void   usb_cdc_port_on_line_state_isr(uint16_t w_value);
