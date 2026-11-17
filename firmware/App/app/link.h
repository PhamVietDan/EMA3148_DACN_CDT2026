// Ground-station link: output goes to USB CDC (when a host has the port open) and to the USART6 radio
// bridge; commands are accepted from either. Same text protocol on both (PLAN §9.1).
#pragma once

#include <stdbool.h>
#include <stddef.h>

typedef enum { LINK_SRC_USB, LINK_SRC_RADIO, LINK_SRC_COUNT } link_src_t;

// Destination masks. The radio (HC-05 at 115200, ~11 KB/s) cannot take everything USB can.
#define LINK_TO_USB   1u
#define LINK_TO_RADIO 2u
#define LINK_TO_ALL   (LINK_TO_USB | LINK_TO_RADIO)
#define LINK_TO_SRC(src) (((src) == LINK_SRC_USB) ? LINK_TO_USB : LINK_TO_RADIO)

void   link_write(const void *data, size_t len);  // = link_write_to(LINK_TO_ALL, ...)
void   link_write_to(unsigned to, const void *data, size_t len);  // excess beyond a port's free space is dropped
size_t link_tx_free(void);                        // smallest free space among the ports being written
size_t link_tx_free_to(unsigned to);
void   link_flush(void);                          // task_telem only
size_t link_read(link_src_t src, void *dst, size_t max_len);
