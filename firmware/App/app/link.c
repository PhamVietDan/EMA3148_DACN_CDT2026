// Ground-station link: output goes to USB CDC (when a host has the port open) and to the USART6 radio
// bridge; commands are accepted from either. Same text protocol on both (PLAN §9.1).
#include "app/link.h"

#include "bsp/uart_link.h"
#include "bsp/usb_cdc_port.h"

// The radio port is always written: there is no way to tell whether a GCS is listening behind the bridge,
// and the UART drains at line rate either way.
void link_write_to(unsigned to, const void *data, size_t len)
{
    if ((to & LINK_TO_USB) && usb_cdc_port_host_open()) {
        (void)usb_cdc_port_write(data, len);
    }
    if (to & LINK_TO_RADIO) {
        (void)uart_link_write(data, len);
    }
}

void link_write(const void *data, size_t len)
{
    link_write_to(LINK_TO_ALL, data, len);
}

// A closed USB port has unlimited room (nothing is written there).
size_t link_tx_free_to(unsigned to)
{
    size_t room = (size_t)-1;
    if (to & LINK_TO_RADIO) {
        room = uart_link_tx_free();
    }
    if ((to & LINK_TO_USB) && usb_cdc_port_host_open()) {
        size_t usb = usb_cdc_port_tx_free();
        room = (usb < room) ? usb : room;
    }
    return room;
}

size_t link_tx_free(void)
{
    return link_tx_free_to(LINK_TO_ALL);
}

void link_flush(void)
{
    usb_cdc_port_flush();
    uart_link_flush();
}

size_t link_read(link_src_t src, void *dst, size_t max_len)
{
    return (src == LINK_SRC_USB) ? usb_cdc_port_read(dst, max_len) : uart_link_read(dst, max_len);
}
