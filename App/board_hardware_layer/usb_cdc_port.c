// USB CDC (virtual COM) port: ring buffers between the OTG_FS ISR and the single TX task.
#include "board_hardware_layer/usb_cdc_port.h"

#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

#define TX_RING_SIZE 8192u  // power of two
#define RX_RING_SIZE 512u   // power of two
#define TX_CHUNK_MAX 2048u  // one CDC transfer per flush; the USB core splits it into 64-byte packets

// Indices are free-running; (head - tail) is the fill level even after wrap-around.
static uint8_t tx_ring[TX_RING_SIZE];
static uint32_t tx_head;
static uint32_t tx_tail;
// The USB core reads from this buffer asynchronously until TxState returns to 0.
static uint8_t tx_chunk[TX_CHUNK_MAX];

static uint8_t rx_ring[RX_RING_SIZE];
static volatile uint32_t rx_head;  // written only by the OTG_FS ISR
static volatile uint32_t rx_tail;  // written only by the reader task

static osThreadId rx_notify_thread;
static int32_t rx_notify_signal;

void usb_cdc_port_set_rx_notify(osThreadId thread, int32_t signal)
{
    rx_notify_signal = signal;
    rx_notify_thread = thread;
}

bool usb_cdc_port_configured(void)
{
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && hUsbDeviceFS.pClassData != NULL;
}

static volatile bool host_dtr;

void usb_cdc_port_on_line_state_isr(uint16_t w_value)
{
    host_dtr = (w_value & 0x0001u) != 0u;
}

bool usb_cdc_port_host_open(void)
{
    return host_dtr && usb_cdc_port_configured();
}

size_t usb_cdc_port_tx_free(void)
{
    return TX_RING_SIZE - (tx_head - tx_tail);
}

// Several tasks may write; PRIMASK (rather than taskENTER_CRITICAL) also works before the scheduler
// starts. A 128-byte mprintf line keeps interrupts masked for roughly 1 µs.
size_t usb_cdc_port_write(const void *data, size_t len)
{
    const uint8_t *src = data;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t space = TX_RING_SIZE - (tx_head - tx_tail);
    if (len > space) {
        len = space;
    }
    for (size_t i = 0; i < len; i++) {
        tx_ring[(tx_head + i) & (TX_RING_SIZE - 1u)] = src[i];
    }
    tx_head += (uint32_t)len;
    __set_PRIMASK(primask);
    return len;
}

void usb_cdc_port_flush(void)
{
    if (!usb_cdc_port_configured()) {
        return;
    }
    const USBD_CDC_HandleTypeDef *hcdc = hUsbDeviceFS.pClassData;
    if (hcdc->TxState != 0u) {
        return;
    }
    uint32_t n = tx_head - tx_tail;
    if (n == 0u) {
        return;
    }
    if (n > TX_CHUNK_MAX) {
        n = TX_CHUNK_MAX;
    }
    for (uint32_t i = 0; i < n; i++) {
        tx_chunk[i] = tx_ring[(tx_tail + i) & (TX_RING_SIZE - 1u)];
    }
    if (CDC_Transmit_FS(tx_chunk, (uint16_t)n) == USBD_OK) {
        tx_tail += n;
    }
}

size_t usb_cdc_port_rx_available(void)
{
    return rx_head - rx_tail;
}

size_t usb_cdc_port_read(void *dst, size_t max_len)
{
    uint8_t *out = dst;
    uint32_t tail = rx_tail;
    size_t n = rx_head - tail;
    if (n > max_len) {
        n = max_len;
    }
    for (size_t i = 0; i < n; i++) {
        out[i] = rx_ring[(tail + i) & (RX_RING_SIZE - 1u)];
    }
    rx_tail = tail + (uint32_t)n;
    return n;
}

void usb_cdc_port_on_rx_isr(const uint8_t *buf, uint32_t len)
{
    uint32_t head = rx_head;
    for (uint32_t i = 0; i < len; i++) {
        if (head - rx_tail >= RX_RING_SIZE) {
            break;  // full: drop the rest rather than overwrite unread commands
        }
        rx_ring[head & (RX_RING_SIZE - 1u)] = buf[i];
        head++;
    }
    rx_head = head;
    if (rx_notify_thread != NULL) {
        // ST cmsis_os.c osSignalSet takes the FromISR path when inHandlerMode() (verified).
        osSignalSet(rx_notify_thread, rx_notify_signal);
    }
}

// newlib retarget so stray printf() goes to USB; output that does not fit is dropped.
int _write(int file, char *ptr, int len)
{
    (void)file;
    (void)usb_cdc_port_write(ptr, (size_t)len);
    return len;
}
