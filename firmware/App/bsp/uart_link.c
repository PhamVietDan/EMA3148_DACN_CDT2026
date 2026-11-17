// Ground-station radio link on USART6 (PC6 TX, PC7 RX), HC-05. Same text protocol as USB.
// TX: ring -> HAL DMA chunk (DMA2_Stream6, from the .ioc).
// RX: circular DMA (DMA2_Stream1, set up here at register level, no HAL receive, no interrupt). An RXNE
// interrupt lost bytes at 460800 baud (a byte every ~22 us, the USART holds only one, any longer ISR latency
// overwrote it); kept at 115200, it costs nothing.
#include "bsp/uart_link.h"

#include "usart.h"

#define TX_RING_SIZE 4096u  // power of two
#define RX_BUF_SIZE  512u   // DMA ring: > 20 ms of line rate, the task_telem polling period
// ~17 ms at 115200 baud: done before the next 20 ms flush (a longer chunk would make every other flush find
// the DMA busy and halve the rate). ~10 KB/s.
#define TX_CHUNK_MAX 200u

#define RX_DMA         DMA2_Stream1
#define RX_DMA_CHANNEL 5u  // VERIFY: RM0090 Table 43 (DMA2 request mapping): USART6_RX = stream 1/2, channel 5

// Indices are free-running; (head - tail) is the fill level even after wrap-around.
static uint8_t tx_ring[TX_RING_SIZE];
static uint32_t tx_head;
static uint32_t tx_tail;
// Read by DMA until gState returns to READY. Normal .bss (SRAM): DMA2 cannot reach CCM.
static uint8_t tx_chunk[TX_CHUNK_MAX];

static uint8_t rx_buf[RX_BUF_SIZE];  // written by DMA (SRAM, not CCM)
static uint32_t rx_tail;             // next index to read; only the reader task

void uart_link_init(void)
{
    huart6.Init.BaudRate = UART_LINK_BAUD;
    if (HAL_UART_Init(&huart6) != HAL_OK) {
        return;  // link stays silent; USB still works
    }
    __HAL_RCC_DMA2_CLK_ENABLE();
    RX_DMA->CR = 0u;
    while (RX_DMA->CR & DMA_SxCR_EN) {
    }
    DMA2->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;
    RX_DMA->PAR = (uint32_t)&USART6->DR;
    RX_DMA->M0AR = (uint32_t)rx_buf;
    RX_DMA->NDTR = RX_BUF_SIZE;
    RX_DMA->FCR = 0u;  // direct mode
    // Peripheral -> memory, bytes, memory increment, circular, low priority, no interrupts.
    RX_DMA->CR = (RX_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_CIRC;
    RX_DMA->CR |= DMA_SxCR_EN;
    SET_BIT(USART6->CR3, USART_CR3_DMAR);  // HAL's TX DMA only sets/clears DMAT, never DMAR
    rx_tail = 0u;
}

size_t uart_link_tx_free(void)
{
    return TX_RING_SIZE - (tx_head - tx_tail);
}

// Same locking as usb_cdc_port_write: several tasks may write, PRIMASK also works before the scheduler.
size_t uart_link_write(const void *data, size_t len)
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

void uart_link_flush(void)
{
    if (huart6.gState != HAL_UART_STATE_READY) {
        return;  // previous chunk still in flight
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
    if (HAL_UART_Transmit_DMA(&huart6, tx_chunk, (uint16_t)n) == HAL_OK) {
        tx_tail += n;
    }
}

size_t uart_link_read(void *dst, size_t max_len)
{
    if ((RX_DMA->CR & DMA_SxCR_EN) == 0u) {
        return 0u;  // init failed
    }
    uint8_t *out = dst;
    uint32_t head = RX_BUF_SIZE - RX_DMA->NDTR;  // NDTR counts down and reloads at 0 (circular)
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

// RX runs on DMA now; kept so stm32f4xx_it.c needs no change. HAL_UART_IRQHandler still handles the TX
// DMA completion; no receive interrupt is enabled, so it never sees RX events.
void uart_link_irq(void)
{
}
