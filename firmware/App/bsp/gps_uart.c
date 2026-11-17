// GEP-M10DI GPS on USART3, see gps_uart.h. Same register-level circular RX DMA as uart_link.c.
#include "bsp/gps_uart.h"

#include "usart.h"

#define RX_BUF_SIZE    512u  // ~44 ms at 115200 baud; a NAV-PVT frame is 100 bytes, task_slow polls every ~10 ms
#define RX_DMA         DMA1_Stream1
// VERIFY: RM0090 Table 42 (DMA1 request mapping): USART3_RX = stream 1, channel 4 (the same channel-4 row as
// USART2_RX on stream 5, which CubeMX set up in usart.c).
#define RX_DMA_CHANNEL 4u

static uint8_t rx_buf[RX_BUF_SIZE];  // written by DMA (SRAM, not CCM)
static uint32_t rx_tail;

void gps_uart_start(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    RX_DMA->CR = 0u;
    while (RX_DMA->CR & DMA_SxCR_EN) {
    }
    DMA1->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;
    RX_DMA->PAR = (uint32_t)&USART3->DR;
    RX_DMA->M0AR = (uint32_t)rx_buf;
    RX_DMA->NDTR = RX_BUF_SIZE;
    RX_DMA->FCR = 0u;  // direct mode
    // Peripheral -> memory, bytes, memory increment, circular, low priority, no interrupts.
    RX_DMA->CR = (RX_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_CIRC;
    RX_DMA->CR |= DMA_SxCR_EN;
    SET_BIT(USART3->CR3, USART_CR3_DMAR);
    rx_tail = 0u;
}

size_t gps_uart_read(void *dst, size_t max_len)
{
    if ((RX_DMA->CR & DMA_SxCR_EN) == 0u) {
        return 0u;
    }
    uint8_t *out = dst;
    uint32_t head = RX_BUF_SIZE - RX_DMA->NDTR;
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

void gps_uart_write(const void *data, size_t len)
{
    (void)HAL_UART_Transmit(&huart3, (const uint8_t *)data, (uint16_t)len, 20u);
}
