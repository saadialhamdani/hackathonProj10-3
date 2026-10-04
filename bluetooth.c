#include "bluetooth.h"

#define REG(addr) (*(volatile uint32_t *)(addr))
#define RCC_AHB1ENR REG(0x40023830UL)
#define RCC_APB1ENR REG(0x40023840UL)
#define RCC_APB2ENR REG(0x40023844UL)
#define GPIOA_MODER REG(0x40020000UL)
#define GPIOA_OTYPER REG(0x40020004UL)
#define GPIOA_PUPDR REG(0x4002000CUL)
#define GPIOA_AFRL REG(0x40020020UL)
#define GPIOA_AFRH REG(0x40020024UL)

#if BT_USART == 2
#define USART_BASE    0x40004400UL
#define BT_TX_PIN     2U /* PA2 / D1 */
#define BT_RX_PIN     3U /* PA3 / D0 */
#define BT_ENABLE_CLK() (RCC_APB1ENR |= (1U << 17))
#else
#define USART_BASE    0x40011000UL
#define BT_TX_PIN     9U /* PA9 / D8 */
#define BT_RX_PIN    10U /* PA10 / D2 */
#define BT_ENABLE_CLK() (RCC_APB2ENR |= (1U << 4))
#endif

#define USART_SR  REG(USART_BASE + 0x00UL)
#define USART_DR  REG(USART_BASE + 0x04UL)
#define USART_BRR REG(USART_BASE + 0x08UL)
#define USART_CR1 REG(USART_BASE + 0x0CUL)
#define USART_CR2 REG(USART_BASE + 0x10UL)
#define USART_CR3 REG(USART_BASE + 0x14UL)

void Bluetooth_Init(void)
{
    RCC_AHB1ENR |= 1U;    /* GPIOA clock */
    BT_ENABLE_CLK();      /* Selected USART clock */
    (void)RCC_AHB1ENR;   /* Clock-enable readback */
#if BT_USART == 2
    (void)RCC_APB1ENR;
#else
    (void)RCC_APB2ENR;
#endif

    /* Alternate-function mode for TX and RX */
    GPIOA_MODER &= ~((3U << (2U * BT_TX_PIN)) |
                     (3U << (2U * BT_RX_PIN)));
    GPIOA_MODER |=  ((2U << (2U * BT_TX_PIN)) |
                     (2U << (2U * BT_RX_PIN)));
    GPIOA_OTYPER &= ~(1U << BT_TX_PIN); /* Push-pull TX */

    GPIOA_PUPDR &= ~(3U << (2U * BT_RX_PIN));
    GPIOA_PUPDR |=  (1U << (2U * BT_RX_PIN)); /* RX pull-up */

#if BT_USART == 2
    GPIOA_AFRL &= ~((15U << 8) | (15U << 12));
    GPIOA_AFRL |=  ((7U << 8) | (7U << 12)); /* AF7 PA2/PA3 */
#else
    GPIOA_AFRH &= ~((15U << 4) | (15U << 8));
    GPIOA_AFRH |=  ((7U << 4) | (7U << 8)); /* AF7 PA9/PA10 */
#endif

    USART_CR1 = 0U; /* Disable USART while configuring */
    USART_CR2 = 0U; /* One stop bit */
    USART_CR3 = 0U; /* No hardware flow control */
    /* OVER8=0, 8 data bits, no parity; round divider to nearest integer */
    USART_BRR = (BT_PCLK_HZ + BT_BAUD / 2U) / BT_BAUD;
    USART_CR1 = (1U << 13) | (1U << 3) | (1U << 2); /* UE/TE/RE */
}

void Bluetooth_Send(const char *message)
{
    if (message == NULL) return;
    while (*message != '\0') {
        while (!(USART_SR & (1U << 7))) { } /* TXE */
        USART_DR = (uint8_t)*message++;
    }
    while (!(USART_SR & (1U << 6))) { } /* TC */
}

void Bluetooth_SendLine(const char *message)
{
    Bluetooth_Send(message);
    Bluetooth_Send("\r\n");
}

int Bluetooth_ReadLine(char *buffer, size_t size)
{
    static char line[32];
    static size_t length = 0;
    static int discard = 0;
    if (buffer == NULL || size == 0U) return 0;

    /* Reading SR then DR clears ORE/NE/FE/PE on STM32F401. */
    uint32_t status = USART_SR;
    if (status & ((1U << 3) | (1U << 2) |
                  (1U << 1) | (1U << 0))) {
        (void)USART_DR;
        length = 0;
        discard = 0;
        return 0;
    }

    while (USART_SR & (1U << 5)) { /* RXNE: a byte is available */
        char c = (char)(uint8_t)USART_DR;
        if (c == '\r') continue;
        if (c == '\n') {
            if (discard || length == 0U || length >= size) {
                length = 0;
                discard = 0;
                return 0;
            }
            for (size_t i = 0; i < length; ++i) buffer[i] = line[i];
            buffer[length] = '\0';
            length = 0;
            discard = 0;
            return 1;
        }
        if (discard) continue;
        if (length < sizeof(line) - 1U) line[length++] = c;
        else { length = 0; discard = 1; }
    }
    return 0;
}
