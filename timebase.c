#include "timebase.h"

#define REG(addr) (*(volatile uint32_t *)(addr))
#define RCC_APB1ENR REG(0x40023840UL)
#define TIM5_CR1    REG(0x40000C00UL)
#define TIM5_EGR    REG(0x40000C14UL)
#define TIM5_PSC    REG(0x40000C28UL)
#define TIM5_ARR    REG(0x40000C2CUL)
#define TIM5_CNT    REG(0x40000C24UL)

#if (TIME_PCLK_HZ % 1000000UL) != 0
#error "TIME_PCLK_HZ must be divisible by 1 MHz"
#endif

void Time_Init(void)
{
    RCC_APB1ENR |= (1U << 3); /* TIM5 peripheral clock */
    (void)RCC_APB1ENR;
    TIM5_CR1 = 0U;
    TIM5_PSC = (TIME_PCLK_HZ / 1000000UL) - 1U;
    TIM5_ARR = 0xFFFFFFFFUL; /* TIM5 is a 32-bit timer */
    TIM5_EGR = 1U;           /* Load the prescaler */
    TIM5_CNT = 0U;
    TIM5_CR1 = 1U;           /* Start timer */
}

uint32_t Time_Micros(void)
{
    return TIM5_CNT;
}
