#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>

/* TIM5 at 1 MHz: 1 tick = 1 microsecond; wraps after ~71.6 min.
 * Clock_Init16MHz() in main.c ensures APB1/TIM5 clock is 16 MHz.
 */
#define TIME_PCLK_HZ 16000000UL

void Time_Init(void);
uint32_t Time_Micros(void);

#endif /* TIMEBASE_H */
