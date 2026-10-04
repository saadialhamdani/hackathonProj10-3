#include "motor.h"
#include <stdint.h>

#if MOTOR_TIMER_HZ % MOTOR_PWM_HZ != 0
#error "Choose PWM frequency that divides MOTOR_TIMER_HZ"
#endif

#define PWM_TICKS (MOTOR_TIMER_HZ / MOTOR_PWM_HZ)
#define REG(addr) (*(volatile uint32_t *)(addr))

#define RCC_AHB1ENR REG(0x40023830UL)
#define RCC_APB1ENR REG(0x40023840UL)
#define GPIOA_MODER REG(0x40020000UL)
#define GPIOA_OTYPER REG(0x40020004UL)
#define GPIOA_AFRL REG(0x40020020UL)
#define GPIOA_BSRR REG(0x40020018UL)
#define GPIOB_MODER REG(0x40020400UL)
#define GPIOB_OTYPER REG(0x40020404UL)
#define GPIOB_AFRL REG(0x40020420UL)
#define GPIOB_AFRH REG(0x40020424UL)
#define GPIOB_BSRR REG(0x40020418UL)

/* PWMA PB6 = TIM4_CH1 (AF2); PWMB PB10 = TIM2_CH3 (AF1). */
#define TIM4_CR1   REG(0x40000800UL)
#define TIM4_EGR   REG(0x40000814UL)
#define TIM4_CCMR1 REG(0x40000818UL)
#define TIM4_CCER  REG(0x40000820UL)
#define TIM4_PSC   REG(0x40000828UL)
#define TIM4_ARR   REG(0x4000082CUL)
#define TIM4_CCR1  REG(0x40000834UL)

#define TIM2_CR1   REG(0x40000000UL)
#define TIM2_EGR   REG(0x40000014UL)
#define TIM2_CCMR2 REG(0x4000001CUL)
#define TIM2_CCER  REG(0x40000020UL)
#define TIM2_PSC   REG(0x40000028UL)
#define TIM2_ARR   REG(0x4000002CUL)
#define TIM2_CCR3  REG(0x4000003CUL)

/* GPIO BSRR atomically sets high bits and resets low bits. */
static void left_direction(int speed)
{
    /* Left AIN1 = PA7, AIN2 = PA6. */
    if (LEFT_INVERT) speed = -speed;
    if (speed > 0) GPIOA_BSRR = (1U << 7) | (1U << (6 + 16));
    else if (speed < 0) GPIOA_BSRR = (1U << 6) | (1U << (7 + 16));
    else GPIOA_BSRR = (1U << (6 + 16)) | (1U << (7 + 16));
}

static void right_direction(int speed)
{
    /* Right BIN1 = PB4, BIN2 = PA8. */
    if (RIGHT_INVERT) speed = -speed;
    if (speed > 0) {
        GPIOB_BSRR = (1U << 4);
        GPIOA_BSRR = (1U << (8 + 16));
    } else if (speed < 0) {
        GPIOB_BSRR = (1U << (4 + 16));
        GPIOA_BSRR = (1U << 8);
    } else {
        GPIOB_BSRR = (1U << (4 + 16));
        GPIOA_BSRR = (1U << (8 + 16));
    }
}

void Motors_Init(void)
{
    RCC_AHB1ENR |= (1U << 0) | (1U << 1); /* GPIOA and GPIOB */
    RCC_APB1ENR |= (1U << 0) | (1U << 2); /* TIM2 and TIM4 */
    (void)RCC_AHB1ENR;
    (void)RCC_APB1ENR;

    /* Force direction signals LOW before enabling pin outputs. */
    GPIOA_BSRR = (1U << (6 + 16)) | (1U << (7 + 16)) |
                 (1U << (8 + 16));
    GPIOB_BSRR = (1U << (4 + 16));

    /* PA6, PA7, PA8 = direction GPIO outputs. */
    GPIOA_MODER &= ~((3U << 12) | (3U << 14) | (3U << 16));
    GPIOA_MODER |=  ((1U << 12) | (1U << 14) | (1U << 16));
    GPIOA_OTYPER &= ~((1U << 6) | (1U << 7) | (1U << 8));

    /* PB4 = direction output. PB6 and PB10 = timer outputs. */
    GPIOB_MODER &= ~((3U << 8) | (3U << 12) | (3U << 20));
    GPIOB_MODER |=  ((1U << 8) | (2U << 12) | (2U << 20));
    GPIOB_OTYPER &= ~((1U << 4) | (1U << 6) | (1U << 10));
    GPIOB_AFRL &= ~(15U << 24);
    GPIOB_AFRL |=  (2U << 24); /* PB6 = AF2 = TIM4_CH1 */
    GPIOB_AFRH &= ~(15U << 8);
    GPIOB_AFRH |=  (1U << 8);  /* PB10 = AF1 = TIM2_CH3 */

    /* Motor A: TIM4 channel 1, PWM mode 1, 20 kHz. */
    TIM4_CR1 = 0U;
    TIM4_PSC = 0U;                 /* 16 MHz timer clock */
    TIM4_ARR = PWM_TICKS - 1U;
    TIM4_CCR1 = 0U;               /* Start stopped */
    TIM4_CCMR1 = (6U << 4) | (1U << 3); /* PWM1 + preload */
    TIM4_CCER = 1U;               /* Enable CH1 output */
    TIM4_EGR = 1U;
    TIM4_CR1 = (1U << 7) | 1U;    /* ARR preload + run */

    /* Motor B: TIM2 channel 3, PWM mode 1, 20 kHz. */
    TIM2_CR1 = 0U;
    TIM2_PSC = 0U;
    TIM2_ARR = PWM_TICKS - 1U;
    TIM2_CCR3 = 0U;
    TIM2_CCMR2 = (6U << 4) | (1U << 3); /* PWM1 + preload */
    TIM2_CCER = (1U << 8);        /* Enable CH3 output */
    TIM2_EGR = 1U;
    TIM2_CR1 = (1U << 7) | 1U;

    Motors_Stop();
}

void Motors_Set(int left_pct, int right_pct)
{
    if (left_pct > 100) left_pct = 100;
    if (left_pct < -100) left_pct = -100;
    if (right_pct > 100) right_pct = 100;
    if (right_pct < -100) right_pct = -100;

    /* Remove PWM before changing direction. STBY is wired permanently HIGH. */
    TIM4_CCR1 = 0U;
    TIM2_CCR3 = 0U;
    left_direction(left_pct);
    right_direction(right_pct);

    int left_duty = left_pct < 0 ? -left_pct : left_pct;
    int right_duty = right_pct < 0 ? -right_pct : right_pct;
    TIM4_CCR1 = (PWM_TICKS * (uint32_t)left_duty) / 100U;
    TIM2_CCR3 = (PWM_TICKS * (uint32_t)right_duty) / 100U;
}

void Motors_Stop(void)
{
    Motors_Set(0, 0);
}
