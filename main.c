#include "bluetooth.h"
#include "motor.h"
#include "timebase.h"
#include <stdint.h>
#include <string.h>

#define REG(addr) (*(volatile uint32_t *)(addr))
#define RCC_CR   REG(0x40023800UL)
#define RCC_CFGR REG(0x40023808UL)

#define MAX_COMMAND_MS 5000UL  /* Matches Python's 5-second chunks */

/* Select HSI 16 MHz and divide all buses by 1.
 * This sets the clock assumed by UART, TIM2, TIM4 and TIM5.
 */
static void Clock_Init16MHz(void)
{
    RCC_CR |= 1U; /* HSI ON */
    while (!(RCC_CR & (1U << 1))) { } /* HSI ready */
    RCC_CFGR &= ~3U; /* SYSCLK = HSI */
    while (RCC_CFGR & (3U << 2)) { } /* Wait for switch */
    RCC_CFGR &= ~((15U << 4) | (7U << 10) | (7U << 13));
}

/* Accept exactly F,<ms>, L,<ms> or R,<ms>, 1..5000 ms. */
static int Parse_Movement(const char *s, char *action, uint32_t *ms)
{
    if ((s[0] != 'F' && s[0] != 'L' && s[0] != 'R') ||
        s[1] != ',' || s[2] == '\0') return 0;

    uint32_t value = 0U;
    for (size_t i = 2U; s[i] != '\0'; ++i) {
        if (s[i] < '0' || s[i] > '9') return 0;
        uint32_t digit = (uint32_t)(s[i] - '0');
        if (value > (MAX_COMMAND_MS - digit) / 10U) return 0;
        value = value * 10U + digit;
    }
    if (value == 0U || value > MAX_COMMAND_MS) return 0;
    *action = s[0];
    *ms = value;
    return 1;
}

int main(void)
{
    char command[32];
    uint32_t movement_started = 0U;
    uint32_t movement_us = 0U;
    int moving = 0;

    Clock_Init16MHz();
    Time_Init();
    Motors_Init();
    Motors_Stop();
    Bluetooth_Init();

    for (;;) {
        /* Stop on time, even when TIM5's 32-bit counter wraps around. */
        if (moving &&
            (uint32_t)(Time_Micros() - movement_started) >= movement_us) {
            Motors_Stop();
            moving = 0;
            Bluetooth_SendLine("DONE");
        }

        if (!Bluetooth_ReadLine(command, sizeof(command))) continue;

        /* Stop works even if another command is running. */
        if (strcmp(command, "S") == 0) {
            Motors_Stop();
            moving = 0;
            Bluetooth_SendLine("DONE");
            continue;
        }
        if (moving) {
            Bluetooth_SendLine("BUSY");
            continue;
        }

        char action;
        uint32_t duration_ms;
        if (!Parse_Movement(command, &action, &duration_ms)) {
            Bluetooth_SendLine("ERR");
            continue;
        }

        if (action == 'F') Motors_Set(MOTOR_DRIVE_PCT, MOTOR_DRIVE_PCT);
        else if (action == 'L') Motors_Set(-MOTOR_DRIVE_PCT, MOTOR_DRIVE_PCT);
        else Motors_Set(MOTOR_DRIVE_PCT, -MOTOR_DRIVE_PCT);

        movement_started = Time_Micros();
        movement_us = duration_ms * 1000U;
        moving = 1;
    }
}
