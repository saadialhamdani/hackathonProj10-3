#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stddef.h>
#include <stdint.h>

/* STM32F401RE bare-metal Bluetooth configuration.
 * 2: USART2 PA2 (TX / D1), PA3 (RX / D0). Matches your present wiring,
 *    BUT NUCLEO-F401RE normally needs solder-bridge changes for D0/D1.
 * 1: USART1 PA9 (TX / D8), PA10 (RX / D2). Easier, no bridge changes.
 *    Set BT_USART to 1 AND move the HC-05 wires to D8/D2.
 */
#define BT_USART   2
#define BT_PCLK_HZ 16000000UL  /* Clock_Init16MHz() in main.c */
#define BT_BAUD    9600UL

#if (BT_USART != 1) && (BT_USART != 2)
#error "BT_USART must be 1 or 2"
#endif

void Bluetooth_Init(void);
void Bluetooth_Send(const char *message);
void Bluetooth_SendLine(const char *message);
int Bluetooth_ReadLine(char *buffer, size_t size);

#endif /* BLUETOOTH_H */
