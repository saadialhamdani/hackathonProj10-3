#ifndef MOTOR_H
#define MOTOR_H

/* NUCLEO-F401RE -> TB6612FNG (your wiring):
 * Motor A (left): PWMA=D10/PB6/TIM4_CH1, AIN1=D11/PA7, AIN2=D12/PA6
 * Motor B (right): PWMB=D6/PB10/TIM2_CH3, BIN1=D5/PB4, BIN2=D7/PA8
 * STBY and VCC = 3.3 V; VM = +6 V motor battery (if motors rated 6 V).
 * All grounds, including battery negative, MUST be common.
 * AO1/AO2 and BO1/BO2 connect to motor pairs, not battery power.
 */
#define LEFT_INVERT   0 /* Change to 1 if left wheel runs backwards */
#define RIGHT_INVERT  0 /* Change to 1 if right wheel runs backwards */

#define MOTOR_TIMER_HZ 16000000UL /* HSI clock, APB1 prescaler = 1 */
#define MOTOR_PWM_HZ     20000UL
#define MOTOR_DRIVE_PCT   50     /* Adjust after wheels-off-ground test */

void Motors_Init(void);
void Motors_Set(int left_pct, int right_pct); /* Range -100 to 100 */
void Motors_Stop(void);

#endif /* MOTOR_H */
