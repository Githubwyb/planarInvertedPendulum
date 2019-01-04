#ifndef _MOTOR_H_
#define _MOTOR_H_
#include <stm32f4xx.h>

void MOTOR_Init(uint16_t prescaler, uint16_t period);
void MOTOR0_Set(int32_t pwm);
void MOTOR1_Set(int32_t pwm);
void MOTOR2_Set(int32_t pwm);

#endif // !_LED_H_
