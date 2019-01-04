#ifndef _PUBLIC_H_
#define _PUBLIC_H_
#include <stm32f4xx.h>

//driver
#include "usart.h"
#include "tim.h"
#include "delay.h"
#include "i2c.h"

//device
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "motor.h"
#include "encoder.h"
#include "MPU6050.h"
#include "touch.h"

//algorithm
#include "handle.h"
#include "pid.h"

//slave_computer
#include "slave_computer.h"

//control.h
#include "main_control.h"

void STM32_Init(void);

#endif // _PUBLIC_H
