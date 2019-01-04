#include <public.h>

void STM32_Init(void)
{
    KEY_Init();
    LED_Init();
    USART1_Init(115200, 1);
    MOTOR_Init(1,4000);
    LCD_Init();
    TP_Init();
    ENCODER_Init();
    MPU6050_Init();
    TIM_4ms_Init();
    PID_param_Init();
}

