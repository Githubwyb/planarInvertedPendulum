#include <public.h>

void STM32_Init()
{
    USART1_Init(115200, 1);
    KEY_Init();
    LED_Init();
}

