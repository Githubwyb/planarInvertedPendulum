#include <public.h>

int main()
{
    STM32_Init();
    while(1)
    {
        SHOW_Pages();
        Control_main();
    }
}
