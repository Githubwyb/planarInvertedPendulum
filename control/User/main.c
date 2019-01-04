#include <public.h>

int main()
{
	uint8_t key = 3;
    while(1)
    {
        key = KEY_S(0);
        USART_Send(USART1, &key, 1);
    }
}
