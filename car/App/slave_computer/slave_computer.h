#ifndef _SLAVE_COMPUTER_H_
#define _SLAVE_COMPUTER_H_
#include <stm32f4xx.h>

void SHOW_Pages(void);
void Page_1(void);
void Page_2(void);
void Page_3(void);
void Page(uint8_t name[][255], int32_t data[][3], uint8_t n_data, uint8_t n_show);
uint8_t GET_touch_key(void);
void data_op(int32_t* data, int32_t data_max, int32_t data_min);


#endif
