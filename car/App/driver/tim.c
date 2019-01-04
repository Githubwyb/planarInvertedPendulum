#include "tim.h"
#include "main_control.h"

uint32_t time_4ms = 0;

void TIM_4ms_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);            //TIM7时钟使能
      
    TIM_TimeBaseStructure.TIM_Prescaler = 83;                       //定时器分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //向上计数模式
    TIM_TimeBaseStructure.TIM_Period = 3999;                        //自动重装载值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    
    TIM_TimeBaseInit(TIM7,&TIM_TimeBaseStructure);
        
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);                       //允许更新中断 ,允许CC1IE捕获中断
    
    TIM_Cmd(TIM7, ENABLE);                                           //使能定时器7
 
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 3;         //抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;               //子优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                  //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                                  //根据指定的参数初始化NVIC寄存器
}

void TIM7_IRQHandler(void)
{
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update); //清除中断标志位
    time_4ms++;
    Control_4ms();
}
