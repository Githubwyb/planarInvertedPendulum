#include <motor.h>

#define PWM_MAX 4000

void MOTOR_Init(uint16_t prescaler, uint16_t period)
{
    //pwm初始化
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);         //TIM4时钟使能   
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);       //使能PORTB时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);       //使能PORTB时钟
    
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);      //GPIOB6复用为定时器4
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4);      //GPIOB7复用为定时器4
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM4);      //GPIOB8复用为定时器4
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;      //6/7/8
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;          //速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;              //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                //上拉
    GPIO_Init(GPIOB,&GPIO_InitStructure);                       //初始化PB6/PB7/PB8

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, GPIO_Pin_10);
    GPIO_ResetBits(GPIOC, GPIO_Pin_11);
    GPIO_ResetBits(GPIOC, GPIO_Pin_12);
      
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler - 1;        //定时器分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
    TIM_TimeBaseStructure.TIM_Period = period - 1;              //自动重装载值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);             //初始化定时器4
    
    //初始化TIM3 Channel1/2 PWM模式    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;           //选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   //输出极性:TIM输出比较极性高
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);                   //根据T指定的参数初始化外设TIM4 OC1
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);                   //根据T指定的参数初始化外设TIM4 OC2
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);                   //根据T指定的参数初始化外设TIM4 OC3

    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);          //使能TIM4在CCR1上的预装载寄存器
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);          //使能TIM4在CCR2上的预装载寄存器
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);          //使能TIM4在CCR3上的预装载寄存器

    TIM_ARRPreloadConfig(TIM4,ENABLE);                         //ARPE使能 
    TIM_Cmd(TIM4, ENABLE);                                     //使能TIM4

    TIM_SetCompare1(TIM4, 0);
    TIM_SetCompare2(TIM4, 0);
    TIM_SetCompare3(TIM4, 0);
}

void MOTOR0_Set(int32_t pwm)
{
    if(pwm >= 0)
    {
        TIM_SetCompare1(TIM4, pwm);
        GPIO_ResetBits(GPIOC, GPIO_Pin_10);
        TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Rising);      //CC1P=0 设置为上升沿捕获
    }
    else
    {
        TIM_SetCompare1(TIM4, PWM_MAX + pwm);
        GPIO_SetBits(GPIOC, GPIO_Pin_10);
        TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Falling);      //CC1P=0 设置为下降沿捕获
    }
}

void MOTOR1_Set(int32_t pwm)
{
    if(pwm >= 0)
    {
        TIM_SetCompare2(TIM4, pwm);
        GPIO_ResetBits(GPIOC, GPIO_Pin_11);
        TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Rising);      //CC1P=0 设置为上升沿捕获
    }
    else
    {
        TIM_SetCompare2(TIM4, PWM_MAX + pwm);
        GPIO_SetBits(GPIOC, GPIO_Pin_11);
        TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Falling);      //CC1P=0 设置为下降沿捕获
    }
}

void MOTOR2_Set(int32_t pwm)
{
    if(pwm >= 0)
    {
        TIM_SetCompare3(TIM4, pwm);
        GPIO_ResetBits(GPIOC, GPIO_Pin_12);
        TIM_OC2PolarityConfig(TIM5,TIM_ICPolarity_Rising);      //CC1P=0 设置为上升沿捕获
    }
    else
    {
        TIM_SetCompare3(TIM4, PWM_MAX + pwm);
        GPIO_SetBits(GPIOC, GPIO_Pin_12);
        TIM_OC2PolarityConfig(TIM5,TIM_ICPolarity_Falling);      //CC1P=0 设置为下降沿捕获
    }
}
