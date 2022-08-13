#ifndef __DELAY_H
#define __DELAY_H 			   
  
#include "stm32f10x.h"


/********************基本定时器TIM参数定义，只限TIM6************/

#define            BASIC_TIM                   TIM6
#define            BASIC_TIM_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            BASIC_TIM_CLK               RCC_APB1Periph_TIM6
#define            BASIC_TIM_Period            2000-1
#define            BASIC_TIM_Prescaler         71
#define            BASIC_TIM_IRQ               TIM6_IRQn
#define            BASIC_TIM_IRQHandler        TIM6_IRQHandler

/**************************函数声明********************************/

void BASIC_TIM_Init(void);
void delay_init(void);
void delay_ms(u16 nms);
void delay_us(u32 nus);
void delay_IIC(void);

#endif





























