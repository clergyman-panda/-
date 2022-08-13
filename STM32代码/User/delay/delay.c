#include "delay.h"
#include <stdio.h>
// 基本定时器TIMx,x[6,7]定时初始化函数
volatile uint32_t time=0;

// 中断优先级配置
static void BASIC_TIM_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    // 设置中断组为0
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);		
		// 设置中断来源
    NVIC_InitStructure.NVIC_IRQChannel = BASIC_TIM_IRQ ;	
		// 设置主优先级为 0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	 
	  // 设置抢占优先级为3
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*
 * 注意：TIM_TimeBaseInitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
 * TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
 * 另外三个成员是通用定时器和高级定时器才有.
 *-----------------------------------------------------------------------------
 *typedef struct
 *{ TIM_Prescaler            都有
 *	TIM_CounterMode			     TIMx,x[6,7]没有，其他都有
 *  TIM_Period               都有
 *  TIM_ClockDivision        TIMx,x[6,7]没有，其他都有
 *  TIM_RepetitionCounter    TIMx,x[1,8,15,16,17]才有
 *}TIM_TimeBaseInitTypeDef; 
 *-----------------------------------------------------------------------------
 */


static void BASIC_TIM_Mode_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		
		// 开启定时器时钟,即内部时钟CK_INT=72M
    BASIC_TIM_APBxClock_FUN(BASIC_TIM_CLK, ENABLE);
	
		// 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
    TIM_TimeBaseStructure.TIM_Period = BASIC_TIM_Period;	

	  // 时钟预分频数为
    TIM_TimeBaseStructure.TIM_Prescaler= BASIC_TIM_Prescaler;
	
		// 时钟分频因子 ，基本定时器没有，不用管
    //TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
		
		// 计数器计数模式，基本定时器只能向上计数，没有计数模式的设置
    //TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
		
		// 重复计数器的值，基本定时器没有，不用管
		//TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	
	  // 初始化定时器
    TIM_TimeBaseInit(BASIC_TIM, &TIM_TimeBaseStructure);
		
		// 清除计数器中断标志位
    TIM_ClearFlag(BASIC_TIM, TIM_FLAG_Update);
	  
		// 开启计数器中断
    //TIM_ITConfig(BASIC_TIM,TIM_IT_Update,ENABLE);
		
		// 使能计数器
    TIM_Cmd(BASIC_TIM, ENABLE);	
}

void BASIC_TIM_Init(void)
{
	//BASIC_TIM_NVIC_Config();
	BASIC_TIM_Mode_Config();
}
/*********************************************END OF FILE**********************/

//void  BASIC_TIM_IRQHandler (void)
//{
//	if ( TIM_GetITStatus( BASIC_TIM, TIM_IT_Update) != RESET ) 
//	{	
//		time++;
//		TIM_ClearITPendingBit(BASIC_TIM , TIM_FLAG_Update);  		 
//	}		 	
//}

#if SYSTEM_SUPPORT_OS


#endif /*SYSTEM_SUPPORT_OS*/


			   


//延时nus
//nus为要延时的us数.		    								   
void delay_us(u32 nus)
{		
		//TIM_Cmd(BASIC_TIM, ENABLE);	
		BASIC_TIM->CNT=0;
		// 使能计数器  
		while((BASIC_TIM->CNT)<nus){};	
		return;
	
}
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对72M条件下,nms<=1864 
void delay_ms(uint16_t nms)
{	 		  	  
	uint32_t t=0;
//    uint32_t nowtime=time;
//		while(nowtime+nms>time){};	
	for(t=0;t<nms;t++){
		delay_us(1000);
	}	
		return;
} 



//uint16_t  0-65535
void delay_IIC(void)
{	 	
		uint16_t i=0;
		TIM_Cmd(BASIC_TIM, ENABLE);	
		BASIC_TIM->CNT=0;
		// 使能计数器  
		while((BASIC_TIM->CNT)<2){

		};	
		TIM_Cmd(BASIC_TIM, DISABLE);	
		return;
} 



