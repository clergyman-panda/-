#ifndef __LTEMODULE_H
#define	__LTEMODULE_H


#include "stm32f10x.h"
#include <stdio.h>





// 串口2-USART2
#define  ltemodule_USARTx                   USART2
#define  ltemodule_USART_CLK                RCC_APB1Periph_USART2
#define  ltemodule_USART_APBxClkCmd         RCC_APB1PeriphClockCmd
#define  ltemodule_USART_BAUDRATE           115200

//// USART GPIO 引脚宏定义
#define  ltemodule_USART_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  ltemodule_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  ltemodule_USART_TX_GPIO_PORT       GPIOA   
#define  ltemodule_USART_TX_GPIO_PIN        GPIO_Pin_2
#define  ltemodule_USART_RX_GPIO_PORT       GPIOA
#define  ltemodule_USART_RX_GPIO_PIN        GPIO_Pin_3

#define  ltemodule_USART_IRQ                USART2_IRQn
#define  ltemodule_USART_IRQHandler         USART2_IRQHandler

typedef struct{
	float 		pitch;
	float			roll;
	float			yaw; 		//欧拉角
	uint32_t	timestamp;
}EuleranglesTypeDef;


typedef struct{
	uint8_t 	VehSpeed;
	float			VBAT;
	uint16_t	RPM;
	float			throttle_opening;
	float			engine_load;
	float			ECT;
	uint8_t			POWER;
	float			residual_oil_volume;
	double		Longitude;
	double		Latitude;
	EuleranglesTypeDef	Eulerangles;
	uint32_t	timestamp;
	
}Info_1secTypeDef;


extern uint64_t uinx_timestamp;


void ltemodule_USART_Config(void);
void ltemodule_Usart_SendByte( uint8_t ch);
void ltemodule_Usart_SendString(char *str);
void ltemodule_Usart_SendHalfWord(uint16_t ch);
void ltemodule_MQTT_init(void);
uint64_t get_timestamp(char *str);



uint8_t jlink_send_1s(Info_1secTypeDef Info,char *retstr);
uint8_t jlink_send_1s_test(Info_1secTypeDef Info);
uint8_t jlink_Eulerangles(Info_1secTypeDef Info,char *retstr);
#endif /* __LTEMODULE_H */
