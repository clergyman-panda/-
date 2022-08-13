#ifndef __USART_H
#define	__USART_H


#include "stm32f10x.h"
#include <stdio.h>
#include "JY901.h"

/** 
  * 串口宏定义，不同的串口挂载的总线和IO不一样，移植时需要修改这几个宏
	* 1-修改总线时钟的宏，uart1挂载到apb2总线，其他uart挂载到apb1总线
	* 2-修改GPIO的宏
  */
	
/*--------------------------------------------------*/
// Debug-UART1
#define  DEBUG_USARTx                   USART1
#define  DEBUG_USART_CLK                RCC_APB2Periph_USART1
#define  DEBUG_USART_APBxClkCmd         RCC_APB2PeriphClockCmd
#define  DEBUG_USART_BAUDRATE           115200

// Debug-UART1 引脚宏定义
#define  DEBUG_USART_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  DEBUG_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  DEBUG_USART_TX_GPIO_PORT       GPIOA   
#define  DEBUG_USART_TX_GPIO_PIN        GPIO_Pin_9
#define  DEBUG_USART_RX_GPIO_PORT       GPIOA
#define  DEBUG_USART_RX_GPIO_PIN        GPIO_Pin_10

#define  DEBUG_USART_IRQ                USART1_IRQn
#define  DEBUG_USART_IRQHandler         USART1_IRQHandler
/*--------------------------------------------------*/
// ATKBLE02-UART3
#define  ATKBLE02_USARTx                   	USART3
#define  ATKBLE02_USART_CLK                	RCC_APB1Periph_USART3
#define  ATKBLE02_USART_APBxClkCmd         	RCC_APB1PeriphClockCmd
#define  ATKBLE02_USART_BAUDRATE           	115200

//ATKBLE02-UART3引脚宏定义
#define  ATKBLE02_USART_GPIO_CLK           	(RCC_APB2Periph_GPIOB)
#define  ATKBLE02_USART_GPIO_APBxClkCmd    	RCC_APB2PeriphClockCmd
    
#define  ATKBLE02_USART_TX_GPIO_PORT       	GPIOB   
#define  ATKBLE02_USART_TX_GPIO_PIN        	GPIO_Pin_10
#define  ATKBLE02_USART_RX_GPIO_PORT       	GPIOB
#define  ATKBLE02_USART_RX_GPIO_PIN        	GPIO_Pin_11

#define  ATKBLE02_USART_IRQ                USART3_IRQn
#define  ATKBLE02_USART_IRQHandler         USART3_IRQHandler
/*--------------------------------------------------*/

//GPS-USART2
#define  GPS_USARTx                   	USART2
#define  GPS_USART_CLK                	RCC_APB1Periph_USART2
#define  GPS_USART_APBxClkCmd         	RCC_APB1PeriphClockCmd
#define  GPS_USART_BAUDRATE           	9600

//GPS-USART2引脚宏定义
#define  GPS_USART_GPIO_CLK           	(RCC_APB2Periph_GPIOA)
#define  GPS_USART_GPIO_APBxClkCmd    	RCC_APB2PeriphClockCmd
    
#define  GPS_USART_TX_GPIO_PORT       	GPIOA  
#define  GPS_USART_TX_GPIO_PIN        	GPIO_Pin_2
#define  GPS_USART_RX_GPIO_PORT       	GPIOA
#define  GPS_USART_RX_GPIO_PIN        	GPIO_Pin_3

#define  GPS_USART_IRQ                USART2_IRQn
#define  GPS_USART_IRQHandler         USART2_IRQHandler

/*--------------------------------------------------*/
//MPU-UART5
 #define  MPU_USARTx                   	UART5
#define  MPU_USART_CLK                	RCC_APB1Periph_UART5
#define  MPU_USART_APBxClkCmd         	RCC_APB1PeriphClockCmd
#define  MPU_USART_BAUDRATE           	9600

#define  MPU_USART_GPIO_CLK           	(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD)
#define  MPU_USART_GPIO_APBxClkCmd    	RCC_APB2PeriphClockCmd
    
#define  MPU_USART_TX_GPIO_PORT       	GPIOC   
#define  MPU_USART_TX_GPIO_PIN        	GPIO_Pin_12
#define  MPU_USART_RX_GPIO_PORT       	GPIOD
#define  MPU_USART_RX_GPIO_PIN        	GPIO_Pin_2

#define  MPU_USART_IRQ                	UART5_IRQn
#define  MPU_USART_IRQHandler         	UART5_IRQHandler

/*--------------------------------------------------*/

/*--------------------------------------------------*/
//GSP解析
//定义数组长度
#define GPS_Buffer_Length 80
#define UTCTime_Length 11
#define latitude_Length 11
#define N_S_Length 2
#define longitude_Length 12
#define E_W_Length 2 
typedef struct SaveData 
{
	char GPS_Buffer[GPS_Buffer_Length];
	char isGetData;		//是否获取到GPS数据
	char isParseData;	//是否解析完成
	char UTCTime[UTCTime_Length];		//UTC时间
	char latitude[latitude_Length];		//纬度
	char N_S[N_S_Length];		//N/S
	char longitude[longitude_Length];		//经度
	char E_W[E_W_Length];		//E/W
	char isUsefull;		//定位信息是否有效
} _SaveData;

typedef struct  
{
	char latitude[latitude_Length];		//纬度
	//char N_S[N_S_Length];		//N/S
	char longitude[longitude_Length];		//经度
	//char E_W[E_W_Length];		//E/W
	double Longitude;
	double Latitude;
	double Altitude;
	char CoordinateSystem;
} _SendGPS;

extern struct STime		stcTime;
extern struct SAcc 		stcAcc;
extern struct SGyro 		stcGyro;
extern struct SAngle 	stcAngle;
extern struct SMag 		stcMag;
extern struct SDStatus stcDStatus;
extern struct SPress 	stcPress;
extern struct SLonLat 	stcLonLat;
extern struct SGPSV 		stcGPSV;
extern struct SQ       stcQ;



void ALL_UART_Config(void);

void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void Usart_SendString( USART_TypeDef * pUSARTx, char *str);
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);
void Usart_SendArray( USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num);

void parseGpsBuffer(void);
void printGpsBuffer(void);
void convertGps(_SaveData *befconvert,_SendGPS *sendGPS);
void CopeSerial2Data(unsigned char ucData);
void sendcmd(char cmd[]);
void Atkble02Rec(unsigned char ucData);

void ATKBLE02_init();

#endif /* __USART_H */
