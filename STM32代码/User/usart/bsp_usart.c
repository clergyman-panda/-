/**
  ******************************************************************************
  * @file    bsp_usart.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   重定向c库printf函数到usart端口
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火STM32 F103-MINI 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
	
#include "bsp_usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "beep.h"
//#include "JY901.h"

extern QueueHandle_t UART_Queue;
 /**
  * @brief  配置嵌套向量中断控制器NVIC
  * @param  无
  * @retval 无
  */
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
	
	/* 嵌套向量中断控制器组选择 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
/*--------------------------------------------------*/ 
//DEBUG 配置
  /* 配置USART为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
  /* 抢断优先级*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  /* 子优先级 */
	//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  /* 使能中断 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* 初始化配置NVIC */
  NVIC_Init(&NVIC_InitStructure);
	
/*--------------------------------------------------*/
//ATKBLE02 配置
  NVIC_InitStructure.NVIC_IRQChannel = ATKBLE02_USART_IRQ;
  /* 抢断优先级*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  /* 使能中断 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* 初始化配置NVIC */
  NVIC_Init(&NVIC_InitStructure);
/*--------------------------------------------------*/
//GPS 配置
  NVIC_InitStructure.NVIC_IRQChannel = GPS_USART_IRQ;
  /* 抢断优先级*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  /* 使能中断 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* 初始化配置NVIC */
  NVIC_Init(&NVIC_InitStructure);
	/*--------------------------------------------------*/
//MPU 配置
  NVIC_InitStructure.NVIC_IRQChannel = MPU_USART_IRQ;
  /* 抢断优先级*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  /* 使能中断 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* 初始化配置NVIC */
  NVIC_Init(&NVIC_InitStructure);
	
	
}

 /**
  * @brief  USART GPIO 配置,工作参数配置
  * @param  无
  * @retval 无
  */
void ALL_UART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// 打开串口GPIO的时钟
	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);
	ATKBLE02_USART_GPIO_APBxClkCmd(ATKBLE02_USART_GPIO_CLK,ENABLE);
	GPS_USART_GPIO_APBxClkCmd(GPS_USART_GPIO_CLK,ENABLE);
	MPU_USART_GPIO_APBxClkCmd(MPU_USART_GPIO_CLK,ENABLE);
	
	// 打开串口外设的时钟
	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);
	ATKBLE02_USART_APBxClkCmd(ATKBLE02_USART_CLK, ENABLE);
	GPS_USART_APBxClkCmd(GPS_USART_CLK, ENABLE);
	MPU_USART_APBxClkCmd(MPU_USART_CLK, ENABLE);

/*--------------------------------------------------*/
	//DEBUG_UART GPIO配置
	// 将USART Tx的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);
  // 将USART Rx的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
/*--------------------------------------------------*/
	//ATKBLE_UART GPIO配置
	// 将ATKBLE_UART的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = ATKBLE02_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ATKBLE02_USART_TX_GPIO_PORT, &GPIO_InitStructure);
  // 将ATKBLE_UART的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = ATKBLE02_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(ATKBLE02_USART_RX_GPIO_PORT, &GPIO_InitStructure);
/*--------------------------------------------------*/
	//GPS_UART GPIO配置
	// 将USART Tx的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = GPS_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPS_USART_TX_GPIO_PORT, &GPIO_InitStructure);
  // 将USART Rx的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = GPS_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPS_USART_RX_GPIO_PORT, &GPIO_InitStructure);
/*--------------------------------------------------*/
	//MPU_UART GPIO配置
	// 将USART Tx的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = MPU_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MPU_USART_TX_GPIO_PORT, &GPIO_InitStructure);
  // 将USART Rx的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = MPU_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(MPU_USART_RX_GPIO_PORT, &GPIO_InitStructure);
/*--------------------------------------------------*/
// 配置串口的工作参数
/*--------------------------------------------------*/
	//DEBUG UART配置
	// 配置波特率
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	// 配置 针数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// 配置停止位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// 配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl = 
	USART_HardwareFlowControl_None;
	// 配置工作模式，收发一起
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// 完成串口的初始化配置
	USART_Init(DEBUG_USARTx, &USART_InitStructure);
/*--------------------------------------------------*/
	//ATKBLE02 UART配置
	// 配置波特率
	USART_InitStructure.USART_BaudRate = ATKBLE02_USART_BAUDRATE;
	// 配置 针数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// 配置停止位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// 配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// 配置工作模式，收发一起
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// 完成串口的初始化配置
	USART_Init(ATKBLE02_USARTx, &USART_InitStructure);
/*--------------------------------------------------*/
	//GPS UART配置
	// 配置波特率
	USART_InitStructure.USART_BaudRate = GPS_USART_BAUDRATE;
	// 配置 针数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// 配置停止位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// 配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// 配置工作模式，收发一起
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// 完成串口的初始化配置
	USART_Init(GPS_USARTx, &USART_InitStructure);
/*--------------------------------------------------*/
	//MPU UART配置
	// 配置波特率
	USART_InitStructure.USART_BaudRate = MPU_USART_BAUDRATE;
	// 配置 针数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// 配置停止位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// 配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// 配置工作模式，收发一起
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// 完成串口的初始化配置
	USART_Init(MPU_USARTx, &USART_InitStructure);
/*--------------------------------------------------*/	
	// 串口中断优先级配置
	NVIC_Configuration();
	
	// 使能串口接收中断
	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);	
	USART_ITConfig(ATKBLE02_USARTx, USART_IT_RXNE, ENABLE);	
	USART_ITConfig(GPS_USARTx, USART_IT_RXNE, ENABLE);	
	USART_ITConfig(MPU_USARTx, USART_IT_RXNE, ENABLE);	
	// 使能串口
	USART_Cmd(DEBUG_USARTx, ENABLE);	    
	USART_Cmd(ATKBLE02_USARTx, ENABLE);	
	USART_Cmd(GPS_USARTx, ENABLE);	
	USART_Cmd(MPU_USARTx, ENABLE);	
}

/*--------------------------------------------------*/	
//ATKBLE02_USART 中断服务函数
extern QueueHandle_t ATKBLE02_Queue;
void ATKBLE02_USART_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	char cChar;	
	if( USART_GetITStatus( ATKBLE02_USARTx, USART_IT_RXNE ) == SET )
	{
		cChar = USART_ReceiveData( ATKBLE02_USARTx );
		xQueueSendFromISR( ATKBLE02_Queue, &cChar, &xHigherPriorityTaskWoken );
		//Usart_SendByte( ATKBLE02_USARTx, cChar);
		USART_ClearFlag(ATKBLE02_USARTx, USART_IT_RXNE);
	}	
	
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

}
/*--------------------------------------------------*/	
//GPS_USART 中断服务函数
extern QueueHandle_t GPS_Queue;
void GPS_USART_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	char cChar;	
	if( USART_GetITStatus( GPS_USARTx, USART_IT_RXNE ) == SET )
	{
		cChar = USART_ReceiveData( GPS_USARTx );
		//printf("%c",cChar);
		xQueueSendFromISR( GPS_Queue, &cChar, &xHigherPriorityTaskWoken );
		USART_ClearFlag(GPS_USARTx, USART_IT_RXNE);
	}	
	
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

}

/*--------------------------------------------------*/	
//MPU_UART 中断服务函数
extern QueueHandle_t MPU_Queue;
void MPU_USART_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	char cChar;	
	if( USART_GetITStatus( MPU_USARTx, USART_IT_RXNE ) == SET )
	{
		cChar = USART_ReceiveData( MPU_USARTx );
		//printf("%c",cChar);
		xQueueSendFromISR( MPU_Queue, &cChar, &xHigherPriorityTaskWoken );
		USART_ClearFlag(MPU_USARTx, USART_IT_RXNE);
	}	
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}



/*****************  发送一个字节 **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* 发送一个字节数据到USART */
	USART_SendData(pUSARTx,ch);
		
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/****************** 发送8位的数组 ************************/
void Usart_SendArray( USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num)
{
  uint8_t i;
	
	for(i=0; i<num; i++)
  {
	    /* 发送一个字节数据到USART */
	    Usart_SendByte(pUSARTx,array[i]);	
  
  }
	/* 等待发送完成 */
	while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET);
}

/*****************  发送字符串 **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* 等待发送完成 */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}

/*****************  发送一个16位数 **********************/
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* 取出高八位 */
	temp_h = (ch&0XFF00)>>8;
	/* 取出低八位 */
	temp_l = ch&0XFF;
	
	/* 发送高八位 */
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
	
	/* 发送低八位 */
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

///重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到串口 */
		USART_SendData(DEBUG_USARTx, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		/* 等待串口输入数据 */
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(DEBUG_USARTx);
}


//void ATKBLE02_init()
//{
//	Usart_SendString(USART3,"+++a");
//	Usart_SendString(USART1,"AT+ENTM\r\n");
//}
struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SMag 		stcMag;
struct SDStatus stcDStatus;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SGPSV 		stcGPSV;
struct SQ       stcQ;

extern _SaveData Save_Data;
void parseGpsBuffer()
{
	char *subString;
	char *subStringNext;
	char i = 0;
	if (Save_Data.isGetData)
	{
		Save_Data.isGetData = 0;
		//printf("**************\r\n");
		printf("%s\r\n",Save_Data.GPS_Buffer);
	
		for (i = 0 ; i <= 6 ; i++)
		{
			if (i == 0)
			{
				if ((subString = strstr(Save_Data.GPS_Buffer, ",")) == NULL)
					//while(1){
						printf("ERROR%d\r\n",1);
					//}
				//errorLog(1);	//解析错误
			}
			else
			{
				subString++;
				if ((subStringNext = strstr(subString, ",")) != NULL)
				{
						char usefullBuffer[2]; 
						switch(i)
						{
							case 1:memcpy(Save_Data.UTCTime, subString, subStringNext - subString);break;	//获取UTC时间
							case 2:memcpy(usefullBuffer, subString, subStringNext - subString);break;	//获取UTC时间
							case 3:memcpy(Save_Data.latitude, subString, subStringNext - subString);break;	//获取纬度信息
							case 4:memcpy(Save_Data.N_S, subString, subStringNext - subString);break;	//获取N/S
							case 5:memcpy(Save_Data.longitude, subString, subStringNext - subString);break;	//获取经度信息
							case 6:memcpy(Save_Data.E_W, subString, subStringNext - subString);break;	//获取E/W
							default:break;
						}
						subString = subStringNext;
						Save_Data.isParseData = 1;
						if(usefullBuffer[0] == 'A')
							Save_Data.isUsefull = 1;
						else if(usefullBuffer[0] == 'V'){
							Save_Data.isUsefull = 0;
						}					
				}else
				{
					//while(1){
					//	vTaskDelay(2000);	
						printf("NMEA ERROR%d\r\n",1);
					//}
					//errorLog(2);	//解析错误
				}
			}


		}
	}
	if(Save_Data.isUsefull == 0)
				printf("gps数据无效%d\r\n",1);
}


void printGpsBuffer()
{
	char gps_sendbuf[128]={0};
	if (Save_Data.isParseData)
	{
		Save_Data.isParseData = 0;
		
		printf("Save_Data.UTCTime = ");
		printf("%s",Save_Data.UTCTime);
		printf("\r\n");

		if(Save_Data.isUsefull)
		{
			Save_Data.isUsefull = 0;
			printf("Save_Data.latitude = ");
			printf("%s",Save_Data.latitude);
			printf("\r\n");


			printf("Save_Data.N_S = ");
			printf("%s",Save_Data.N_S);
			printf("\r\n");

			printf("Save_Data.longitude = ");
			printf("%s",Save_Data.longitude);
			printf("\r\n");

			printf("Save_Data.E_W = ");
			printf("%s",Save_Data.E_W);
			printf("\r\n");
			sprintf(gps_sendbuf,"Latitude:%s\r\n",Save_Data.latitude);
			Usart_SendString(ATKBLE02_USARTx,gps_sendbuf);
			//memset(gps_sendbuf,0x00,128);
//			sprintf(gps_sendbuf,"Longitude:%s\r\n",Save_Data.longitude);
//			Usart_SendString(ATKBLE02_USARTx,gps_sendbuf);
		}
		else
		{
			printf("GPS DATA is not usefull!\r\n");
		}
		
	}
}



void convertGps(_SaveData *befconvert,_SendGPS *sendGPS){
		char latchar[12];
		char lonchar[12];
		//char gps_sendbuf[128]={0};
		memset(latchar,0x00,12);
		memset(lonchar,0x00,12);
		if(befconvert->isUsefull){
			strncpy(latchar,befconvert->latitude,11);
			strncpy(lonchar,befconvert->longitude,11);
			strncpy(sendGPS->latitude,befconvert->latitude,11);
			strncpy(sendGPS->longitude,befconvert->longitude,11);
			
			sendGPS->Latitude=atof(latchar);	
			sendGPS->Longitude=atof(lonchar);		
			//printf("latitude:%s\r\n",sendGPS->latitude);
			printf("longitude:%f\r\n",sendGPS->Latitude);
			//printf("latitude:%s\r\n",sendGPS->longitude);
			printf("latitude:%f\r\n",sendGPS->Longitude);
			//sprintf(gps_sendbuf,"Latitude:%f,Longitude:%f\r\n",sendGPS->Latitude,sendGPS->Longitude);
			//Usart_SendString(ATKBLE02_USARTx,gps_sendbuf);
		}
		return;
}


//CopeSerialData为串口5中断调用函数，串口每收到一个数据，调用一次这个函数。
void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	
	
	
	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
			case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
			case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
			case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
			case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
			case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
			case 0x59:	memcpy(&stcQ,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;//清空缓存区
	}
}

//用串口5给JY模块发送指令
void sendcmd(char cmd[])
{
	char i;
	for(i=0;i<5;i++)
		Usart_SendByte(MPU_USARTx, cmd[i]);
}


void Atkble02Rec(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	
	
	
	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!='$') //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<17) {return;}//数据不满11个，则返回
	else
	{
		if(strcmp((const char*)ucRxBuffer,"$FatigueDriving\r\n")==0)
		{
			BEEP=1;
			vTaskDelay(100);
			BEEP=0;
		}
		ucRxCnt=0;//清空缓存区
	}
}
