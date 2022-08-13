/**
  ******************************************************************************
  * @file    bsp_usart.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   �ض���c��printf������usart�˿�
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:����STM32 F103-MINI ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
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
  * @brief  ����Ƕ�������жϿ�����NVIC
  * @param  ��
  * @retval ��
  */
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Ƕ�������жϿ�������ѡ�� */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
/*--------------------------------------------------*/ 
//DEBUG ����
  /* ����USARTΪ�ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
  /* �������ȼ�*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  /* �����ȼ� */
	//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  /* ʹ���ж� */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* ��ʼ������NVIC */
  NVIC_Init(&NVIC_InitStructure);
	
/*--------------------------------------------------*/
//ATKBLE02 ����
  NVIC_InitStructure.NVIC_IRQChannel = ATKBLE02_USART_IRQ;
  /* �������ȼ�*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  /* ʹ���ж� */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* ��ʼ������NVIC */
  NVIC_Init(&NVIC_InitStructure);
/*--------------------------------------------------*/
//GPS ����
  NVIC_InitStructure.NVIC_IRQChannel = GPS_USART_IRQ;
  /* �������ȼ�*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  /* ʹ���ж� */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* ��ʼ������NVIC */
  NVIC_Init(&NVIC_InitStructure);
	/*--------------------------------------------------*/
//MPU ����
  NVIC_InitStructure.NVIC_IRQChannel = MPU_USART_IRQ;
  /* �������ȼ�*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  /* ʹ���ж� */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* ��ʼ������NVIC */
  NVIC_Init(&NVIC_InitStructure);
	
	
}

 /**
  * @brief  USART GPIO ����,������������
  * @param  ��
  * @retval ��
  */
void ALL_UART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// �򿪴���GPIO��ʱ��
	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);
	ATKBLE02_USART_GPIO_APBxClkCmd(ATKBLE02_USART_GPIO_CLK,ENABLE);
	GPS_USART_GPIO_APBxClkCmd(GPS_USART_GPIO_CLK,ENABLE);
	MPU_USART_GPIO_APBxClkCmd(MPU_USART_GPIO_CLK,ENABLE);
	
	// �򿪴��������ʱ��
	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);
	ATKBLE02_USART_APBxClkCmd(ATKBLE02_USART_CLK, ENABLE);
	GPS_USART_APBxClkCmd(GPS_USART_CLK, ENABLE);
	MPU_USART_APBxClkCmd(MPU_USART_CLK, ENABLE);

/*--------------------------------------------------*/
	//DEBUG_UART GPIO����
	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);
  // ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
/*--------------------------------------------------*/
	//ATKBLE_UART GPIO����
	// ��ATKBLE_UART��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = ATKBLE02_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ATKBLE02_USART_TX_GPIO_PORT, &GPIO_InitStructure);
  // ��ATKBLE_UART��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = ATKBLE02_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(ATKBLE02_USART_RX_GPIO_PORT, &GPIO_InitStructure);
/*--------------------------------------------------*/
	//GPS_UART GPIO����
	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = GPS_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPS_USART_TX_GPIO_PORT, &GPIO_InitStructure);
  // ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = GPS_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPS_USART_RX_GPIO_PORT, &GPIO_InitStructure);
/*--------------------------------------------------*/
	//MPU_UART GPIO����
	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = MPU_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MPU_USART_TX_GPIO_PORT, &GPIO_InitStructure);
  // ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = MPU_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(MPU_USART_RX_GPIO_PORT, &GPIO_InitStructure);
/*--------------------------------------------------*/
// ���ô��ڵĹ�������
/*--------------------------------------------------*/
	//DEBUG UART����
	// ���ò�����
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	// ���� �������ֳ�
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// ����ֹͣλ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// ����У��λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// ����Ӳ��������
	USART_InitStructure.USART_HardwareFlowControl = 
	USART_HardwareFlowControl_None;
	// ���ù���ģʽ���շ�һ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(DEBUG_USARTx, &USART_InitStructure);
/*--------------------------------------------------*/
	//ATKBLE02 UART����
	// ���ò�����
	USART_InitStructure.USART_BaudRate = ATKBLE02_USART_BAUDRATE;
	// ���� �������ֳ�
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// ����ֹͣλ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// ����У��λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// ����Ӳ��������
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// ���ù���ģʽ���շ�һ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(ATKBLE02_USARTx, &USART_InitStructure);
/*--------------------------------------------------*/
	//GPS UART����
	// ���ò�����
	USART_InitStructure.USART_BaudRate = GPS_USART_BAUDRATE;
	// ���� �������ֳ�
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// ����ֹͣλ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// ����У��λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// ����Ӳ��������
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// ���ù���ģʽ���շ�һ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(GPS_USARTx, &USART_InitStructure);
/*--------------------------------------------------*/
	//MPU UART����
	// ���ò�����
	USART_InitStructure.USART_BaudRate = MPU_USART_BAUDRATE;
	// ���� �������ֳ�
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// ����ֹͣλ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// ����У��λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// ����Ӳ��������
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// ���ù���ģʽ���շ�һ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(MPU_USARTx, &USART_InitStructure);
/*--------------------------------------------------*/	
	// �����ж����ȼ�����
	NVIC_Configuration();
	
	// ʹ�ܴ��ڽ����ж�
	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);	
	USART_ITConfig(ATKBLE02_USARTx, USART_IT_RXNE, ENABLE);	
	USART_ITConfig(GPS_USARTx, USART_IT_RXNE, ENABLE);	
	USART_ITConfig(MPU_USARTx, USART_IT_RXNE, ENABLE);	
	// ʹ�ܴ���
	USART_Cmd(DEBUG_USARTx, ENABLE);	    
	USART_Cmd(ATKBLE02_USARTx, ENABLE);	
	USART_Cmd(GPS_USARTx, ENABLE);	
	USART_Cmd(MPU_USARTx, ENABLE);	
}

/*--------------------------------------------------*/	
//ATKBLE02_USART �жϷ�����
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
//GPS_USART �жϷ�����
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
//MPU_UART �жϷ�����
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



/*****************  ����һ���ֽ� **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* ����һ���ֽ����ݵ�USART */
	USART_SendData(pUSARTx,ch);
		
	/* �ȴ��������ݼĴ���Ϊ�� */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/****************** ����8λ������ ************************/
void Usart_SendArray( USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num)
{
  uint8_t i;
	
	for(i=0; i<num; i++)
  {
	    /* ����һ���ֽ����ݵ�USART */
	    Usart_SendByte(pUSARTx,array[i]);	
  
  }
	/* �ȴ�������� */
	while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET);
}

/*****************  �����ַ��� **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* �ȴ�������� */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}

/*****************  ����һ��16λ�� **********************/
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* ȡ���߰�λ */
	temp_h = (ch&0XFF00)>>8;
	/* ȡ���Ͱ�λ */
	temp_l = ch&0XFF;
	
	/* ���͸߰�λ */
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
	
	/* ���͵Ͱ�λ */
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

///�ض���c�⺯��printf�����ڣ��ض�����ʹ��printf����
int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ����ݵ����� */
		USART_SendData(DEBUG_USARTx, (uint8_t) ch);
		
		/* �ȴ�������� */
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

///�ض���c�⺯��scanf�����ڣ���д����ʹ��scanf��getchar�Ⱥ���
int fgetc(FILE *f)
{
		/* �ȴ������������� */
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
				//errorLog(1);	//��������
			}
			else
			{
				subString++;
				if ((subStringNext = strstr(subString, ",")) != NULL)
				{
						char usefullBuffer[2]; 
						switch(i)
						{
							case 1:memcpy(Save_Data.UTCTime, subString, subStringNext - subString);break;	//��ȡUTCʱ��
							case 2:memcpy(usefullBuffer, subString, subStringNext - subString);break;	//��ȡUTCʱ��
							case 3:memcpy(Save_Data.latitude, subString, subStringNext - subString);break;	//��ȡγ����Ϣ
							case 4:memcpy(Save_Data.N_S, subString, subStringNext - subString);break;	//��ȡN/S
							case 5:memcpy(Save_Data.longitude, subString, subStringNext - subString);break;	//��ȡ������Ϣ
							case 6:memcpy(Save_Data.E_W, subString, subStringNext - subString);break;	//��ȡE/W
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
					//errorLog(2);	//��������
				}
			}


		}
	}
	if(Save_Data.isUsefull == 0)
				printf("gps������Ч%d\r\n",1);
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


//CopeSerialDataΪ����5�жϵ��ú���������ÿ�յ�һ�����ݣ�����һ�����������
void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	
	
	
	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������
	if (ucRxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//���ݲ���11�����򷵻�
	else
	{
		switch(ucRxBuffer[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݽṹ�����棬�Ӷ�ʵ�����ݵĽ�����
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
		ucRxCnt=0;//��ջ�����
	}
}

//�ô���5��JYģ�鷢��ָ��
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
	
	
	
	
	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������
	if (ucRxBuffer[0]!='$') //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<17) {return;}//���ݲ���11�����򷵻�
	else
	{
		if(strcmp((const char*)ucRxBuffer,"$FatigueDriving\r\n")==0)
		{
			BEEP=1;
			vTaskDelay(100);
			BEEP=0;
		}
		ucRxCnt=0;//��ջ�����
	}
}
