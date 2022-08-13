/*
*************************************************************************
*                             ������ͷ�ļ�
*************************************************************************
*/ 
/* FreeRTOSͷ�ļ� */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* ������Ӳ��bspͷ�ļ� */
#include "bsp_led.h"
#include "bsp_usart.h"
#include "bsp_key.h"
#include "ltemodule.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "beep.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "delay.h"
#include <string.h>
#include "CAN.h"
//#include "JY901.h"
#include <math.h>

#define CAN_250K_11B         ((uint32_t)0x01) 
#define CAN_250K_29B         ((uint32_t)0x02) 
#define CAN_500K_11B         ((uint32_t)0x04) 
#define CAN_500K_29B         ((uint32_t)0x08) 

uint64_t uinx_timestamp;

#define get_timestamp_ms()         (uinx_timestamp*1000+(uint64_t)xTaskGetTickCount())
#define get_timestamp_s()         (uinx_timestamp+((uint64_t)xTaskGetTickCount())/1000)

uint32_t CAN_TYPE=0;
_SaveData Save_Data;
_SendGPS	GPSINFO;

void MPU6050_TEST(void);
/**************************** ������ ********************************/
/* 
 * ��������һ��ָ�룬����ָ��һ�����񣬵����񴴽���֮�����;�����һ��������
 * �Ժ�����Ҫ��������������Ҫͨ�������������������������������Լ�����ô
 * ����������ΪNULL��
 */
static TaskHandle_t AppTaskCreate_Handle = NULL;/* ���������� */
static TaskHandle_t OBDASKDATA_Task_Handle = NULL;
static TaskHandle_t OBDGETDATA_Task_Handle = NULL;
static TaskHandle_t ATKBLE02_Task_Handle = NULL;
static TaskHandle_t GPS_Task_Handle = NULL;
static TaskHandle_t MPU_Task_Handle=NULL;
static TaskHandle_t OBDTEST_Task_Handle = NULL;

/********************************** �ں˶����� *********************************/
/*
 * �ź�������Ϣ���У��¼���־�飬�����ʱ����Щ�������ں˵Ķ���Ҫ��ʹ����Щ�ں�
 * ���󣬱����ȴ����������ɹ�֮��᷵��һ����Ӧ�ľ����ʵ���Ͼ���һ��ָ�룬������
 * �ǾͿ���ͨ��������������Щ�ں˶���
 *
 * �ں˶���˵���˾���һ��ȫ�ֵ����ݽṹ��ͨ����Щ���ݽṹ���ǿ���ʵ��������ͨ�ţ�
 * �������¼�ͬ���ȸ��ֹ��ܡ�������Щ���ܵ�ʵ��������ͨ��������Щ�ں˶���ĺ���
 * ����ɵ�
 * 
 */

QueueHandle_t CANmsg_Queue =NULL;
QueueHandle_t ATKBLE02_Queue =NULL;
QueueHandle_t GPS_Queue =NULL;
//QueueHandle_t WIFI_Queue =NULL;
QueueHandle_t MPU_Queue =NULL;
SemaphoreHandle_t  BLESEND_Handle=NULL;


/******************************* ȫ�ֱ������� ************************************/
/*
 * ��������дӦ�ó����ʱ�򣬿�����Ҫ�õ�һЩȫ�ֱ�����
 */
Info_1secTypeDef  	Info_1sec_gal;

char OBDhasasked=0;
struct {
	double Longitude;
	double Latitude;
	double accy;
	uint8_t VSS;
}ACCCAL;
/******************************* �궨�� ************************************/
#define  MPU6050_QUEUE_LEN    10   /* ���еĳ��ȣ����ɰ������ٸ���Ϣ */
#define  MPU6050_QUEUE_SIZE   (sizeof(EuleranglesTypeDef))   /* ������ÿ����Ϣ��С���ֽڣ� */
#define  CANMSG_QUEUE_LEN    128   /* ���еĳ��ȣ����ɰ������ٸ���Ϣ */
#define  CANMSG_QUEUE_SIZE   (sizeof(CanRxMsg))   /* ������ÿ����Ϣ��С���ֽڣ� */
#define  ATKBLE02_QUEUE_LEN    100   /* ���еĳ��ȣ����ɰ������ٸ���Ϣ */
#define  ATKBLE02_QUEUE_SIZE   1   /* ������ÿ����Ϣ��С���ֽڣ� */
#define  GPS_QUEUE_LEN    100   /* ���еĳ��ȣ����ɰ������ٸ���Ϣ */
#define  GPS_QUEUE_SIZE   1   /* ������ÿ����Ϣ��С���ֽڣ� */


#define  MPU_QUEUE_LEN    1024   /* ���еĳ��ȣ����ɰ������ٸ���Ϣ */
#define  MPU_QUEUE_SIZE   1   /* ������ÿ����Ϣ��С���ֽڣ� */	
#define  WIFISEND_QUEUE_LEN    2048   /* ���еĳ��ȣ����ɰ������ٸ���Ϣ */
#define  WIFISEND_QUEUE_SIZE   1   /* ������ÿ����Ϣ��С���ֽڣ� */	
/*
*************************************************************************
*                             ��������
*************************************************************************
*/
static void AppTaskCreate(void);/* ���ڴ������� */
static void	OBD_askdata_Task(void* pvParameters);/* SendInfo_1s_Task */
static void	OBD_getdata_Task(void* pvParameters);/* SendInfo_1s_Task */

static void	GPS_Task(void* pvParameters);/* SendInfo_1s_Task */

static void	MPU_Task(void* pvParameters);/* SendInfo_1s_Task */
static void OBDTEST_Task(void* pvParameters);
static void	ATKBLE02_Task(void* pvParameters);/* SendInfo_1s_Task */


static void BSP_Init(void);/* ���ڳ�ʼ�����������Դ */
///*FreeRTOS-CLI��ʼ������*/
//extern void vUARTCommandConsoleStart( uint16_t usStackSize, UBaseType_t uxPriority );//FreeRTOS-CL���񴴽�
//extern	void vRegisterSampleCLICommands( void );//FreeRTOS-CLIָ��ע��

/*****************************************************************
  * @brief  ������
  * @param  ��
  * @retval ��
  * @note   ��һ����������Ӳ����ʼ�� 
            �ڶ���������APPӦ������
            ������������FreeRTOS����ʼ���������
  ****************************************************************/
int main(void)
{	
  BaseType_t xReturn = pdPASS;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */
  

	/* ������Ӳ����ʼ�� */
  BSP_Init();

   /* ����AppTaskCreate���� */
  xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate,  /* ������ں��� */
                        (const char*    )"AppTaskCreate",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )1, /* ��������ȼ� */
                        (TaskHandle_t*  )&AppTaskCreate_Handle);/* ������ƿ�ָ�� */ 


	
  /* ����������� */           
  if(pdPASS == xReturn)
    vTaskStartScheduler();   /* �������񣬿������� */
  else
    return -1;  
  

  while(1);   /* ��������ִ�е����� */    
}




/***********************************************************************
  * @ ������  �� AppTaskCreate
  * @ ����˵���� Ϊ�˷���������е����񴴽����������������������
  * @ ����    �� ��  
  * @ ����ֵ  �� ��
  **********************************************************************/
static void AppTaskCreate(void)
{
  BaseType_t xReturn = pdPASS;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */

  taskENTER_CRITICAL();           //�����ٽ���
  
//		BLESEND_Handle=xSemaphoreCreateMutex();
//		if(NULL != BLESEND_Handle)
//		printf("����BLESEND_Handle�����ź����ɹ�!\r\n");
//		
//		
//		xReturn=xSemaphoreGive(BLESEND_Handle);//�ͷŻ����ź���
		

		
		/* ����CANmsg_Queue */
		CANmsg_Queue = xQueueCreate((UBaseType_t ) CANMSG_QUEUE_LEN,/* ��Ϣ���еĳ��� */
															(UBaseType_t ) CANMSG_QUEUE_SIZE);/* ��Ϣ�Ĵ�С */
		if(NULL != CANmsg_Queue)
			//printf("����CANmsg_Queue���гɹ�!\r\n");
		
		/* ����ATKBLE02_Queue */
		ATKBLE02_Queue = xQueueCreate((UBaseType_t ) ATKBLE02_QUEUE_LEN,/* ��Ϣ���еĳ��� */
															(UBaseType_t ) ATKBLE02_QUEUE_SIZE);/* ��Ϣ�Ĵ�С */
		if(NULL != ATKBLE02_Queue)
		//printf("����ATKBLE02_Queue���гɹ�!\r\n");
		/* ����GPS_Queue */
		GPS_Queue = xQueueCreate((UBaseType_t ) GPS_QUEUE_LEN,/* ��Ϣ���еĳ��� */
															(UBaseType_t ) GPS_QUEUE_SIZE);/* ��Ϣ�Ĵ�С */
		if(NULL != GPS_Queue)
		//printf("����GPS_Queue���гɹ�!\r\n");
		/* ����MPU_Queue */

		MPU_Queue = xQueueCreate((UBaseType_t ) MPU_QUEUE_LEN,/* ��Ϣ���еĳ��� */
															(UBaseType_t ) MPU_QUEUE_SIZE);/* ��Ϣ�Ĵ�С */
		if(NULL != MPU_Queue)
		//printf("����MPU_Queue���гɹ�!\r\n");	
		


		/* ����OBD_askdata_Task */
	    xReturn = xTaskCreate((TaskFunction_t )OBD_askdata_Task, /* ������ں��� */
                        (const char*    )"OBD_getdata_Task",/* �������� */
                        (uint16_t       )512,   /* ����ջ��С */
                        (void*          )NULL,	/* ������ں������� */
                        (UBaseType_t    )8,	    /* ��������ȼ� */
                        (TaskHandle_t*  )&OBDASKDATA_Task_Handle);/* ������ƿ�ָ�� */
		if(pdPASS == xReturn)
		printf("����OBD_askdata_Task�ɹ�!\r\n");	
		
		/* ����OBD_getdata_Task */
	    xReturn = xTaskCreate((TaskFunction_t )OBD_getdata_Task, /* ������ں��� */
                        (const char*    )"OBD_getdata_Task",/* �������� */
                        (uint16_t       )512,   /* ����ջ��С */
                        (void*          )NULL,	/* ������ں������� */
                        (UBaseType_t    )8,	    /* ��������ȼ� */
                        (TaskHandle_t*  )&OBDGETDATA_Task_Handle);/* ������ƿ�ָ�� */
		if(pdPASS == xReturn)
		printf("����OBD_getdata_Task�ɹ�!\r\n");	
		
		/* ����GPS_Task */
//		xReturn = xTaskCreate((TaskFunction_t )GPS_Task, /* ������ں��� */
//						(const char*    )"GPS_Task",/* �������� */
//						(uint16_t       )512,   /* ����ջ��С */
//						(void*          )NULL,	/* ������ں������� */
//						(UBaseType_t    )8,	    /* ��������ȼ� */
//						(TaskHandle_t*  )&GPS_Task_Handle);/* ������ƿ�ָ�� */
//		if(pdPASS == xReturn)
//		printf("����GPS_Task�ɹ�!\r\n");
		/* ����MPU_Task */
//		xReturn = xTaskCreate((TaskFunction_t )MPU_Task, /* ������ں��� */
//													(const char*    )"MPU_Task",/* �������� */
//													(uint16_t       )512,   /* ����ջ��С */
//													(void*          )NULL,	/* ������ں������� */
//													(UBaseType_t    )8,	    /* ��������ȼ� */
//													(TaskHandle_t*  )&MPU_Task_Handle);/* ������ƿ�ָ�� */
//		if(pdPASS == xReturn)
//		printf("����MPU_Task�ɹ�!\r\n");
		
		
		/* ����ATKBLE02_Task */
		xReturn = xTaskCreate((TaskFunction_t )ATKBLE02_Task, /* ������ں��� */
													(const char*    )"ATKBLE02_Task",/* �������� */
													(uint16_t       )512,   /* ����ջ��С */
													(void*          )NULL,	/* ������ں������� */
													(UBaseType_t    )8,	    /* ��������ȼ� */
													(TaskHandle_t*  )&ATKBLE02_Task_Handle);/* ������ƿ�ָ�� */
		if(pdPASS == xReturn)
		printf("����ATKBLE02_Task�ɹ�!\r\n");
		
	
		
		/* ����OBDTEST_Task */
		xReturn = xTaskCreate((TaskFunction_t )OBDTEST_Task, /* ������ں��� */
													(const char*    )"OBDTEST_Task",/* �������� */
													(uint16_t       )512,   /* ����ջ��С */
													(void*          )NULL,	/* ������ں������� */
													(UBaseType_t    )2,	    /* ��������ȼ� */
													(TaskHandle_t*  )&OBDTEST_Task_Handle);/* ������ƿ�ָ�� */
		if(pdPASS == xReturn)
		printf("����OBDTEST_Task_Task�ɹ�!\r\n");
		
	
		
//	vRegisterSampleCLICommands(  );
//	vUARTCommandConsoleStart( 512, 3 );
	
  vTaskDelete(AppTaskCreate_Handle); //ɾ��AppTaskCreate���� 
  taskEXIT_CRITICAL();            //�˳��ٽ���

}

static void OBD_askdata_Task(void* parameter){
    BaseType_t xReturn = pdPASS;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */
	
	//Usart_SendString(ATKBLE02_USARTx, "OBD_askdata_Task\r\n");
	TickType_t pxPreviousWakeTime=0;//xTaskGetTickCount();
	uint8_t IDE;
	int i=0;
	vTaskDelay(1000);
	//CAN_TYPE = CAN_250K_29B;//CAN_250K_11B      CAN_250K_29B   CAN_500K_11B    CAN_500K_29B 

	if(CAN_TYPE==CAN_250K_11B||CAN_TYPE==CAN_250K_29B){
		CANinit(250);
	}else{
		CANinit(500);
	}
	if(CAN_TYPE == CAN_250K_11B||CAN_TYPE == CAN_500K_11B){
			IDE=CAN_Id_Standard;
	}else{
			IDE=CAN_Id_Extended;
	}
	pxPreviousWakeTime=xTaskGetTickCount();
  while (1)
  {
		printf("askdata_timestamp_ms:%lld\r\n",get_timestamp_ms());
		
		for(i=0;i<numofask+1;i++){//����i������֡
			if(OBDdataHz_new[i]==0)continue;
				OBDTransmit_askframe(newOBDdata[i].StdId,IDE,newOBDdata[i].SID,newOBDdata[i].PID);
				vTaskDelay(20);
				if(OBDhasasked==0)OBDhasasked=1;
		}
		//ģ��GPS��Ч���ݣ���ʹ��ʱҪ�ص�GPS���ڡ�
//		if(1){
//			char *send="$GNRMC,044717.000,A,2934.09367,N,10627.68745,E,0.00,0.00,140322,,,A*7A\n";
//			int len=strlen(send);
//			int i=0;
//			for(i=0;i<len;i++){
//				xQueueSend( GPS_Queue, &send[i] ,0);
//			}	
//		}


  }
}


/**********************************************************************
  * @ ������  	��GPS_Task
* @ ����˵��		:				
  * @ ����			��   
  * @ ����ֵ  	�� ��
  ********************************************************************/
//static void GPS_Task(void* parameter)
//{	 
//	BaseType_t xReturn = pdPASS;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */
//	char rechar[128];
//	
//	memset(rechar,0x00,128);
//	int point1=0;	
//	USART_ITConfig(GPS_USARTx, USART_IT_RXNE, ENABLE);
//	USART_Cmd(GPS_USARTx, ENABLE);	

//    while (1)
//	{	
//		xReturn = xQueueReceive( 		GPS_Queue,    /* ��Ϣ���еľ�� */
//																(rechar+point1),      /* ���͵���Ϣ���� */
//																portMAX_DELAY); /* �ȴ�ʱ�� һֱ�� */	
//		if(*(rechar+point1) == '$'){
//			point1 = 0;	
//			memset(rechar,0x00,128);
//			*(rechar+point1)='$';			
//		}
//		if(rechar[0] == '$' && rechar[4] == 'M' && rechar[5] == 'C')			//ȷ���Ƿ��յ�"GPRMC/GNRMC"��һ֡����
//		{
//			
//			if(*(rechar+point1) == '\n')									   
//			{
//				//char timestrbuff[128];
//				//uinxtostr(get_timestamp_s(),timestrbuff);
//				//printf("now time:%s\r\n",timestrbuff);
//				memset(Save_Data.GPS_Buffer, 0, GPS_Buffer_Length);      //���
//				memcpy(Save_Data.GPS_Buffer, rechar, point1); 	//��������
//				Save_Data.isGetData = 1;
//				parseGpsBuffer();
//				printGpsBuffer();
//				point1 = 0;
//				memset(rechar,0x00,128);
//				convertGps(&Save_Data,&GPSINFO);
//				ACCCAL.Latitude=GPSINFO.Latitude;
//				ACCCAL.Longitude=GPSINFO.Longitude;
////				sprintf(gps_sendbuf,"Latitude:%f,Longitude:%f\r\n",ACCCAL.Latitude,ACCCAL.Longitude);
////				Usart_SendString(ATKBLE02_USARTx,gps_sendbuf);

//				if(Save_Data.isUsefull == 1){
//					printf("GPS������Ч\r\n");
//															
//				}
//				
//				continue;
//			}		
//		}	
//		point1++;
//    }
//}


//OBDGETDATA_Task_Handle
static void OBD_getdata_Task(void* parameter){
  BaseType_t xReturn = pdPASS;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */
	TickType_t pxPreviousWakeTime=0;//xTaskGetTickCount();
	CanRxMsg tempCANRxMSG;
	while(OBDhasasked==0){	
		vTaskDelay(200);
	}
	
	//Usart_SendString(ATKBLE02_USARTx, "OBD_getdata_Task\r\n");	
	pxPreviousWakeTime=xTaskGetTickCount();
	
  while (1)
  {
		//printf("uxQueueMessagesWaiting:%d\r\n",(uint16_t)uxQueueMessagesWaiting(CANmsg_Queue));
		while(uxQueueMessagesWaiting(CANmsg_Queue)!=0){
			xReturn = xQueueReceive( 	CANmsg_Queue,    /* ��Ϣ���еľ�� */
										&tempCANRxMSG,      /* ���͵���Ϣ���� */
										portMAX_DELAY); /* portMAX_DELAY�ȴ�ʱ�� һֱ�� */	
			char dctbuf[20];
			if((tempCANRxMSG.Data[0]==0x10)&&(tempCANRxMSG.Data[2]==0x43)){//��������֡
				
							
				printf("DCT1:P%02x%02x\r\n",tempCANRxMSG.Data[4],tempCANRxMSG.Data[5]);
				sprintf(dctbuf,"DCT:P%02x%02x\r\n",tempCANRxMSG.Data[4],tempCANRxMSG.Data[5]);
				Usart_SendString(ATKBLE02_USARTx,dctbuf);
				memset(dctbuf,0,20);
				printf("DCT2:P%02x%02x\r\n",tempCANRxMSG.Data[6],tempCANRxMSG.Data[7]);
				sprintf(dctbuf,"DCT:P%02x%02x\r\n",tempCANRxMSG.Data[6],tempCANRxMSG.Data[7]);
				Usart_SendString(ATKBLE02_USARTx,dctbuf);
				memset(dctbuf,0,20);
				
				uint8_t senddata[8]={0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00};			
				CANTransmit_frame(0x7DF,CAN_Id_Standard,senddata,8);//��������֡ʹ����֡��������
				while(uxQueueMessagesWaiting(CANmsg_Queue)==0){	
				}
				xReturn = xQueueReceive( 	CANmsg_Queue,    /* ��Ϣ���еľ�� */
											&tempCANRxMSG,      /* ���͵���Ϣ���� */
											portMAX_DELAY); /* portMAX_DELAY�ȴ�ʱ�� һֱ�� */
				if(tempCANRxMSG.Data[0]==0x21)//����֡
				{	
					
					printf("DCT3:P%02x%02x\r\n",tempCANRxMSG.Data[1],tempCANRxMSG.Data[2]);
					sprintf(dctbuf,"DCT:P%02x%02x\r\n",tempCANRxMSG.Data[1],tempCANRxMSG.Data[2]);
					Usart_SendString(ATKBLE02_USARTx,dctbuf);
					memset(dctbuf,0,20);
					printf("DCT4:P%02x%02x\r\n",tempCANRxMSG.Data[3],tempCANRxMSG.Data[4]);
					sprintf(dctbuf,"DCT:P%02x%02x\r\n",tempCANRxMSG.Data[3],tempCANRxMSG.Data[4]);
					Usart_SendString(ATKBLE02_USARTx,dctbuf);
					memset(dctbuf,0,20);
				}
							
			}
			if(tempCANRxMSG.Data[1]==0x41){
				char sendbuf[20];
				for(int i=0;i<numofask;i++){
					if(tempCANRxMSG.Data[2]==newOBDdata[i].PID){
						if(strstr(newOBDdata[i].dataname,"VSS")!=NULL){
							
							printf("VSS:%d\r\n",tempCANRxMSG.Data[3]);
							sprintf(sendbuf,"VSS:%d\r\n",tempCANRxMSG.Data[3]);
							Usart_SendString(ATKBLE02_USARTx, sendbuf);
							memset(sendbuf,0,20);
						}
						if(strstr(newOBDdata[i].dataname,"RPM")!=NULL){
							printf("RPM:%d\r\n",(tempCANRxMSG.Data[3]*256+tempCANRxMSG.Data[4])/4);
							sprintf(sendbuf,"RPM:%d\r\n",(tempCANRxMSG.Data[3]*256+tempCANRxMSG.Data[4])/4);
							Usart_SendString(ATKBLE02_USARTx, sendbuf);
							memset(sendbuf,0,20);
							
						}
						if(strstr(newOBDdata[i].dataname,"LOAD_PCT")!=NULL){
							
							printf("LOAD_PCT:%d\r\n",100*tempCANRxMSG.Data[3]/255);
							sprintf(sendbuf,"LOAD_PCT:%d\r\n",100*tempCANRxMSG.Data[3]/255);
							Usart_SendString(ATKBLE02_USARTx, sendbuf);
							memset(sendbuf,0,20);
			
						}
						if(strstr(newOBDdata[i].dataname,"TP")!=NULL){
							
							printf("TP:%d\r\n",100*tempCANRxMSG.Data[3]/255);
							sprintf(sendbuf,"TP:%d\r\n",100*tempCANRxMSG.Data[3]/255);
							Usart_SendString(ATKBLE02_USARTx, sendbuf);
							memset(sendbuf,0,20);
						}
//						if(strstr(newOBDdata[i].dataname,"DTC_CNT")!=NULL){
//	
//							printf("DTC_CNT:%d\r\n",tempCANRxMSG.Data[3]&0X7F);
//							sprintf(sendbuf,"DTC_CNT:%d\r\n",tempCANRxMSG.Data[3]&0X7F);
//							Usart_SendString(ATKBLE02_USARTx, sendbuf);
//							memset(sendbuf,0,20);
//						}
					}
				}
			}
//			if(((tempCANRxMSG.Data[0]&0xf0)==20)&&((tempCANRxMSG.Data[0]&0x0f)>=1))//����֡
//			{	
//				printf("test2\r\n");
//				printf("DCT3:P%02x%02x\r\n",tempCANRxMSG.Data[1],tempCANRxMSG.Data[2]);
//				sprintf(dctbuf,"DCT:P%02x%02x\r\n",tempCANRxMSG.Data[1],tempCANRxMSG.Data[2]);
//				Usart_SendString(ATKBLE02_USARTx,dctbuf);
//				memset(dctbuf,0,10);
//				printf("DCT4:P%02x%02x\r\n",tempCANRxMSG.Data[3],tempCANRxMSG.Data[4]);
//				sprintf(dctbuf,"DCT:P%02x%02x\r\n",tempCANRxMSG.Data[3],tempCANRxMSG.Data[4]);
//				Usart_SendString(ATKBLE02_USARTx,dctbuf);
//				memset(dctbuf,0,10);
//			}
			vTaskDelay(100);
			//vTaskDelayUntil(&pxPreviousWakeTime,1000);
		}
	}

}

/**********************************************************************
  * @ ������  	��MPU6050_Task
  * @ ����˵��	��ÿ��һ��ʱ�䣬��ȡMPU6050��ֵ�����ʱ�䲻��̫������֪��Ϊʲô
	*							ʹ�����ģ��I2C����ͨ��ʱ��Ҫ���ж�						
  * @ ����		��   
  * @ ����ֵ  	�� ��
  ********************************************************************/
//static void MPU_Task(void* parameter)
//{	 
//  BaseType_t xReturn = pdFALSE;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */
//	TickType_t pxPreviousWakeTime=0;//xTaskGetTickCount();
//	char ch=0;
//	char time=0;
//	float ACC=0;
//	char sendbuf[20];
////	EuleranglesTypeDef S_Eulerangles;
//	pxPreviousWakeTime=xTaskGetTickCount();
//	char ACCCALSW[5] = {0XFF,0XAA,0X01,0X01,0X00};//������ٶ�У׼ģʽ
//	char SAVACALSW[5]= {0XFF,0XAA,0X00,0X00,0X00};//���浱ǰ����
//	USART_ITConfig(MPU_USARTx, USART_IT_RXNE, ENABLE);
//	USART_Cmd(MPU_USARTx, ENABLE);	

//  while (1)
//  {
//		int i=0;
//		while(uxQueueMessagesWaiting(MPU_Queue)!=0){
//			xReturn = xQueueReceive( 		MPU_Queue,    /* ��Ϣ���еľ�� */
//																	&ch,      /* ���͵���Ϣ���� */
//																	5); /* �ȴ�ʱ�� һֱ�� */	//portMAX_DELAY
//			if(xReturn!=pdFALSE){
//				//printf("%c",ch);
//				CopeSerial2Data(ch);
//			}
//				i++;
//		}
//		time++;
//		vTaskDelayUntil(&pxPreviousWakeTime,1000);
//		if(time==20){
//			printf("���ڽ��м��ٶ�У׼\r\n");
//			sendcmd(ACCCALSW);//�ȴ�ģ���ڲ��Զ�У׼�ã�ģ���ڲ����Զ�������Ҫһ����ʱ��
//			vTaskDelay(100);
//			sendcmd(SAVACALSW);//���浱ǰ����
//			vTaskDelay(100);
//			printf("���ٶ�У׼���\r\n");
//			time=0;
//		}else{
//			//���ʱ��
//			//������ٶ�
//			//���ڽ��ܵ��������Ѿ���������Ӧ�Ľṹ��ı������ˣ�����˵�����Э�飬�Լ��ٶ�Ϊ�� stcAcc.a[0]/32768*16����X��ļ��ٶȣ�
//			printf("Acc:%.3f %.3f %.3f\r\n",(float)stcAcc.a[0]/32768*16,(float)stcAcc.a[1]/32768*16,(float)stcAcc.a[2]/32768*16);
//			//ACCCAL.accy=(float)stcAcc.a[1]/32768*16;
//			ACC=pow((pow(((float)stcAcc.a[0]/32768*16),2)+pow(((float)stcAcc.a[1]/32768*16),2)+pow(((float)stcAcc.a[2]/32768*16),2)),0.5)*9.8-9.8;
//			printf("ACC:%.3f\r\n",ACC);
//			sprintf(sendbuf,"ACC:%.3f\r\n",ACC);
//			Usart_SendString(ATKBLE02_USARTx, sendbuf);
////			//������ٶ�
////			printf("Gyro:%.3f %.3f %.3f\r\n",(float)stcGyro.w[0]/32768*2000,(float)stcGyro.w[1]/32768*2000,(float)stcGyro.w[2]/32768*2000);
////			//����Ƕ�
////			printf("Angle:%.3f %.3f %.3f\r\n",(float)stcAngle.Angle[0]/32768*180,(float)stcAngle.Angle[1]/32768*180,(float)stcAngle.Angle[2]/32768*180);	
//		}
//		vTaskDelay(100);
//  }
//}
/**********************************************************************
  * @ ������  	��OBDTEST_Task
  * @ ����˵��	��					
  * @ ����		��   
  * @ ����ֵ  	�� ��
  ********************************************************************/
static void OBDTEST_Task(void* parameter)
{	 
  BaseType_t xReturn = pdPASS;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */
	CanRxMsg tempCANRxMSG;
	char VIN[18];
	memset((char *)&tempCANRxMSG,0x00,sizeof(CanRxMsg));
	memset(VIN,0x00,18);
  while (1)
  {

			CANinit(500);
			vTaskDelay(100);
			uint8_t senddata[8]={0x02,0x09,0x02,0x00,0x00,0x00,0x00,0x00};
			CANTransmit_frame(0x7DF,CAN_Id_Standard,senddata,8);
			//OBDTransmit_askframe(0x7DF,CAN_Id_Standard,0x09,0x02);
			xReturn = xQueueReceive( 	CANmsg_Queue,    /* ��Ϣ���еľ�� */
																&tempCANRxMSG,      /* ���͵���Ϣ���� */
																500); /* portMAX_DELAY�ȴ�ʱ�� һֱ�� */
			if(xReturn==pdFALSE){
					printf("CAN_500K_11B is not support\r\n");
					CANinit(500);
					vTaskDelay(100);
					uint8_t senddata[8]={0x02,0x09,0x02,0x00,0x00,0x00,0x00,0x00};
					CANTransmit_frame(0x18DB33F1,CAN_Id_Extended,senddata,8);
					xReturn = xQueueReceive( 	CANmsg_Queue,    /* ��Ϣ���еľ�� */
																		&tempCANRxMSG,      /* ���͵���Ϣ���� */
																		500); /* portMAX_DELAY�ȴ�ʱ�� һֱ�� */
					if(xReturn==pdFALSE){
									printf("CAN_500K_29B is not support\r\n");
									CANinit(250);
									vTaskDelay(100);
									uint8_t senddata[8]={0x02,0x09,0x02,0x00,0x00,0x00,0x00,0x00};
									CANTransmit_frame(0x7DF,CAN_Id_Standard,senddata,8);
									xReturn = xQueueReceive( 	CANmsg_Queue,    /* ��Ϣ���еľ�� */
																						&tempCANRxMSG,      /* ���͵���Ϣ���� */
																						500); /* portMAX_DELAY�ȴ�ʱ�� һֱ�� */							
									if(xReturn==pdFALSE){
												printf("CAN_250K_11B is not support\r\n");
												CANinit(250);
												vTaskDelay(100);
												uint8_t senddata[8]={0x02,0x09,0x02,0x00,0x00,0x00,0x00,0x00};
												CANTransmit_frame(0x18DB33F1,CAN_Id_Extended,senddata,8);
												xReturn = xQueueReceive( 	CANmsg_Queue,    /* ��Ϣ���еľ�� */
																									&tempCANRxMSG,      /* ���͵���Ϣ���� */
																									500); /* portMAX_DELAY�ȴ�ʱ�� һֱ�� */										
												if(xReturn==pdFALSE){
													printf("CAN_250K_29B is not support\r\n");
													while(1){
														printf("��ǰOBD��֧��CANЭ�飬����δ����");
														vTaskDelay(3000);
													}

												}else{
												//CAN_250K_29B is support
															memset(VIN,0x00,18);
															for(int i=0;i<3;i++){
																//printf("%#x ",tempCANRxMSG.Data[5+i]);
																VIN[i]=tempCANRxMSG.Data[5+i];
															}											
															uint8_t senddata[8]={0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
															CANTransmit_frame(0x7DF,CAN_Id_Standard,senddata,8);
															xReturn = xQueueReceive( 	CANmsg_Queue,    /* ��Ϣ���еľ�� */
																												&tempCANRxMSG,      /* ���͵���Ϣ���� */
																												200); /* portMAX_DELAY�ȴ�ʱ�� һֱ�� */
															for(int i=0;i<7;i++){
																//printf("%#x ",tempCANRxMSG.Data[1+i]);
																VIN[3+i]=tempCANRxMSG.Data[1+i];
															}		
															xReturn = xQueueReceive( 	CANmsg_Queue,    /* ��Ϣ���еľ�� */
																												&tempCANRxMSG,      /* ���͵���Ϣ���� */
																												200); /* portMAX_DELAY�ȴ�ʱ�� һֱ�� */
															for(int i=0;i<7;i++){
																//printf("%#x ",tempCANRxMSG.Data[1+i]);
																VIN[10+i]=tempCANRxMSG.Data[1+i];
															}
															//VIN[17]=0;
															//printf("\r\n");
															for(int i=0;i<18;i++){
																//printf("%#x ",VIN[i]);
															}
															printf("VIN:%s\r\n",VIN);
															printf("Э�飺CAN_250K_29B\r\n");
															CAN_TYPE=CAN_250K_29B;
												}
																				
									}else{
										//CAN_250K_11B is support
										memset(VIN,0x00,18);
												for(int i=0;i<3;i++){
													//printf("%#x ",tempCANRxMSG.Data[5+i]);
													VIN[i]=tempCANRxMSG.Data[5+i];
												}											
												uint8_t senddata[8]={0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
												CANTransmit_frame(0x7DF,CAN_Id_Standard,senddata,8);
												xReturn = xQueueReceive( 	CANmsg_Queue,    /* ��Ϣ���еľ�� */
																									&tempCANRxMSG,      /* ���͵���Ϣ���� */
																									200); /* portMAX_DELAY�ȴ�ʱ�� һֱ�� */    
												for(int i=0;i<7;i++){
													//printf("%#x ",tempCANRxMSG.Data[1+i]);
													VIN[3+i]=tempCANRxMSG.Data[1+i];
												}		
												xReturn = xQueueReceive( 	CANmsg_Queue,    /* ��Ϣ���еľ�� */
																									&tempCANRxMSG,      /* ���͵���Ϣ���� */
																									200); /* portMAX_DELAY�ȴ�ʱ�� һֱ�� */
												for(int i=0;i<7;i++){
													//printf("%#x ",tempCANRxMSG.Data[1+i]);
													VIN[10+i]=tempCANRxMSG.Data[1+i];
												}
												//VIN[17]=0;

												printf("VIN:%s\r\n",VIN);
												printf("���ӳɹ�����Э�飺CAN_250K_11B\r\n");
												CAN_TYPE=CAN_250K_11B;
									
									}
					}else{
					//CAN_500K_29B is support
							memset(VIN,0x00,18);
							for(int i=0;i<3;i++){
								//printf("%#x ",tempCANRxMSG.Data[5+i]);
								VIN[i]=tempCANRxMSG.Data[5+i];
							}											
							uint8_t senddata[8]={0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
							CANTransmit_frame(0x7DF,CAN_Id_Extended,senddata,8);
							xReturn = xQueueReceive( 	CANmsg_Queue,    /* ��Ϣ���еľ�� */
																				&tempCANRxMSG,      /* ���͵���Ϣ���� */
																				200); /* portMAX_DELAY�ȴ�ʱ�� һֱ�� */
							for(int i=0;i<7;i++){
								//printf("%#x ",tempCANRxMSG.Data[1+i]);
								VIN[3+i]=tempCANRxMSG.Data[1+i];
							}		
							xReturn = xQueueReceive( 	CANmsg_Queue,    /* ��Ϣ���еľ�� */
																				&tempCANRxMSG,      /* ���͵���Ϣ���� */
																				200); /* portMAX_DELAY�ȴ�ʱ�� һֱ�� */
							for(int i=0;i<7;i++){
								//printf("%#x ",tempCANRxMSG.Data[1+i]);
								VIN[10+i]=tempCANRxMSG.Data[1+i];
							}
							//VIN[17]=0;
							//printf("\r\n");
							//for(int i=0;i<18;i++){
								//printf("%#x ",VIN[i]);
							//}
							printf("VIN:%s\r\n",VIN);
							printf("Э�飺CAN_500K_29B\r\n");
							CAN_TYPE=CAN_500K_29B;
				
				}
			}else{
				//CAN_500K_11B is support
							memset(VIN,0x00,18);
							for(int i=0;i<3;i++){
								//printf("%#x ",tempCANRxMSG.Data[5+i]);
								VIN[i]=tempCANRxMSG.Data[5+i];
							}											
							uint8_t senddata[8]={0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
							CANTransmit_frame(0x7DF,CAN_Id_Standard,senddata,8);
							xReturn = xQueueReceive( 	CANmsg_Queue,    /* ��Ϣ���еľ�� */
																				&tempCANRxMSG,      /* ���͵���Ϣ���� */
																				200); /* portMAX_DELAY�ȴ�ʱ�� һֱ�� */
							for(int i=0;i<7;i++){
								//printf("%#x ",tempCANRxMSG.Data[1+i]);
								VIN[3+i]=tempCANRxMSG.Data[1+i];
							}		
							xReturn = xQueueReceive( 	CANmsg_Queue,    /* ��Ϣ���еľ�� */
																				&tempCANRxMSG,      /* ���͵���Ϣ���� */
																				200); /* portMAX_DELAY�ȴ�ʱ�� һֱ�� */
							for(int i=0;i<7;i++){
								//printf("%#x ",tempCANRxMSG.Data[1+i]);
								VIN[10+i]=tempCANRxMSG.Data[1+i];
							}
							//VIN[17]=0;
							//printf("\r\n");
							//for(int i=0;i<18;i++){
								//printf("%#x ",VIN[i]);
							//}
							printf("VIN:%s\r\n",VIN);
							printf("Э�飺CAN_500K_11B\r\n");
							CAN_TYPE=CAN_500K_11B;
			
			}
			while(1){
					//printf("��Գɹ�\r\n");
					vTaskDelay(1000);
			}
		}
}


static void ATKBLE02_Task(void* parameter)
{
	BaseType_t xReturn = pdFALSE;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */
	TickType_t pxPreviousWakeTime=0;//xTaskGetTickCount();
	char ch =0;
	while(1)
	{
		while(uxQueueMessagesWaiting(ATKBLE02_Queue)!=0){
			xReturn = xQueueReceive( 		ATKBLE02_Queue,    /* ��Ϣ���еľ�� */
																	&ch,      /* ���͵���Ϣ���� */
													5); /* �ȴ�ʱ�� һֱ�� */	//portMAX_DELAY
			
			if(xReturn!=pdFALSE){
				//printf("ch=%c\r\n",ch);
				Atkble02Rec(ch);
				}
			}

		vTaskDelay(300);
		
	}
}
/***********************************************************************
  * @ ������  �� BSP_Init
  * @ ����˵���� �弶�����ʼ�������а����ϵĳ�ʼ�����ɷ��������������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  *********************************************************************/
extern uint32_t time;
static void BSP_Init(void)
{
	/*
	 * STM32�ж����ȼ�����Ϊ4����4bit��������ʾ��ռ���ȼ�����ΧΪ��0~15
	 * ���ȼ�����ֻ��Ҫ����һ�μ��ɣ��Ժ������������������Ҫ�õ��жϣ�
	 * ��ͳһ��������ȼ����飬ǧ��Ҫ�ٷ��飬�мɡ�
	 */
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
	/* LED ��ʼ�� */
	//LED_GPIO_Config();
	BASIC_TIM_Init();
	/* ���ڳ�ʼ��	*/
	ALL_UART_Config();

	BEEP_Init();

	
	/*����ģ���ʼ��*/
	//ATKBLE02_init();
	//ltemodule_USART_Config();
	/*CAN���߳�ʼ��*/
	//CANinit(500);
	
	//ltemodule_MQTT_init();

	//MPU6050_TEST();
	//MPU6050_ENABLE();//ʹ��MP[U6050ģ�飬��ʼ��IIC���ߣ�����ģ�鹤��ģʽ��ʹ��dmp
	


}

//void MPU6050_TEST(void){
//	
//	float pitch,roll,yaw; 		//ŷ����
//	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
//	short gyrox,gyroy,gyroz;	//������ԭʼ����
//	short temp;					//�¶�	
//	
//	while(MPU_Init())
// 	{
//		printf("MPU_Init Error\r\n");
// 		delay_ms(1000);
//	} 				//��ʼ��MPU6050
//	while(mpu_dmp_init())
// 	{
//		printf("MPU6050 Error\r\n");
// 		delay_ms(1000);
//	}  

// 	while(1)
//	{
//	
//		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
//		{ 
//			temp=MPU_Get_Temperature();	//�õ��¶�ֵ
//			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
//			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
//			//if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//���Զ���֡���ͼ��ٶȺ�������ԭʼ����
//			//if(report)usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
//			
//			printf("temp:%d\r\n",temp);
//			printf("pitch*10:%f\r\n",pitch);
//			printf("roll*10:%f\r\n",roll);
//			printf("yaw*10:%f\r\n",yaw);
//			delay_ms(1000);
//			delay_ms(10);
//		}
//	}
//}






/********************************END OF FILE****************************/

