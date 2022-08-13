/*
*************************************************************************
*                             包含的头文件
*************************************************************************
*/ 
/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* 开发板硬件bsp头文件 */
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
/**************************** 任务句柄 ********************************/
/* 
 * 任务句柄是一个指针，用于指向一个任务，当任务创建好之后，它就具有了一个任务句柄
 * 以后我们要想操作这个任务都需要通过这个任务句柄，如果是自身的任务操作自己，那么
 * 这个句柄可以为NULL。
 */
static TaskHandle_t AppTaskCreate_Handle = NULL;/* 创建任务句柄 */
static TaskHandle_t OBDASKDATA_Task_Handle = NULL;
static TaskHandle_t OBDGETDATA_Task_Handle = NULL;
static TaskHandle_t ATKBLE02_Task_Handle = NULL;
static TaskHandle_t GPS_Task_Handle = NULL;
static TaskHandle_t MPU_Task_Handle=NULL;
static TaskHandle_t OBDTEST_Task_Handle = NULL;

/********************************** 内核对象句柄 *********************************/
/*
 * 信号量，消息队列，事件标志组，软件定时器这些都属于内核的对象，要想使用这些内核
 * 对象，必须先创建，创建成功之后会返回一个相应的句柄。实际上就是一个指针，后续我
 * 们就可以通过这个句柄操作这些内核对象。
 *
 * 内核对象说白了就是一种全局的数据结构，通过这些数据结构我们可以实现任务间的通信，
 * 任务间的事件同步等各种功能。至于这些功能的实现我们是通过调用这些内核对象的函数
 * 来完成的
 * 
 */

QueueHandle_t CANmsg_Queue =NULL;
QueueHandle_t ATKBLE02_Queue =NULL;
QueueHandle_t GPS_Queue =NULL;
//QueueHandle_t WIFI_Queue =NULL;
QueueHandle_t MPU_Queue =NULL;
SemaphoreHandle_t  BLESEND_Handle=NULL;


/******************************* 全局变量声明 ************************************/
/*
 * 当我们在写应用程序的时候，可能需要用到一些全局变量。
 */
Info_1secTypeDef  	Info_1sec_gal;

char OBDhasasked=0;
struct {
	double Longitude;
	double Latitude;
	double accy;
	uint8_t VSS;
}ACCCAL;
/******************************* 宏定义 ************************************/
#define  MPU6050_QUEUE_LEN    10   /* 队列的长度，最大可包含多少个消息 */
#define  MPU6050_QUEUE_SIZE   (sizeof(EuleranglesTypeDef))   /* 队列中每个消息大小（字节） */
#define  CANMSG_QUEUE_LEN    128   /* 队列的长度，最大可包含多少个消息 */
#define  CANMSG_QUEUE_SIZE   (sizeof(CanRxMsg))   /* 队列中每个消息大小（字节） */
#define  ATKBLE02_QUEUE_LEN    100   /* 队列的长度，最大可包含多少个消息 */
#define  ATKBLE02_QUEUE_SIZE   1   /* 队列中每个消息大小（字节） */
#define  GPS_QUEUE_LEN    100   /* 队列的长度，最大可包含多少个消息 */
#define  GPS_QUEUE_SIZE   1   /* 队列中每个消息大小（字节） */


#define  MPU_QUEUE_LEN    1024   /* 队列的长度，最大可包含多少个消息 */
#define  MPU_QUEUE_SIZE   1   /* 队列中每个消息大小（字节） */	
#define  WIFISEND_QUEUE_LEN    2048   /* 队列的长度，最大可包含多少个消息 */
#define  WIFISEND_QUEUE_SIZE   1   /* 队列中每个消息大小（字节） */	
/*
*************************************************************************
*                             函数声明
*************************************************************************
*/
static void AppTaskCreate(void);/* 用于创建任务 */
static void	OBD_askdata_Task(void* pvParameters);/* SendInfo_1s_Task */
static void	OBD_getdata_Task(void* pvParameters);/* SendInfo_1s_Task */

static void	GPS_Task(void* pvParameters);/* SendInfo_1s_Task */

static void	MPU_Task(void* pvParameters);/* SendInfo_1s_Task */
static void OBDTEST_Task(void* pvParameters);
static void	ATKBLE02_Task(void* pvParameters);/* SendInfo_1s_Task */


static void BSP_Init(void);/* 用于初始化板载相关资源 */
///*FreeRTOS-CLI初始化函数*/
//extern void vUARTCommandConsoleStart( uint16_t usStackSize, UBaseType_t uxPriority );//FreeRTOS-CL任务创建
//extern	void vRegisterSampleCLICommands( void );//FreeRTOS-CLI指令注册

/*****************************************************************
  * @brief  主函数
  * @param  无
  * @retval 无
  * @note   第一步：开发板硬件初始化 
            第二步：创建APP应用任务
            第三步：启动FreeRTOS，开始多任务调度
  ****************************************************************/
int main(void)
{	
  BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
  

	/* 开发板硬件初始化 */
  BSP_Init();

   /* 创建AppTaskCreate任务 */
  xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate,  /* 任务入口函数 */
                        (const char*    )"AppTaskCreate",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )1, /* 任务的优先级 */
                        (TaskHandle_t*  )&AppTaskCreate_Handle);/* 任务控制块指针 */ 


	
  /* 启动任务调度 */           
  if(pdPASS == xReturn)
    vTaskStartScheduler();   /* 启动任务，开启调度 */
  else
    return -1;  
  

  while(1);   /* 正常不会执行到这里 */    
}




/***********************************************************************
  * @ 函数名  ： AppTaskCreate
  * @ 功能说明： 为了方便管理，所有的任务创建函数都放在这个函数里面
  * @ 参数    ： 无  
  * @ 返回值  ： 无
  **********************************************************************/
static void AppTaskCreate(void)
{
  BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */

  taskENTER_CRITICAL();           //进入临界区
  
//		BLESEND_Handle=xSemaphoreCreateMutex();
//		if(NULL != BLESEND_Handle)
//		printf("创建BLESEND_Handle互斥信号量成功!\r\n");
//		
//		
//		xReturn=xSemaphoreGive(BLESEND_Handle);//释放互斥信号量
		

		
		/* 创建CANmsg_Queue */
		CANmsg_Queue = xQueueCreate((UBaseType_t ) CANMSG_QUEUE_LEN,/* 消息队列的长度 */
															(UBaseType_t ) CANMSG_QUEUE_SIZE);/* 消息的大小 */
		if(NULL != CANmsg_Queue)
			//printf("创建CANmsg_Queue队列成功!\r\n");
		
		/* 创建ATKBLE02_Queue */
		ATKBLE02_Queue = xQueueCreate((UBaseType_t ) ATKBLE02_QUEUE_LEN,/* 消息队列的长度 */
															(UBaseType_t ) ATKBLE02_QUEUE_SIZE);/* 消息的大小 */
		if(NULL != ATKBLE02_Queue)
		//printf("创建ATKBLE02_Queue队列成功!\r\n");
		/* 创建GPS_Queue */
		GPS_Queue = xQueueCreate((UBaseType_t ) GPS_QUEUE_LEN,/* 消息队列的长度 */
															(UBaseType_t ) GPS_QUEUE_SIZE);/* 消息的大小 */
		if(NULL != GPS_Queue)
		//printf("创建GPS_Queue队列成功!\r\n");
		/* 创建MPU_Queue */

		MPU_Queue = xQueueCreate((UBaseType_t ) MPU_QUEUE_LEN,/* 消息队列的长度 */
															(UBaseType_t ) MPU_QUEUE_SIZE);/* 消息的大小 */
		if(NULL != MPU_Queue)
		//printf("创建MPU_Queue队列成功!\r\n");	
		


		/* 创建OBD_askdata_Task */
	    xReturn = xTaskCreate((TaskFunction_t )OBD_askdata_Task, /* 任务入口函数 */
                        (const char*    )"OBD_getdata_Task",/* 任务名字 */
                        (uint16_t       )512,   /* 任务栈大小 */
                        (void*          )NULL,	/* 任务入口函数参数 */
                        (UBaseType_t    )8,	    /* 任务的优先级 */
                        (TaskHandle_t*  )&OBDASKDATA_Task_Handle);/* 任务控制块指针 */
		if(pdPASS == xReturn)
		printf("创建OBD_askdata_Task成功!\r\n");	
		
		/* 创建OBD_getdata_Task */
	    xReturn = xTaskCreate((TaskFunction_t )OBD_getdata_Task, /* 任务入口函数 */
                        (const char*    )"OBD_getdata_Task",/* 任务名字 */
                        (uint16_t       )512,   /* 任务栈大小 */
                        (void*          )NULL,	/* 任务入口函数参数 */
                        (UBaseType_t    )8,	    /* 任务的优先级 */
                        (TaskHandle_t*  )&OBDGETDATA_Task_Handle);/* 任务控制块指针 */
		if(pdPASS == xReturn)
		printf("创建OBD_getdata_Task成功!\r\n");	
		
		/* 创建GPS_Task */
//		xReturn = xTaskCreate((TaskFunction_t )GPS_Task, /* 任务入口函数 */
//						(const char*    )"GPS_Task",/* 任务名字 */
//						(uint16_t       )512,   /* 任务栈大小 */
//						(void*          )NULL,	/* 任务入口函数参数 */
//						(UBaseType_t    )8,	    /* 任务的优先级 */
//						(TaskHandle_t*  )&GPS_Task_Handle);/* 任务控制块指针 */
//		if(pdPASS == xReturn)
//		printf("创建GPS_Task成功!\r\n");
		/* 创建MPU_Task */
//		xReturn = xTaskCreate((TaskFunction_t )MPU_Task, /* 任务入口函数 */
//													(const char*    )"MPU_Task",/* 任务名字 */
//													(uint16_t       )512,   /* 任务栈大小 */
//													(void*          )NULL,	/* 任务入口函数参数 */
//													(UBaseType_t    )8,	    /* 任务的优先级 */
//													(TaskHandle_t*  )&MPU_Task_Handle);/* 任务控制块指针 */
//		if(pdPASS == xReturn)
//		printf("创建MPU_Task成功!\r\n");
		
		
		/* 创建ATKBLE02_Task */
		xReturn = xTaskCreate((TaskFunction_t )ATKBLE02_Task, /* 任务入口函数 */
													(const char*    )"ATKBLE02_Task",/* 任务名字 */
													(uint16_t       )512,   /* 任务栈大小 */
													(void*          )NULL,	/* 任务入口函数参数 */
													(UBaseType_t    )8,	    /* 任务的优先级 */
													(TaskHandle_t*  )&ATKBLE02_Task_Handle);/* 任务控制块指针 */
		if(pdPASS == xReturn)
		printf("创建ATKBLE02_Task成功!\r\n");
		
	
		
		/* 创建OBDTEST_Task */
		xReturn = xTaskCreate((TaskFunction_t )OBDTEST_Task, /* 任务入口函数 */
													(const char*    )"OBDTEST_Task",/* 任务名字 */
													(uint16_t       )512,   /* 任务栈大小 */
													(void*          )NULL,	/* 任务入口函数参数 */
													(UBaseType_t    )2,	    /* 任务的优先级 */
													(TaskHandle_t*  )&OBDTEST_Task_Handle);/* 任务控制块指针 */
		if(pdPASS == xReturn)
		printf("创建OBDTEST_Task_Task成功!\r\n");
		
	
		
//	vRegisterSampleCLICommands(  );
//	vUARTCommandConsoleStart( 512, 3 );
	
  vTaskDelete(AppTaskCreate_Handle); //删除AppTaskCreate任务 
  taskEXIT_CRITICAL();            //退出临界区

}

static void OBD_askdata_Task(void* parameter){
    BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
	
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
		
		for(i=0;i<numofask+1;i++){//发送i个请求帧
			if(OBDdataHz_new[i]==0)continue;
				OBDTransmit_askframe(newOBDdata[i].StdId,IDE,newOBDdata[i].SID,newOBDdata[i].PID);
				vTaskDelay(20);
				if(OBDhasasked==0)OBDhasasked=1;
		}
		//模拟GPS有效数据，主使用时要关掉GPS串口。
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
  * @ 函数名  	：GPS_Task
* @ 功能说明		:				
  * @ 参数			：   
  * @ 返回值  	： 无
  ********************************************************************/
//static void GPS_Task(void* parameter)
//{	 
//	BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
//	char rechar[128];
//	
//	memset(rechar,0x00,128);
//	int point1=0;	
//	USART_ITConfig(GPS_USARTx, USART_IT_RXNE, ENABLE);
//	USART_Cmd(GPS_USARTx, ENABLE);	

//    while (1)
//	{	
//		xReturn = xQueueReceive( 		GPS_Queue,    /* 消息队列的句柄 */
//																(rechar+point1),      /* 发送的消息内容 */
//																portMAX_DELAY); /* 等待时间 一直等 */	
//		if(*(rechar+point1) == '$'){
//			point1 = 0;	
//			memset(rechar,0x00,128);
//			*(rechar+point1)='$';			
//		}
//		if(rechar[0] == '$' && rechar[4] == 'M' && rechar[5] == 'C')			//确定是否收到"GPRMC/GNRMC"这一帧数据
//		{
//			
//			if(*(rechar+point1) == '\n')									   
//			{
//				//char timestrbuff[128];
//				//uinxtostr(get_timestamp_s(),timestrbuff);
//				//printf("now time:%s\r\n",timestrbuff);
//				memset(Save_Data.GPS_Buffer, 0, GPS_Buffer_Length);      //清空
//				memcpy(Save_Data.GPS_Buffer, rechar, point1); 	//保存数据
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
//					printf("GPS数据有效\r\n");
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
  BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
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
			xReturn = xQueueReceive( 	CANmsg_Queue,    /* 消息队列的句柄 */
										&tempCANRxMSG,      /* 发送的消息内容 */
										portMAX_DELAY); /* portMAX_DELAY等待时间 一直等 */	
			char dctbuf[20];
			if((tempCANRxMSG.Data[0]==0x10)&&(tempCANRxMSG.Data[2]==0x43)){//故障码首帧
				
							
				printf("DCT1:P%02x%02x\r\n",tempCANRxMSG.Data[4],tempCANRxMSG.Data[5]);
				sprintf(dctbuf,"DCT:P%02x%02x\r\n",tempCANRxMSG.Data[4],tempCANRxMSG.Data[5]);
				Usart_SendString(ATKBLE02_USARTx,dctbuf);
				memset(dctbuf,0,20);
				printf("DCT2:P%02x%02x\r\n",tempCANRxMSG.Data[6],tempCANRxMSG.Data[7]);
				sprintf(dctbuf,"DCT:P%02x%02x\r\n",tempCANRxMSG.Data[6],tempCANRxMSG.Data[7]);
				Usart_SendString(ATKBLE02_USARTx,dctbuf);
				memset(dctbuf,0,20);
				
				uint8_t senddata[8]={0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00};			
				CANTransmit_frame(0x7DF,CAN_Id_Standard,senddata,8);//发送流控帧使连续帧继续发送
				while(uxQueueMessagesWaiting(CANmsg_Queue)==0){	
				}
				xReturn = xQueueReceive( 	CANmsg_Queue,    /* 消息队列的句柄 */
											&tempCANRxMSG,      /* 发送的消息内容 */
											portMAX_DELAY); /* portMAX_DELAY等待时间 一直等 */
				if(tempCANRxMSG.Data[0]==0x21)//连续帧
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
//			if(((tempCANRxMSG.Data[0]&0xf0)==20)&&((tempCANRxMSG.Data[0]&0x0f)>=1))//连续帧
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
  * @ 函数名  	：MPU6050_Task
  * @ 功能说明	：每隔一段时间，读取MPU6050的值，间隔时间不能太长，不知道为什么
	*							使用软件模拟I2C总线通信时，要关中断						
  * @ 参数		：   
  * @ 返回值  	： 无
  ********************************************************************/
//static void MPU_Task(void* parameter)
//{	 
//  BaseType_t xReturn = pdFALSE;/* 定义一个创建信息返回值，默认为pdPASS */
//	TickType_t pxPreviousWakeTime=0;//xTaskGetTickCount();
//	char ch=0;
//	char time=0;
//	float ACC=0;
//	char sendbuf[20];
////	EuleranglesTypeDef S_Eulerangles;
//	pxPreviousWakeTime=xTaskGetTickCount();
//	char ACCCALSW[5] = {0XFF,0XAA,0X01,0X01,0X00};//进入加速度校准模式
//	char SAVACALSW[5]= {0XFF,0XAA,0X00,0X00,0X00};//保存当前配置
//	USART_ITConfig(MPU_USARTx, USART_IT_RXNE, ENABLE);
//	USART_Cmd(MPU_USARTx, ENABLE);	

//  while (1)
//  {
//		int i=0;
//		while(uxQueueMessagesWaiting(MPU_Queue)!=0){
//			xReturn = xQueueReceive( 		MPU_Queue,    /* 消息队列的句柄 */
//																	&ch,      /* 发送的消息内容 */
//																	5); /* 等待时间 一直等 */	//portMAX_DELAY
//			if(xReturn!=pdFALSE){
//				//printf("%c",ch);
//				CopeSerial2Data(ch);
//			}
//				i++;
//		}
//		time++;
//		vTaskDelayUntil(&pxPreviousWakeTime,1000);
//		if(time==20){
//			printf("正在进行加速度校准\r\n");
//			sendcmd(ACCCALSW);//等待模块内部自动校准好，模块内部会自动计算需要一定的时间
//			vTaskDelay(100);
//			sendcmd(SAVACALSW);//保存当前配置
//			vTaskDelay(100);
//			printf("加速度校准完成\r\n");
//			time=0;
//		}else{
//			//输出时间
//			//输出加速度
//			//串口接受到的数据已经拷贝到对应的结构体的变量中了，根据说明书的协议，以加速度为例 stcAcc.a[0]/32768*16就是X轴的加速度，
//			printf("Acc:%.3f %.3f %.3f\r\n",(float)stcAcc.a[0]/32768*16,(float)stcAcc.a[1]/32768*16,(float)stcAcc.a[2]/32768*16);
//			//ACCCAL.accy=(float)stcAcc.a[1]/32768*16;
//			ACC=pow((pow(((float)stcAcc.a[0]/32768*16),2)+pow(((float)stcAcc.a[1]/32768*16),2)+pow(((float)stcAcc.a[2]/32768*16),2)),0.5)*9.8-9.8;
//			printf("ACC:%.3f\r\n",ACC);
//			sprintf(sendbuf,"ACC:%.3f\r\n",ACC);
//			Usart_SendString(ATKBLE02_USARTx, sendbuf);
////			//输出角速度
////			printf("Gyro:%.3f %.3f %.3f\r\n",(float)stcGyro.w[0]/32768*2000,(float)stcGyro.w[1]/32768*2000,(float)stcGyro.w[2]/32768*2000);
////			//输出角度
////			printf("Angle:%.3f %.3f %.3f\r\n",(float)stcAngle.Angle[0]/32768*180,(float)stcAngle.Angle[1]/32768*180,(float)stcAngle.Angle[2]/32768*180);	
//		}
//		vTaskDelay(100);
//  }
//}
/**********************************************************************
  * @ 函数名  	：OBDTEST_Task
  * @ 功能说明	：					
  * @ 参数		：   
  * @ 返回值  	： 无
  ********************************************************************/
static void OBDTEST_Task(void* parameter)
{	 
  BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
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
			xReturn = xQueueReceive( 	CANmsg_Queue,    /* 消息队列的句柄 */
																&tempCANRxMSG,      /* 发送的消息内容 */
																500); /* portMAX_DELAY等待时间 一直等 */
			if(xReturn==pdFALSE){
					printf("CAN_500K_11B is not support\r\n");
					CANinit(500);
					vTaskDelay(100);
					uint8_t senddata[8]={0x02,0x09,0x02,0x00,0x00,0x00,0x00,0x00};
					CANTransmit_frame(0x18DB33F1,CAN_Id_Extended,senddata,8);
					xReturn = xQueueReceive( 	CANmsg_Queue,    /* 消息队列的句柄 */
																		&tempCANRxMSG,      /* 发送的消息内容 */
																		500); /* portMAX_DELAY等待时间 一直等 */
					if(xReturn==pdFALSE){
									printf("CAN_500K_29B is not support\r\n");
									CANinit(250);
									vTaskDelay(100);
									uint8_t senddata[8]={0x02,0x09,0x02,0x00,0x00,0x00,0x00,0x00};
									CANTransmit_frame(0x7DF,CAN_Id_Standard,senddata,8);
									xReturn = xQueueReceive( 	CANmsg_Queue,    /* 消息队列的句柄 */
																						&tempCANRxMSG,      /* 发送的消息内容 */
																						500); /* portMAX_DELAY等待时间 一直等 */							
									if(xReturn==pdFALSE){
												printf("CAN_250K_11B is not support\r\n");
												CANinit(250);
												vTaskDelay(100);
												uint8_t senddata[8]={0x02,0x09,0x02,0x00,0x00,0x00,0x00,0x00};
												CANTransmit_frame(0x18DB33F1,CAN_Id_Extended,senddata,8);
												xReturn = xQueueReceive( 	CANmsg_Queue,    /* 消息队列的句柄 */
																									&tempCANRxMSG,      /* 发送的消息内容 */
																									500); /* portMAX_DELAY等待时间 一直等 */										
												if(xReturn==pdFALSE){
													printf("CAN_250K_29B is not support\r\n");
													while(1){
														printf("当前OBD不支持CAN协议，或者未插入");
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
															xReturn = xQueueReceive( 	CANmsg_Queue,    /* 消息队列的句柄 */
																												&tempCANRxMSG,      /* 发送的消息内容 */
																												200); /* portMAX_DELAY等待时间 一直等 */
															for(int i=0;i<7;i++){
																//printf("%#x ",tempCANRxMSG.Data[1+i]);
																VIN[3+i]=tempCANRxMSG.Data[1+i];
															}		
															xReturn = xQueueReceive( 	CANmsg_Queue,    /* 消息队列的句柄 */
																												&tempCANRxMSG,      /* 发送的消息内容 */
																												200); /* portMAX_DELAY等待时间 一直等 */
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
															printf("协议：CAN_250K_29B\r\n");
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
												xReturn = xQueueReceive( 	CANmsg_Queue,    /* 消息队列的句柄 */
																									&tempCANRxMSG,      /* 发送的消息内容 */
																									200); /* portMAX_DELAY等待时间 一直等 */    
												for(int i=0;i<7;i++){
													//printf("%#x ",tempCANRxMSG.Data[1+i]);
													VIN[3+i]=tempCANRxMSG.Data[1+i];
												}		
												xReturn = xQueueReceive( 	CANmsg_Queue,    /* 消息队列的句柄 */
																									&tempCANRxMSG,      /* 发送的消息内容 */
																									200); /* portMAX_DELAY等待时间 一直等 */
												for(int i=0;i<7;i++){
													//printf("%#x ",tempCANRxMSG.Data[1+i]);
													VIN[10+i]=tempCANRxMSG.Data[1+i];
												}
												//VIN[17]=0;

												printf("VIN:%s\r\n",VIN);
												printf("连接成功！！协议：CAN_250K_11B\r\n");
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
							xReturn = xQueueReceive( 	CANmsg_Queue,    /* 消息队列的句柄 */
																				&tempCANRxMSG,      /* 发送的消息内容 */
																				200); /* portMAX_DELAY等待时间 一直等 */
							for(int i=0;i<7;i++){
								//printf("%#x ",tempCANRxMSG.Data[1+i]);
								VIN[3+i]=tempCANRxMSG.Data[1+i];
							}		
							xReturn = xQueueReceive( 	CANmsg_Queue,    /* 消息队列的句柄 */
																				&tempCANRxMSG,      /* 发送的消息内容 */
																				200); /* portMAX_DELAY等待时间 一直等 */
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
							printf("协议：CAN_500K_29B\r\n");
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
							xReturn = xQueueReceive( 	CANmsg_Queue,    /* 消息队列的句柄 */
																				&tempCANRxMSG,      /* 发送的消息内容 */
																				200); /* portMAX_DELAY等待时间 一直等 */
							for(int i=0;i<7;i++){
								//printf("%#x ",tempCANRxMSG.Data[1+i]);
								VIN[3+i]=tempCANRxMSG.Data[1+i];
							}		
							xReturn = xQueueReceive( 	CANmsg_Queue,    /* 消息队列的句柄 */
																				&tempCANRxMSG,      /* 发送的消息内容 */
																				200); /* portMAX_DELAY等待时间 一直等 */
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
							printf("协议：CAN_500K_11B\r\n");
							CAN_TYPE=CAN_500K_11B;
			
			}
			while(1){
					//printf("配对成功\r\n");
					vTaskDelay(1000);
			}
		}
}


static void ATKBLE02_Task(void* parameter)
{
	BaseType_t xReturn = pdFALSE;/* 定义一个创建信息返回值，默认为pdPASS */
	TickType_t pxPreviousWakeTime=0;//xTaskGetTickCount();
	char ch =0;
	while(1)
	{
		while(uxQueueMessagesWaiting(ATKBLE02_Queue)!=0){
			xReturn = xQueueReceive( 		ATKBLE02_Queue,    /* 消息队列的句柄 */
																	&ch,      /* 发送的消息内容 */
													5); /* 等待时间 一直等 */	//portMAX_DELAY
			
			if(xReturn!=pdFALSE){
				//printf("ch=%c\r\n",ch);
				Atkble02Rec(ch);
				}
			}

		vTaskDelay(300);
		
	}
}
/***********************************************************************
  * @ 函数名  ： BSP_Init
  * @ 功能说明： 板级外设初始化，所有板子上的初始化均可放在这个函数里面
  * @ 参数    ：   
  * @ 返回值  ： 无
  *********************************************************************/
extern uint32_t time;
static void BSP_Init(void)
{
	/*
	 * STM32中断优先级分组为4，即4bit都用来表示抢占优先级，范围为：0~15
	 * 优先级分组只需要分组一次即可，以后如果有其他的任务需要用到中断，
	 * 都统一用这个优先级分组，千万不要再分组，切忌。
	 */
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
	/* LED 初始化 */
	//LED_GPIO_Config();
	BASIC_TIM_Init();
	/* 串口初始化	*/
	ALL_UART_Config();

	BEEP_Init();

	
	/*蓝牙模块初始化*/
	//ATKBLE02_init();
	//ltemodule_USART_Config();
	/*CAN总线初始化*/
	//CANinit(500);
	
	//ltemodule_MQTT_init();

	//MPU6050_TEST();
	//MPU6050_ENABLE();//使能MP[U6050模块，初始化IIC总线，设置模块工作模式，使能dmp
	


}

//void MPU6050_TEST(void){
//	
//	float pitch,roll,yaw; 		//欧拉角
//	short aacx,aacy,aacz;		//加速度传感器原始数据
//	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
//	short temp;					//温度	
//	
//	while(MPU_Init())
// 	{
//		printf("MPU_Init Error\r\n");
// 		delay_ms(1000);
//	} 				//初始化MPU6050
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
//			temp=MPU_Get_Temperature();	//得到温度值
//			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
//			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
//			//if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//用自定义帧发送加速度和陀螺仪原始数据
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

