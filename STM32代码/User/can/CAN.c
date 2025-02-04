#include "CAN.h"
#include "stm32f10x.h" 
#include "bsp_usart.h"
#include <string.h>
CanRxMsg CANRXMSG_1;
CanRxMsg CANRXMSG_2;
CanRxMsg CANRXMSG_3;
CanRxMsg CANRXMSG_4;
CanRxMsg CANRXMSG_5;
CanRxMsg CANRXMSG_6;
CanRxMsg CANRXMSG_7;
CanRxMsg CANRXMSG_8;
uint8_t volatile MESTRUCT_flag_5;
uint8_t volatile MESTRUCT_flag_6;
uint8_t volatile MESTRUCT_flag_7;
uint8_t volatile MESTRUCT_flag_8;
uint8_t volatile MESTRUCT_flag_9;
uint8_t volatile MESTRUCT_flag_10;
uint8_t volatile MESTRUCT_flag_11;
uint8_t volatile MESTRUCT_flag_12;

#define		PID0E					(0x0e)			//点火提前角
#define		PID1F					(0x1f)			//发动机启动时间
#define		PID23					(0x23)			//油轨压力
#define		PID2F					(0x2f)			//剩余油量
#define		PID49					(0x49)			//加速踏板位置D
#define		PID5A					(0x5a)			//相对加速踏板位置
#define		PID5C					(0x6c)			//发动机机油温度
#define		PID5D					(0x5d)			//燃油喷射正时角
#define		PID5E					(0x5e)			//燃油消耗率
#define		PID83					(0x83)			//NOx传感器

//OBDdataType OBDdata[20]=
//{
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID04,"LOAD_PCT",Engineload_Formula},				//0				发动机负载
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID05,"ECT",Coolant_temperature_Formula},		//1				冷却液温度
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID06,"SHRTFT1",SHRTFT1_Formula},						//2				短期燃油修正
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID07,"LONGFT1",LONGFT1_Formula},						//3				长时燃油修正
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID0B,"MAP",Intake_abspressure},						//				进气歧管绝对压力
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID0C,"RPM",RPM_Formula},										//3				发动机转速
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID0D,"VSS",Speed_Formula},									//4				车速
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID0F,"IAT",Intake_airtmep},								//5				进气歧管空气温度
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID10,"MAF",Airflow_Formula},								//6				进气流量
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID11,"TP",Throttle_position},							//7				节气门开度
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID0E,"SPARKADV",SPARKADV_Formula},				//7				点火提前角
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID1F,"RUNTM",RUNTM_Formula},				//7				发动机启动时间
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID23,"FRP",FRP_Formula},				//7				点火提前角
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID2F,"FLI",FLI_Formula},				//7				点火提前角
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID49,"APP_D",APP_D_Formula},				//7				点火提前角
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID5A,"APP_R",APP_R_Formula},				//7				点火提前角
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID5C,"EOT",EOT_Formula},				//7				点火提前角
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID5D,"FUEL_TIMING",FUEL_TIMING_Formula},				//7				点火提前角
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID5E,"FUEL_RATE",FUEL_RATE_Formula},				//7				点火提前角
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID83,"NOx",NOx_Formula},				//7				点火提前角	
//};

//char OBDdataHz_1[20]={	1,1,1,1,1,
//												1,1,1,1,1,
//												1,1,1,1,1,
//												1,1,1,1,1,};

void Command_Formula(void* OBDdatadef, CanRxMsg tempCANRxMSG,cJSON * root){
	cJSON * Arry;
	OBDdataTypeNew *OBDdata=(OBDdataTypeNew *)OBDdatadef;
	int num=OBDdata->datanum;
	int numarry[num];
	int i=0;
	if(tempCANRxMSG.Data[1]!=(OBDdata->SID|0x40)||tempCANRxMSG.Data[2]!=OBDdata->PID){
		printf("%s resolve error!\r\n",OBDdata->dataname);
		printf("tempCANRxMSG.Data[1]:%#x  tempCANRxMSG.Data[2]:%#x\r\n",tempCANRxMSG.Data[1],tempCANRxMSG.Data[2]);
	}	
	for(i=0;i<num;i++){
		numarry[i]=tempCANRxMSG.Data[3+i];
	}
	Arry=cJSON_CreateIntArray(numarry,OBDdata->datanum);
	cJSON_AddItemToObject(root,OBDdata->dataname,Arry);
	return;
}

uint8_t  numofask=23 ;
OBDdataTypeNew newOBDdata[25]=
{//0x7df
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID01,1,"DTC_CNT",Command_Formula},
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID04,1,"LOAD_PCT",Command_Formula},				//0				发动机负载
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID05,1,"ECT",Command_Formula},		//1				冷却液温度
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID06,1,"SHRTFT1",Command_Formula},						//2				短期燃油修正
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID07,1,"LONGFT1",Command_Formula},						//3				长时燃油修正
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID0A,1,"FP",Command_Formula},				//7				油轨压力(表压)
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID0B,2,"MAP",Command_Formula},						//				进气歧管绝对压力
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID0C,2,"RPM",Command_Formula},										//3				发动机转速
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID0D,1,"VSS",Command_Formula},									//4				车速
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID0F,1,"IAT",Command_Formula},								//5				进气歧管空气温度
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID10,2,"MAF",Command_Formula},								//6				进气流量
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID11,1,"TP",Command_Formula},							//7				节气门开度
	{CAN_Id_Standard,0x7e8,0x18DB33F1,ServiceID01,PID0E,1,"SPARKADV",Command_Formula},				//7				点火提前角
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID1F,2,"RUNTM",Command_Formula},				//7				发动机启动时间
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID23,2,"FRP",Command_Formula},				//7				点火提前角
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID2F,1,"FLI",Command_Formula},				//7				点火提前角
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID49,1,"APP_D",Command_Formula},				//7				点火提前角
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID5A,1,"APP_R",Command_Formula},				//7				点火提前角
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID5C,1,"EOT",Command_Formula},				//7				点火提前角
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID5D,2,"FUEL_TIMING",Command_Formula},				//7				点火提前角
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID5E,2,"FUEL_RATE",Command_Formula},				//7				点火提前角
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID83,2,"NOx",Command_Formula},				//7				点火提前�
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID03,PID01,2,"DCTs",Command_Formula},
};


char OBDdataHz_new[25]={	1,1,1,1,1,
													1,1,1,1,1,
													1,1,1,1,1,
													1,1,1,1,1,
													1,1,1,1,1,
																		};

int sum_OBDdataHz_new(char *arry,char size){
	int i=0;
	int sum=0;
	for(i=0;i<size;i++){
		sum+=arry[i];
	}
	return sum;
}
extern QueueHandle_t CANmsg_Queue;

static void CAN1_NVICConfig(void)
{
		NVIC_InitTypeDef       NVIC_InitStructure;

			/* CAN1 Enabling interrupt */		
		//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置优先级分组 选择组2  2位抢占式优先级 2
		NVIC_InitStructure.NVIC_IRQChannel=USB_LP_CAN1_RX0_IRQn;//选择中断源（待修改）
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
		//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);			
}

/**************************************************************************************************************
  CAN1 Config    CAN1_RX PA11  CAN1_TX  PA12
  FIFO_0	  
 * 函数名：CAN1_Config
 * 描述  ：CAN1的初始化化
 * 输入  ：sjw：同步跳跃宽度，一般定义为1
					 bs1：时间段1.由传播段和相位缓冲段1也就是PROP_SEG和PHASE_SEG1组成
					 bs2；相位缓冲段2即PHASE_SEG2
					 pres：分频系数，ABH1的频率为36M
 * 输出  : 无
 * 备注  ：CAN1波特率计算
*****************************************************************************************************************/
void CAN1_Config(uint32_t Baud)
{
		GPIO_InitTypeDef       GPIO_InitStructure;
		CAN_InitTypeDef        CAN_InitStructure;
		CAN_FilterInitTypeDef  CAN_FilterInitStructure;
		int Prescaler = (1000/Baud)*4;
		

		/* 打开GPIO时钟、AFIO时钟，CAN时钟 */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);


		/* CAN1 RX PB8 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//上拉输入
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		/* CAN1 TX PB9 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

//		GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);  // CAN1 remap
									
		/* CAN  BaudRate = RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler */
		CAN_DeInit(CAN1);
		CAN_StructInit(&CAN_InitStructure);   
		CAN_InitStructure.CAN_TTCM=DISABLE;              //MCR-TTCM  关闭时间触发通信模式使能
		CAN_InitStructure.CAN_ABOM=DISABLE;              //MCR-ABOM  自动离线管理
		CAN_InitStructure.CAN_AWUM=DISABLE;              //MCR-AWUM  使用自动唤醒模式
		CAN_InitStructure.CAN_NART=DISABLE;              //MCR-NART  禁止报文自动重传	  DISABLE-自动重传              
		CAN_InitStructure.CAN_RFLM=DISABLE;              //MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文
		CAN_InitStructure.CAN_TXFP=DISABLE;              //MCR-TXFP  发送FIFO优先级 DISABLE-优先级取决于报文标示符
		CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;   
		//CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;
		CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;                   //CAN_InitStructure.CAN_SJW=CAN_SJW_2tq; BTR-SJW 重新同步跳跃宽度 2个时间单元
		
	/* ss=1 bs1=5 bs2=3 位时间宽度为(1+5+3) 波特率即为时钟周期tq*(1+3+5)  */		
		CAN_InitStructure.CAN_BS1=CAN_BS1_5tq;           //CAN_InitStructure.CAN_BS1= CAN_BS1_5tq;//BTR-TS1 时间段1 占用了5个时间单元  
		CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;           //CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;//BTR-TS1 时间段2 占用了3个时间单元
  /* CAN Baudrate = 1 MBps (1MBps已为stm32的CAN最高速率) (CAN 时钟频率为 APB1 = 36 MHz) */
	
//		CAN_InitStructure.CAN_Prescaler =4;		 //BTR-BRP 波特率分频器  定义了时间单元的时间长度 36/(1+5+3)/4=1 Mbps
//		CAN_InitStructure.CAN_Prescaler =32;

		CAN_InitStructure.CAN_Prescaler =Prescaler;


//CAN_InitStructure.CAN_Prescaler=pres;
		CAN_Init(CAN1,&CAN_InitStructure);	// CAN1											

/*
 * 函数名：CAN_Filter_Config
 * 描述  ：CAN的过滤器 配置*/

		CAN_FilterInitStructure.CAN_FilterNumber=0;	 
		CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	 // 标识符屏蔽位模式
		CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   // 32位过滤器
		CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;			// 过滤器标识符
		CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;				
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;		// 过滤器屏蔽标识符
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;	 // FIFO0指向过滤器
		CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
		CAN_FilterInit(&CAN_FilterInitStructure);
		CAN1_NVICConfig();
		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);  // CAN1
}

void CANinit(uint32_t Baud){
	//CAN1_NVICConfig();
	CAN1_Config(Baud);
	return;
}


void USB_LP_CAN1_RX0_IRQHandler()
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	CanRxMsg tempCANMSG;
	
	if( CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET )
	{
		CAN_Receive(CAN1, CAN_FIFO0, &tempCANMSG);
		xQueueSendFromISR( CANmsg_Queue, &tempCANMSG, &xHigherPriorityTaskWoken );
	}	
	CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}


void CANTransmit_frame(uint32_t Id,  uint8_t IDE, uint8_t *data,uint8_t datalen){
	CanTxMsg tempCANTxMSG;
	if(IDE==CAN_Id_Standard){
		tempCANTxMSG.StdId=Id;
	}else{
		tempCANTxMSG.ExtId=Id;
	}
	tempCANTxMSG.IDE=IDE;
	tempCANTxMSG.RTR=CAN_RTR_Data;
	tempCANTxMSG.DLC=8;
	for(int i=0;i<datalen;i++){
		tempCANTxMSG.Data[i]=data[i];
	}	
	CAN_Transmit(CAN1, &tempCANTxMSG);
	return;
}


void OBDTransmit_askframe(uint32_t Id,  uint8_t IDE, uint8_t SID,uint8_t PID){
	CanTxMsg tempCANTxMSG;
	memset(&tempCANTxMSG,0x00,sizeof(CanTxMsg));
	if(IDE==CAN_Id_Standard){
		tempCANTxMSG.StdId=Id;
	}else{
		tempCANTxMSG.ExtId=0x18DB33F1;
	}
	tempCANTxMSG.IDE=IDE;
	tempCANTxMSG.RTR=CAN_RTR_Data;
	tempCANTxMSG.DLC=8;
	tempCANTxMSG.Data[0]=02;
	tempCANTxMSG.Data[1]=SID;
	tempCANTxMSG.Data[2]=PID;
	
	CAN_Transmit(CAN1, &tempCANTxMSG);
	return;
}
void AddOBDdata_to_json(char *retarr,uint8_t VSS,uint32_t RPM,float TP,float LOAD_PCT,uint8_t MAP,float MAF,uint64_t timestamp){
		char *retchar=NULL;
    cJSON * root =  cJSON_CreateObject();
	
		cJSON_AddItemToObject(root, "VSS",cJSON_CreateNumber(VSS));//添加name节点
    cJSON_AddItemToObject(root, "RPM",cJSON_CreateNumber(RPM) );//根节点下添加
    cJSON_AddItemToObject(root, "TP", cJSON_CreateNumber(TP));
    cJSON_AddItemToObject(root, "LOAD_PCT",cJSON_CreateNumber(LOAD_PCT));
    cJSON_AddItemToObject(root, "MAP",cJSON_CreateNumber(MAP));
    cJSON_AddItemToObject(root, "MAF", cJSON_CreateNumber(MAF));//root节点下添加semantic节点
    cJSON_AddItemToObject(root, "timestamp", cJSON_CreateNumber(timestamp));//semantic节点下添加item节点

 
		retchar= cJSON_Print(root);
		printf("strlen(retchar):%d\r\n",strlen(retchar));
    printf("%s\n", retchar);;
		strcpy(retarr, retchar);
		cJSON_Delete(root);
		vPortFree(retchar);
		printf("xPortGetFreeHeapSize():%d\r\n",xPortGetFreeHeapSize());
		
		return;
}

void Speed_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root){
	cJSON * Arry;
	int numarry[2];
	if(tempCANRxMSG.Data[1]!=0x41||tempCANRxMSG.Data[2]!=0x0D){
		printf("Speed_Formula error!\r\n");
	}

//	*(uint8_t *)ret=tempCANRxMSG.Data[3];
	numarry[0]=tempCANRxMSG.Data[3];
	Arry=cJSON_CreateIntArray(numarry,1);
	cJSON_AddItemToObject(root,"VSS",Arry);
	return ;
}


void RPM_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root){
	//CanRxMsg tempCANTxMSG;
	cJSON * Arry;
	int numarry[2];
	if(tempCANRxMSG.Data[1]!=0x41||tempCANRxMSG.Data[2]!=0x0C){
		printf("RPM_Formula error!\r\n");
	}

//	*(uint32_t *)ret=(tempCANRxMSG.Data[3]*256+tempCANRxMSG.Data[4])>>2;	

	numarry[0]=tempCANRxMSG.Data[3];
	numarry[1]=tempCANRxMSG.Data[4];
	Arry=cJSON_CreateIntArray(numarry,2);
	cJSON_AddItemToObject(root,"PRM",Arry);
	return;
}


void Engineload_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root){
	cJSON * Arry;
	int numarry[2];
	if(tempCANRxMSG.Data[1]!=0x41||tempCANRxMSG.Data[2]!=0x04){
		printf("LOAD_PCT error!\r\n");
	}

//	*(float *)ret=(float)tempCANRxMSG.Data[3]*100/255;

	numarry[0]=tempCANRxMSG.Data[3];
	Arry=cJSON_CreateIntArray(numarry,1);
	cJSON_AddItemToObject(root,"LOAD_PCT",Arry);
	return;
}

void  Coolant_temperature_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root){
	//CanRxMsg tempCANTxMSG;
	cJSON * Arry;
	int numarry[1];
	if(tempCANRxMSG.Data[1]!=0x41||tempCANRxMSG.Data[2]!=0x05){
		printf("ECT error!\r\n");
	}

//	*(float *)ret=(float)tempCANRxMSG.Data[3]-40;

	numarry[0]=tempCANRxMSG.Data[3];
	Arry=cJSON_CreateIntArray(numarry,1);
	cJSON_AddItemToObject(root,"ECT",Arry);
	return;
}

void SHRTFT1_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root){
	cJSON * Arry;
	int numarry[1];
	
	if(tempCANRxMSG.Data[1]!=0x41||tempCANRxMSG.Data[2]!=0x06){
		printf("SHRTFT1 error!r\n");
	}
//*(float *)ret=((float)tempCANRxMSG.Data[3]-128)/128;
	numarry[0]=tempCANRxMSG.Data[3];
	Arry=cJSON_CreateIntArray(numarry,1);
	cJSON_AddItemToObject(root,"SHRTFT1",Arry);
	return;
}

void LONGFT1_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root){
	cJSON * Arry;
	int numarry[1];
	if(tempCANRxMSG.Data[1]!=0x41||tempCANRxMSG.Data[2]!=0x07){
		printf("LONGFT1 error!\r\n");
	}
//*(float *)ret=((float)tempCANRxMSG.Data[3]-128)/128;
	numarry[0]=tempCANRxMSG.Data[3];
	Arry=cJSON_CreateIntArray(numarry,1);
	cJSON_AddItemToObject(root,"LONGFT1",Arry);
	return;
}

void  Airflow_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root){
	//CanRxMsg tempCANTxMSG;
	cJSON * Arry;
	int numarry[2];
	if(tempCANRxMSG.Data[1]!=0x41||tempCANRxMSG.Data[2]!=0x10){
		printf("MAF error!\r\n");
	}
//	*(float *)ret=((float)tempCANRxMSG.Data[3]*256+(float)tempCANRxMSG.Data[4])/100;
	numarry[0]=tempCANRxMSG.Data[3];
	numarry[1]=tempCANRxMSG.Data[4];
	Arry=cJSON_CreateIntArray(numarry,2);
	cJSON_AddItemToObject(root,"ECT",Arry);
	return;
}


void  Intake_abspressure(CanRxMsg tempCANRxMSG,void *ret,cJSON * root){
	cJSON * Arry;
	int numarry[2];
	if(tempCANRxMSG.Data[1]!=0x41||tempCANRxMSG.Data[2]!=0x0B){
		printf("MAP error!\r\n");
	}
//	*(uint8_t *)ret=tempCANRxMSG.Data[3];
	numarry[0]=tempCANRxMSG.Data[3];
	numarry[1]=tempCANRxMSG.Data[4];
	Arry=cJSON_CreateIntArray(numarry,2);
	cJSON_AddItemToObject(root,"MAP",Arry);
	return;
}

void  Throttle_position(CanRxMsg tempCANRxMSG,void *ret,cJSON * root){

	cJSON * Arry;
	int numarry[1];
	if(tempCANRxMSG.Data[1]!=0x41||tempCANRxMSG.Data[2]!=0x11){
		printf("TP error!\r\n");
	}

//	*(float *)ret=(float)tempCANRxMSG.Data[3]*100/255;
	numarry[0]=tempCANRxMSG.Data[3];
	Arry=cJSON_CreateIntArray(numarry,1);
	cJSON_AddItemToObject(root,"TP",Arry);
	return;
}

void  Intake_airtmep(CanRxMsg tempCANRxMSG,void *ret,cJSON * root){
	cJSON * Arry;
	int numarry[1];
	if(tempCANRxMSG.Data[1]!=0x41||tempCANRxMSG.Data[2]!=0x0f){
		printf("IAT error!\r\n");
	}

//	*(float *)ret=(float)tempCANRxMSG.Data[3]-40;
	numarry[0]=tempCANRxMSG.Data[3];
	Arry=cJSON_CreateIntArray(numarry,1);
	cJSON_AddItemToObject(root,"IAT",Arry);
	return;
}
//			点火提前角
void  SPARKADV_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root){
	cJSON * Arry;
	int numarry[1];
	if(tempCANRxMSG.Data[1]!=0x41||tempCANRxMSG.Data[2]!=0x0E){
		printf("SPARKADV error!\r\n");
	}

//	*(float *)ret=((float)tempCANRxMSG.Data[3]-128)/2;//(DATA[0]-128)/2	
	numarry[0]=tempCANRxMSG.Data[3];
	Arry=cJSON_CreateIntArray(numarry,1);
	cJSON_AddItemToObject(root,"SPARKADV",Arry);
	return;
}

//7				发动机启动时间
void  RUNTM_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root){
	cJSON * Arry;
	int numarry[2];
	if(tempCANRxMSG.Data[1]!=0x41||tempCANRxMSG.Data[2]!=0x1F){
		printf("RUNTM error!\r\n");
	}

//	*(float *)ret=((float)tempCANRxMSG.Data[3]*256+tempCANRxMSG.Data[4];//DATA[0]*256+DATA[1]
	numarry[0]=tempCANRxMSG.Data[3];
	numarry[1]=tempCANRxMSG.Data[4];
	Arry=cJSON_CreateIntArray(numarry,2);
	cJSON_AddItemToObject(root,"RUNTM",Arry);
	return;
}

//油轨压力
void  FRP_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root){
	cJSON * Arry;
	int numarry[2];
	if(tempCANRxMSG.Data[1]!=0x41||tempCANRxMSG.Data[2]!=0x23){
		printf("FRP error!\r\n");
	}

//	*(float *)ret=tempCANRxMSG.Data[3]*256+tempCANRxMSG.Data[4];//DATA[0]*256+DATA[1]
	numarry[0]=tempCANRxMSG.Data[3];
	numarry[1]=tempCANRxMSG.Data[4];
	Arry=cJSON_CreateIntArray(numarry,2);
	cJSON_AddItemToObject(root,"FRP",Arry);
	return;
}

//剩余油量
void  FLI_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root){
	cJSON * Arry;
	int numarry[1];
	if(tempCANRxMSG.Data[1]!=0x41||tempCANRxMSG.Data[2]!=0x2F){
		printf("FLI error!\r\n");
	}

//	*(float *)ret=(float)(tempCANRxMSG.Data[3]*100)/255;//DATA[0]*100/255
	numarry[0]=tempCANRxMSG.Data[3];
	Arry=cJSON_CreateIntArray(numarry,1);
	cJSON_AddItemToObject(root,"FLI",Arry);
	return;
}

//加速踏板位置D
void  APP_D_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root){
	cJSON * Arry;
	int numarry[1];
	if(tempCANRxMSG.Data[1]!=0x41||tempCANRxMSG.Data[2]!=0x49){
		printf("APP_D error!\r\n");
	}

//	*(float *)ret=(float)(tempCANRxMSG.Data[3]*100)/255;//DATA[0]*100/255
	numarry[0]=tempCANRxMSG.Data[3];
	Arry=cJSON_CreateIntArray(numarry,1);
	cJSON_AddItemToObject(root,"APP_D",Arry);
	return;
}

//相对加速踏板位置
void  APP_R_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root){
	cJSON * Arry;
	int numarry[1];
	if(tempCANRxMSG.Data[1]!=0x41||tempCANRxMSG.Data[2]!=0x5A){
		printf("APP_R error!\r\n");
	}

//	*(float *)ret=(float)(tempCANRxMSG.Data[3]*100)/255;//DATA[0]*100/255
	numarry[0]=tempCANRxMSG.Data[3];
	Arry=cJSON_CreateIntArray(numarry,1);
	cJSON_AddItemToObject(root,"APP_R",Arry);
	return;
}

//机油温度
void  EOT_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root){
	cJSON * Arry;
	int numarry[1];
	if(tempCANRxMSG.Data[1]!=0x41||tempCANRxMSG.Data[2]!=0x5C){
		printf("EOT error!\r\n");
	}

//	*(float *)ret=tempCANRxMSG.Data[3]-40;//DATA[0]-40	
	numarry[0]=tempCANRxMSG.Data[3];
	Arry=cJSON_CreateIntArray(numarry,1);
	cJSON_AddItemToObject(root,"EOT",Arry);
	return;
}
//燃油喷射正时角
void  FUEL_TIMING_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root){
	cJSON * Arry;
	int numarry[2];
	if(tempCANRxMSG.Data[1]!=0x41||tempCANRxMSG.Data[2]!=0x5D){
		printf("FUEL_TIMING error!\r\n");
	}

//	*(float *)ret=((float)(tempCANRxMSG.Data[3]*256+tempCANRxMSG.Data[3]-26880))/128;//(DATA[0]*256+DATA[1]-26880)/128	
	numarry[0]=tempCANRxMSG.Data[3];
	numarry[1]=tempCANRxMSG.Data[4];
	Arry=cJSON_CreateIntArray(numarry,2);
	cJSON_AddItemToObject(root,"FUEL_TIMING",Arry);
	return;
}

//燃油消耗率
void  FUEL_RATE_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root){
	cJSON * Arry;
	int numarry[2];
	if(tempCANRxMSG.Data[1]!=0x41||tempCANRxMSG.Data[2]!=0x5E){
		printf("FUEL_RATE error!\r\n");
	}

//	*(float *)ret=((float)(tempCANRxMSG.Data[3]*256+tempCANRxMSG.Data[3]))*0.05;(DATA[0]*256+DATA[1])*0.05
	numarry[0]=tempCANRxMSG.Data[3];
	numarry[1]=tempCANRxMSG.Data[4];
	Arry=cJSON_CreateIntArray(numarry,2);
	cJSON_AddItemToObject(root,"FUEL_RATE",Arry);
	return;
}

//NOx
void  NOx_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root){
	cJSON * Arry;
	int numarry[2];
	if(tempCANRxMSG.Data[1]!=0x41||tempCANRxMSG.Data[2]!=0x83){
		printf("NOx error!\r\n");
	}

//	*(float *)ret=((float)(tempCANRxMSG.Data[3]*256+tempCANRxMSG.Data[3]))*0.05;(DATA[0]*256+DATA[1])*0.05
	numarry[0]=tempCANRxMSG.Data[3];
	numarry[1]=tempCANRxMSG.Data[4];
	Arry=cJSON_CreateIntArray(numarry,2);
	cJSON_AddItemToObject(root,"NOx",Arry);
	return;
}



