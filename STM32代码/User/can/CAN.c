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

#define		PID0E					(0x0e)			//�����ǰ��
#define		PID1F					(0x1f)			//����������ʱ��
#define		PID23					(0x23)			//�͹�ѹ��
#define		PID2F					(0x2f)			//ʣ������
#define		PID49					(0x49)			//����̤��λ��D
#define		PID5A					(0x5a)			//��Լ���̤��λ��
#define		PID5C					(0x6c)			//�����������¶�
#define		PID5D					(0x5d)			//ȼ��������ʱ��
#define		PID5E					(0x5e)			//ȼ��������
#define		PID83					(0x83)			//NOx������

//OBDdataType OBDdata[20]=
//{
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID04,"LOAD_PCT",Engineload_Formula},				//0				����������
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID05,"ECT",Coolant_temperature_Formula},		//1				��ȴҺ�¶�
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID06,"SHRTFT1",SHRTFT1_Formula},						//2				����ȼ������
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID07,"LONGFT1",LONGFT1_Formula},						//3				��ʱȼ������
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID0B,"MAP",Intake_abspressure},						//				������ܾ���ѹ��
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID0C,"RPM",RPM_Formula},										//3				������ת��
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID0D,"VSS",Speed_Formula},									//4				����
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID0F,"IAT",Intake_airtmep},								//5				������ܿ����¶�
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID10,"MAF",Airflow_Formula},								//6				��������
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID11,"TP",Throttle_position},							//7				�����ſ���
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID0E,"SPARKADV",SPARKADV_Formula},				//7				�����ǰ��
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID1F,"RUNTM",RUNTM_Formula},				//7				����������ʱ��
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID23,"FRP",FRP_Formula},				//7				�����ǰ��
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID2F,"FLI",FLI_Formula},				//7				�����ǰ��
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID49,"APP_D",APP_D_Formula},				//7				�����ǰ��
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID5A,"APP_R",APP_R_Formula},				//7				�����ǰ��
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID5C,"EOT",EOT_Formula},				//7				�����ǰ��
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID5D,"FUEL_TIMING",FUEL_TIMING_Formula},				//7				�����ǰ��
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID5E,"FUEL_RATE",FUEL_RATE_Formula},				//7				�����ǰ��
//	{CAN_Id_Standard,0x7df,0x0000,ServiceID01,PID83,"NOx",NOx_Formula},				//7				�����ǰ��	
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
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID04,1,"LOAD_PCT",Command_Formula},				//0				����������
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID05,1,"ECT",Command_Formula},		//1				��ȴҺ�¶�
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID06,1,"SHRTFT1",Command_Formula},						//2				����ȼ������
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID07,1,"LONGFT1",Command_Formula},						//3				��ʱȼ������
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID0A,1,"FP",Command_Formula},				//7				�͹�ѹ��(��ѹ)
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID0B,2,"MAP",Command_Formula},						//				������ܾ���ѹ��
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID0C,2,"RPM",Command_Formula},										//3				������ת��
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID0D,1,"VSS",Command_Formula},									//4				����
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID0F,1,"IAT",Command_Formula},								//5				������ܿ����¶�
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID10,2,"MAF",Command_Formula},								//6				��������
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID11,1,"TP",Command_Formula},							//7				�����ſ���
	{CAN_Id_Standard,0x7e8,0x18DB33F1,ServiceID01,PID0E,1,"SPARKADV",Command_Formula},				//7				�����ǰ��
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID1F,2,"RUNTM",Command_Formula},				//7				����������ʱ��
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID23,2,"FRP",Command_Formula},				//7				�����ǰ��
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID2F,1,"FLI",Command_Formula},				//7				�����ǰ��
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID49,1,"APP_D",Command_Formula},				//7				�����ǰ��
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID5A,1,"APP_R",Command_Formula},				//7				�����ǰ��
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID5C,1,"EOT",Command_Formula},				//7				�����ǰ��
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID5D,2,"FUEL_TIMING",Command_Formula},				//7				�����ǰ��
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID5E,2,"FUEL_RATE",Command_Formula},				//7				�����ǰ��
	{CAN_Id_Standard,0x7df,0x18DB33F1,ServiceID01,PID83,2,"NOx",Command_Formula},				//7				�����ǰ�
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
		//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�������ȼ����� ѡ����2  2λ��ռʽ���ȼ� 2
		NVIC_InitStructure.NVIC_IRQChannel=USB_LP_CAN1_RX0_IRQn;//ѡ���ж�Դ�����޸ģ�
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
		//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);			
}

/**************************************************************************************************************
  CAN1 Config    CAN1_RX PA11  CAN1_TX  PA12
  FIFO_0	  
 * ��������CAN1_Config
 * ����  ��CAN1�ĳ�ʼ����
 * ����  ��sjw��ͬ����Ծ��ȣ�һ�㶨��Ϊ1
					 bs1��ʱ���1.�ɴ����κ���λ�����1Ҳ����PROP_SEG��PHASE_SEG1���
					 bs2����λ�����2��PHASE_SEG2
					 pres����Ƶϵ����ABH1��Ƶ��Ϊ36M
 * ���  : ��
 * ��ע  ��CAN1�����ʼ���
*****************************************************************************************************************/
void CAN1_Config(uint32_t Baud)
{
		GPIO_InitTypeDef       GPIO_InitStructure;
		CAN_InitTypeDef        CAN_InitStructure;
		CAN_FilterInitTypeDef  CAN_FilterInitStructure;
		int Prescaler = (1000/Baud)*4;
		

		/* ��GPIOʱ�ӡ�AFIOʱ�ӣ�CANʱ�� */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);


		/* CAN1 RX PB8 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		/* CAN1 TX PB9 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//�����������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

//		GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);  // CAN1 remap
									
		/* CAN  BaudRate = RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler */
		CAN_DeInit(CAN1);
		CAN_StructInit(&CAN_InitStructure);   
		CAN_InitStructure.CAN_TTCM=DISABLE;              //MCR-TTCM  �ر�ʱ�䴥��ͨ��ģʽʹ��
		CAN_InitStructure.CAN_ABOM=DISABLE;              //MCR-ABOM  �Զ����߹���
		CAN_InitStructure.CAN_AWUM=DISABLE;              //MCR-AWUM  ʹ���Զ�����ģʽ
		CAN_InitStructure.CAN_NART=DISABLE;              //MCR-NART  ��ֹ�����Զ��ش�	  DISABLE-�Զ��ش�              
		CAN_InitStructure.CAN_RFLM=DISABLE;              //MCR-RFLM  ����FIFO ����ģʽ  DISABLE-���ʱ�±��ĻḲ��ԭ�б���
		CAN_InitStructure.CAN_TXFP=DISABLE;              //MCR-TXFP  ����FIFO���ȼ� DISABLE-���ȼ�ȡ���ڱ��ı�ʾ��
		CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;   
		//CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;
		CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;                   //CAN_InitStructure.CAN_SJW=CAN_SJW_2tq; BTR-SJW ����ͬ����Ծ��� 2��ʱ�䵥Ԫ
		
	/* ss=1 bs1=5 bs2=3 λʱ����Ϊ(1+5+3) �����ʼ�Ϊʱ������tq*(1+3+5)  */		
		CAN_InitStructure.CAN_BS1=CAN_BS1_5tq;           //CAN_InitStructure.CAN_BS1= CAN_BS1_5tq;//BTR-TS1 ʱ���1 ռ����5��ʱ�䵥Ԫ  
		CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;           //CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;//BTR-TS1 ʱ���2 ռ����3��ʱ�䵥Ԫ
  /* CAN Baudrate = 1 MBps (1MBps��Ϊstm32��CAN�������) (CAN ʱ��Ƶ��Ϊ APB1 = 36 MHz) */
	
//		CAN_InitStructure.CAN_Prescaler =4;		 //BTR-BRP �����ʷ�Ƶ��  ������ʱ�䵥Ԫ��ʱ�䳤�� 36/(1+5+3)/4=1 Mbps
//		CAN_InitStructure.CAN_Prescaler =32;

		CAN_InitStructure.CAN_Prescaler =Prescaler;


//CAN_InitStructure.CAN_Prescaler=pres;
		CAN_Init(CAN1,&CAN_InitStructure);	// CAN1											

/*
 * ��������CAN_Filter_Config
 * ����  ��CAN�Ĺ����� ����*/

		CAN_FilterInitStructure.CAN_FilterNumber=0;	 
		CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	 // ��ʶ������λģʽ
		CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   // 32λ������
		CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;			// ��������ʶ��
		CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;				
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;		// ���������α�ʶ��
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;	 // FIFO0ָ�������
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
	
		cJSON_AddItemToObject(root, "VSS",cJSON_CreateNumber(VSS));//���name�ڵ�
    cJSON_AddItemToObject(root, "RPM",cJSON_CreateNumber(RPM) );//���ڵ������
    cJSON_AddItemToObject(root, "TP", cJSON_CreateNumber(TP));
    cJSON_AddItemToObject(root, "LOAD_PCT",cJSON_CreateNumber(LOAD_PCT));
    cJSON_AddItemToObject(root, "MAP",cJSON_CreateNumber(MAP));
    cJSON_AddItemToObject(root, "MAF", cJSON_CreateNumber(MAF));//root�ڵ������semantic�ڵ�
    cJSON_AddItemToObject(root, "timestamp", cJSON_CreateNumber(timestamp));//semantic�ڵ������item�ڵ�

 
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
//			�����ǰ��
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

//7				����������ʱ��
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

//�͹�ѹ��
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

//ʣ������
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

//����̤��λ��D
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

//��Լ���̤��λ��
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

//�����¶�
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
//ȼ��������ʱ��
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

//ȼ��������
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



