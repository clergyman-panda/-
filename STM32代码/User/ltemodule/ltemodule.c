#include "ltemodule.h"
#include <string.h>
#include "cJSON.h"
#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
 /**
  * @brief  ����Ƕ�������жϿ�����NVIC
  * @param  ��
  * @retval ��
  */
volatile u8 ltemodule_USART_state=0;//0x01���յ���һ��0d��0x02���յ���һ��0x0a��0x04���յ��ڶ���0x0d,0x08���յ��ڶ���0a
char ltemodule_USART_buf[50];
char ltemodule_USART_num=0;
uint64_t uinx_timestamp;

static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Ƕ�������жϿ�������ѡ�� */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  
  /* ����USARTΪ�ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = ltemodule_USART_IRQ;
  /* �������ȼ�*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  /* �����ȼ� */
 // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
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
void ltemodule_USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// �򿪴���GPIO��ʱ��
	ltemodule_USART_GPIO_APBxClkCmd(ltemodule_USART_GPIO_CLK, ENABLE);
	
	// �򿪴��������ʱ��
	ltemodule_USART_APBxClkCmd(ltemodule_USART_CLK, ENABLE);

	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = ltemodule_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ltemodule_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  // ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = ltemodule_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(ltemodule_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	
	// ���ô��ڵĹ�������
	// ���ò�����
	USART_InitStructure.USART_BaudRate = ltemodule_USART_BAUDRATE;
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
	USART_Init(ltemodule_USARTx, &USART_InitStructure);
	
	// �����ж����ȼ�����
	NVIC_Configuration();
	
	// ʹ�ܴ��ڽ����ж�
	//USART_ITConfig(ltemodule_USARTx, USART_IT_RXNE, ENABLE);	
	
	// ʹ�ܴ���
	USART_Cmd(ltemodule_USARTx, ENABLE);	    
}

void ltemodule_USART_IRQHandler(void){

	char res=0;
	if(USART_GetITStatus(ltemodule_USARTx, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		res = USART_ReceiveData(ltemodule_USARTx);	//��ȡ���յ�������	
		//USART_SendData(USART1,res);
		ltemodule_USART_buf[ltemodule_USART_num++]=res;
		if(res==0x0d||res==0x0a){
			ltemodule_USART_state++;
			//printf("ltemodule_USART_state:%d\r\n",ltemodule_USART_state);
			//USART_SendData(USART1,ltemodule_USART_state);
		}
	}
	USART_ClearFlag(ltemodule_USARTx, USART_IT_RXNE);
}


/*****************  ����һ���ֽ� **********************/
void ltemodule_Usart_SendByte(  uint8_t ch)
{
	/* ����һ���ֽ����ݵ�USART */
	USART_SendData(ltemodule_USARTx,ch);
		
	/* �ȴ��������ݼĴ���Ϊ�� */
	while (USART_GetFlagStatus(ltemodule_USARTx, USART_FLAG_TXE) == RESET);	
}


/*****************  �����ַ��� **********************/
void ltemodule_Usart_SendString(char *str)
{
	unsigned int k=0;
  do 
  {
      ltemodule_Usart_SendByte( *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* �ȴ�������� */
  while(USART_GetFlagStatus(ltemodule_USARTx,USART_FLAG_TC)==RESET)
  {}
}

/*****************  ����һ��16λ�� **********************/
void ltemodule_Usart_SendHalfWord(uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* ȡ���߰�λ */
	temp_h = (ch&0XFF00)>>8;
	/* ȡ���Ͱ�λ */
	temp_l = ch&0XFF;
	
	/* ���͸߰�λ */
	USART_SendData(ltemodule_USARTx,temp_h);	
	while (USART_GetFlagStatus(ltemodule_USARTx, USART_FLAG_TXE) == RESET);
	
	/* ���͵Ͱ�λ */
	USART_SendData(ltemodule_USARTx,temp_l);	
	while (USART_GetFlagStatus(ltemodule_USARTx, USART_FLAG_TXE) == RESET);	
}

/*
�ȴ�ģ�鷢��AT Ready
����+++��4gģ�飬�������ATָ��ģʽ���յ�OK�󣬽���ָ��ģʽ�ɹ�
 
*/
void ltemodule_MQTT_init(void){

	ltemodule_USART_Config();
	// ʹ�ܴ��ڽ����ж�
	USART_ITConfig(ltemodule_USARTx, USART_IT_RXNE, ENABLE);	
	printf("4g�ȴ�����\r\n");
	while(ltemodule_USART_state!=4){
	}
	ltemodule_USART_buf[ltemodule_USART_num]=0;
	if(strstr(ltemodule_USART_buf, "Ready")==NULL){
		printf("4gģ������ʧ��\r\n");
	}else{
		printf("4gģ�������ɹ�\r\n");
		printf("ltemodule_USART_state:%d\r\n",ltemodule_USART_state);
	}
	
	memset(ltemodule_USART_buf,0,50);
	ltemodule_USART_num=0;
	ltemodule_USART_state=0;
	
	ltemodule_Usart_SendByte('+');
	ltemodule_Usart_SendByte('+');
	ltemodule_Usart_SendByte('+');
	
	while(ltemodule_USART_state!=4);
	if(strstr(ltemodule_USART_buf, "OK")==NULL){
		printf("4gģ�����ָ������ʧ��\r\n");
	}
	memset(ltemodule_USART_buf,0,50);
	ltemodule_USART_num=0;
	ltemodule_USART_state=0;
	
	while(ltemodule_USART_state!=4);
	if(strstr(ltemodule_USART_buf, "MQTT CONNECTED")==NULL){
		printf("4gģ������ʧ��\r\n");
	}
	
	memset(ltemodule_USART_buf,0,50);
	ltemodule_USART_num=0;
	ltemodule_USART_state=0;
	
	printf("MQTT�ɹ�����MQTT\r\n");
	
	ltemodule_Usart_SendString("AT+TIMESTAMP");
	while(ltemodule_USART_state!=4);
	if(strstr(ltemodule_USART_buf, "OK")==NULL){
		printf("���ʱ���ʧ��\r\n");
	}
	uinx_timestamp=get_timestamp(ltemodule_USART_buf);
	printf("���ʱ���%lld\r\n",uinx_timestamp);
	memset(ltemodule_USART_buf,0,50);
	ltemodule_USART_num=0;
	ltemodule_USART_state=0;


	ltemodule_Usart_SendString("ATO");
	while(ltemodule_USART_state!=4);
	if(strstr(ltemodule_USART_buf, "OK")==NULL){
		printf("��ʼ����\r\n");
	}
	memset(ltemodule_USART_buf,0,50);
	ltemodule_USART_num=0;
	ltemodule_USART_state=0;
	
		// ʹ�ܴ��ڽ����ж�
	USART_ITConfig(ltemodule_USARTx, USART_IT_RXNE, DISABLE);	

}


uint64_t get_timestamp(char *str){
	uint64_t timestamp=0;
	int i=0;
	char *ptr=strchr(str, 0x20);
	ptr++;
	for(i=0;i<10;i++){
		uint8_t num=(*ptr++)-'0';
		timestamp+=num;
		timestamp*=10;
	}
	timestamp*=100;
	return timestamp;
}

uint8_t jlink_send_1s(Info_1secTypeDef Info,char *retstr){
		char *tempstr=NULL;
		cJSON * info_1s_obj =  cJSON_CreateObject();
    cJSON * params_obj =  cJSON_CreateObject();
    cJSON * VehSpeed_obj =  cJSON_CreateObject();
		cJSON * GeoLocation_obj =  cJSON_CreateObject();
		
		cJSON_AddItemToObject(info_1s_obj, "method", cJSON_CreateString("thing.event.property.post"));
    cJSON_AddItemToObject(info_1s_obj, "id", cJSON_CreateNumber(4));//���ڵ������
		cJSON_AddItemToObject(info_1s_obj, "params", params_obj);//root�ڵ������semantic�ڵ�
		cJSON_AddItemToObject(params_obj, "VehSpeed", VehSpeed_obj);//root�ڵ������semantic�ڵ�
		cJSON_AddItemToObject(VehSpeed_obj, "value", cJSON_CreateNumber(64));
		cJSON_AddItemToObject(VehSpeed_obj, "time", cJSON_CreateNumber(1636616545000));
		cJSON_AddItemToObject(params_obj, "VBAT", cJSON_CreateNumber(12));
		cJSON_AddItemToObject(params_obj, "RPM", cJSON_CreateNumber(1500));
		cJSON_AddItemToObject(params_obj, "throttle_opening", cJSON_CreateNumber(54));
		cJSON_AddItemToObject(params_obj, "engine_load", cJSON_CreateNumber(62));
		cJSON_AddItemToObject(params_obj, "ECT", cJSON_CreateNumber(41));
		cJSON_AddItemToObject(params_obj, "POWER", cJSON_CreateNumber(1));
		cJSON_AddItemToObject(params_obj, "VIN", cJSON_CreateString("d4sa55dd7456"));
		cJSON_AddItemToObject(params_obj, "residual_oil_volume", cJSON_CreateNumber(0.42));
		
		cJSON_AddItemToObject(params_obj,	"GeoLocation",GeoLocation_obj);
		cJSON_AddItemToObject(GeoLocation_obj,	"Longitude",cJSON_CreateNumber(107.55));
		cJSON_AddItemToObject(GeoLocation_obj,	"Latitude",cJSON_CreateNumber(30.57));
		cJSON_AddItemToObject(GeoLocation_obj,	"Altitude",cJSON_CreateNumber(0));
		cJSON_AddItemToObject(GeoLocation_obj,	"CoordinateSystem",cJSON_CreateNumber(1));
		
		cJSON_AddItemToObject(info_1s_obj,	"version",cJSON_CreateString("1.0"));
		tempstr= cJSON_Print(info_1s_obj);
		strcpy(retstr,tempstr);
//    printf("%s\n",retstr);
//		printf("retchar len:%d\r\n",strlen(retstr));
	
//		printf("xPortGetFreeHeapSize():%d\r\n",xPortGetFreeHeapSize());
		cJSON_Delete(info_1s_obj);	
		vPortFree(tempstr);
//		printf("xPortGetFreeHeapSize():%d\r\n",xPortGetFreeHeapSize());
		
    return 0;
}

uint8_t jlink_send_1s_test(Info_1secTypeDef Info){
	
		char *retchar=NULL;
    cJSON * root =  cJSON_CreateObject();
		printf("sizeof(cJSON):%d\r\n",sizeof(cJSON));
    cJSON * item =  cJSON_CreateObject();
    cJSON * next =  cJSON_CreateObject();
 
    cJSON_AddItemToObject(root, "rc", cJSON_CreateNumber(0));//���ڵ������
    cJSON_AddItemToObject(root, "operation", cJSON_CreateString("CALL"));
    cJSON_AddItemToObject(root, "service", cJSON_CreateString("telephone"));
    cJSON_AddItemToObject(root, "text", cJSON_CreateString("��绰������"));
    cJSON_AddItemToObject(root, "semantic", item);//root�ڵ������semantic�ڵ�
    cJSON_AddItemToObject(item, "slots", next);//semantic�ڵ������item�ڵ�
    cJSON_AddItemToObject(next, "name", cJSON_CreateString("����"));//���name�ڵ�
 
		retchar= cJSON_Print(root);
		printf("strlen(retchar):%d\r\n",strlen(retchar));
    printf("%s\n", retchar);;
//		//cJSON_Delete(next);	
//		//cJSON_Delete(item);
		cJSON_Delete(root);
		vPortFree(retchar);
		printf("xPortGetFreeHeapSize():%d\r\n",xPortGetFreeHeapSize());
		
		return 0;
}

uint8_t jlink_Eulerangles(Info_1secTypeDef Info,char *retstr){
		
		char*tempstr=NULL;
		static float roll;
		cJSON * info_obj =  cJSON_CreateObject();
		cJSON * params_obj =  cJSON_CreateObject();
	
		cJSON_AddItemToObject(info_obj, "method", cJSON_CreateString("thing.event.property.post"));
    cJSON_AddItemToObject(info_obj, "id", cJSON_CreateNumber(4));//���ڵ������
		cJSON_AddItemToObject(info_obj, "params", params_obj);//root�ڵ������semantic�ڵ�
		cJSON_AddItemToObject(params_obj, "pitch", cJSON_CreateNumber(Info.Eulerangles.pitch));//root�ڵ������semantic�ڵ�
		cJSON_AddItemToObject(params_obj, "roll", cJSON_CreateNumber(roll));//root�ڵ������semantic�ڵ�
		cJSON_AddItemToObject(params_obj, "yaw", cJSON_CreateNumber(Info.Eulerangles.yaw));//root�ڵ������semantic�ڵ�
		
		cJSON_AddItemToObject(info_obj,	"version",cJSON_CreateString("1.0"));
	
		roll+=0.001;
		tempstr= cJSON_Print(info_obj);
		strcpy(retstr,tempstr);

	
//		printf("xPortGetFreeHeapSize():%d\r\n",xPortGetFreeHeapSize());
		cJSON_Delete(info_obj);	
		vPortFree(tempstr);
//		printf("xPortGetFreeHeapSize():%d\r\n",xPortGetFreeHeapSize());
		return 0;
}


