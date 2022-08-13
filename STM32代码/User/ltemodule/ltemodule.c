#include "ltemodule.h"
#include <string.h>
#include "cJSON.h"
#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
 /**
  * @brief  配置嵌套向量中断控制器NVIC
  * @param  无
  * @retval 无
  */
volatile u8 ltemodule_USART_state=0;//0x01接收到第一个0d，0x02接收到第一个0x0a，0x04接收到第二个0x0d,0x08接收到第二个0a
char ltemodule_USART_buf[50];
char ltemodule_USART_num=0;
uint64_t uinx_timestamp;

static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* 嵌套向量中断控制器组选择 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  
  /* 配置USART为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = ltemodule_USART_IRQ;
  /* 抢断优先级*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  /* 子优先级 */
 // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
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
void ltemodule_USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// 打开串口GPIO的时钟
	ltemodule_USART_GPIO_APBxClkCmd(ltemodule_USART_GPIO_CLK, ENABLE);
	
	// 打开串口外设的时钟
	ltemodule_USART_APBxClkCmd(ltemodule_USART_CLK, ENABLE);

	// 将USART Tx的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = ltemodule_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ltemodule_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  // 将USART Rx的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = ltemodule_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(ltemodule_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	
	// 配置串口的工作参数
	// 配置波特率
	USART_InitStructure.USART_BaudRate = ltemodule_USART_BAUDRATE;
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
	USART_Init(ltemodule_USARTx, &USART_InitStructure);
	
	// 串口中断优先级配置
	NVIC_Configuration();
	
	// 使能串口接收中断
	//USART_ITConfig(ltemodule_USARTx, USART_IT_RXNE, ENABLE);	
	
	// 使能串口
	USART_Cmd(ltemodule_USARTx, ENABLE);	    
}

void ltemodule_USART_IRQHandler(void){

	char res=0;
	if(USART_GetITStatus(ltemodule_USARTx, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		res = USART_ReceiveData(ltemodule_USARTx);	//读取接收到的数据	
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


/*****************  发送一个字节 **********************/
void ltemodule_Usart_SendByte(  uint8_t ch)
{
	/* 发送一个字节数据到USART */
	USART_SendData(ltemodule_USARTx,ch);
		
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(ltemodule_USARTx, USART_FLAG_TXE) == RESET);	
}


/*****************  发送字符串 **********************/
void ltemodule_Usart_SendString(char *str)
{
	unsigned int k=0;
  do 
  {
      ltemodule_Usart_SendByte( *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* 等待发送完成 */
  while(USART_GetFlagStatus(ltemodule_USARTx,USART_FLAG_TC)==RESET)
  {}
}

/*****************  发送一个16位数 **********************/
void ltemodule_Usart_SendHalfWord(uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* 取出高八位 */
	temp_h = (ch&0XFF00)>>8;
	/* 取出低八位 */
	temp_l = ch&0XFF;
	
	/* 发送高八位 */
	USART_SendData(ltemodule_USARTx,temp_h);	
	while (USART_GetFlagStatus(ltemodule_USARTx, USART_FLAG_TXE) == RESET);
	
	/* 发送低八位 */
	USART_SendData(ltemodule_USARTx,temp_l);	
	while (USART_GetFlagStatus(ltemodule_USARTx, USART_FLAG_TXE) == RESET);	
}

/*
等待模块发来AT Ready
发送+++到4g模块，让其进入AT指令模式，收到OK后，进入指令模式成功
 
*/
void ltemodule_MQTT_init(void){

	ltemodule_USART_Config();
	// 使能串口接收中断
	USART_ITConfig(ltemodule_USARTx, USART_IT_RXNE, ENABLE);	
	printf("4g等待启动\r\n");
	while(ltemodule_USART_state!=4){
	}
	ltemodule_USART_buf[ltemodule_USART_num]=0;
	if(strstr(ltemodule_USART_buf, "Ready")==NULL){
		printf("4g模块启动失败\r\n");
	}else{
		printf("4g模块启动成功\r\n");
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
		printf("4g模块进入指令配置失败\r\n");
	}
	memset(ltemodule_USART_buf,0,50);
	ltemodule_USART_num=0;
	ltemodule_USART_state=0;
	
	while(ltemodule_USART_state!=4);
	if(strstr(ltemodule_USART_buf, "MQTT CONNECTED")==NULL){
		printf("4g模块连接失败\r\n");
	}
	
	memset(ltemodule_USART_buf,0,50);
	ltemodule_USART_num=0;
	ltemodule_USART_state=0;
	
	printf("MQTT成功连接MQTT\r\n");
	
	ltemodule_Usart_SendString("AT+TIMESTAMP");
	while(ltemodule_USART_state!=4);
	if(strstr(ltemodule_USART_buf, "OK")==NULL){
		printf("获得时间戳失败\r\n");
	}
	uinx_timestamp=get_timestamp(ltemodule_USART_buf);
	printf("获得时间戳%lld\r\n",uinx_timestamp);
	memset(ltemodule_USART_buf,0,50);
	ltemodule_USART_num=0;
	ltemodule_USART_state=0;


	ltemodule_Usart_SendString("ATO");
	while(ltemodule_USART_state!=4);
	if(strstr(ltemodule_USART_buf, "OK")==NULL){
		printf("开始传输\r\n");
	}
	memset(ltemodule_USART_buf,0,50);
	ltemodule_USART_num=0;
	ltemodule_USART_state=0;
	
		// 使能串口接收中断
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
    cJSON_AddItemToObject(info_1s_obj, "id", cJSON_CreateNumber(4));//根节点下添加
		cJSON_AddItemToObject(info_1s_obj, "params", params_obj);//root节点下添加semantic节点
		cJSON_AddItemToObject(params_obj, "VehSpeed", VehSpeed_obj);//root节点下添加semantic节点
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
 
    cJSON_AddItemToObject(root, "rc", cJSON_CreateNumber(0));//根节点下添加
    cJSON_AddItemToObject(root, "operation", cJSON_CreateString("CALL"));
    cJSON_AddItemToObject(root, "service", cJSON_CreateString("telephone"));
    cJSON_AddItemToObject(root, "text", cJSON_CreateString("打电话给张三"));
    cJSON_AddItemToObject(root, "semantic", item);//root节点下添加semantic节点
    cJSON_AddItemToObject(item, "slots", next);//semantic节点下添加item节点
    cJSON_AddItemToObject(next, "name", cJSON_CreateString("张三"));//添加name节点
 
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
    cJSON_AddItemToObject(info_obj, "id", cJSON_CreateNumber(4));//根节点下添加
		cJSON_AddItemToObject(info_obj, "params", params_obj);//root节点下添加semantic节点
		cJSON_AddItemToObject(params_obj, "pitch", cJSON_CreateNumber(Info.Eulerangles.pitch));//root节点下添加semantic节点
		cJSON_AddItemToObject(params_obj, "roll", cJSON_CreateNumber(roll));//root节点下添加semantic节点
		cJSON_AddItemToObject(params_obj, "yaw", cJSON_CreateNumber(Info.Eulerangles.yaw));//root节点下添加semantic节点
		
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


