#ifndef CAN_H
#define CAN_H
#include "stm32f10x.h"
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "cJSON.h"

#define  	Formulatest  	0

#define		ServiceID01		(0x01)
#define		ServiceID02		(0x02)
#define		ServiceID03		(0x03)

																		//ServiceID01
#define     PID01					(0x01)          //故障码个数
#define		PID04					(0x04)			//发动机负载  /
#define		PID05					(0x05)			//冷却液温度		/
#define		PID06					(0x06)			//短期燃油修正
#define		PID07					(0x07)			//长期燃油修正
#define		PID10					(0x10)			//空气流量			/
#define		PID11					(0x11)			//绝对节气门位置/
#define		PID0A					(0x0A)			//进气歧管绝对压力
#define		PID0B					(0x0B)			//进气歧管绝对压力
#define		PID0C					(0x0c)			//发动机转速		/
#define		PID0D					(0x0d)			//车速			/
#define		PID0F					(0x0f)			//进气歧管空气温度
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
//typedef struct {
//	uint8_t IDE;
//	uint32_t StdId;
//	uint32_t ExtId;
//	uint8_t SID;
//	uint8_t PID;
//	char *dataname;
//	void (*Formula)(CanRxMsg tempCANRxMSG, void *ret,cJSON * root);
//}OBDdataType;


typedef struct {
	uint8_t IDE;
	uint32_t StdId;
	uint32_t ExtId;
	uint8_t SID;
	uint8_t PID;
	uint8_t datanum;
	char *dataname;
	void (*Formula)(void * OBDdata,CanRxMsg tempCANRxMSG,cJSON * root);
}OBDdataTypeNew;

extern OBDdataTypeNew newOBDdata[25];
extern char OBDdataHz_new[25];
int sum_OBDdataHz_new(char *arry,char size);

//extern OBDdataType OBDdata[20];
//extern char OBDdataHz_1[20];

extern CanRxMsg CANRXMSG_1;
extern CanRxMsg CANRXMSG_2;
extern CanRxMsg CANRXMSG_3;
extern CanRxMsg CANRXMSG_4;
extern CanRxMsg CANRXMSG_5;
extern CanRxMsg CANRXMSG_6;
extern CanRxMsg CANRXMSG_7;
extern CanRxMsg CANRXMSG_8;
extern uint8_t volatile MESTRUCT_flag_5;
extern uint8_t volatile MESTRUCT_flag_6;
extern uint8_t volatile MESTRUCT_flag_7;
extern uint8_t volatile MESTRUCT_flag_8;
extern uint8_t volatile MESTRUCT_flag_9;
extern uint8_t volatile MESTRUCT_flag_10;
extern uint8_t volatile MESTRUCT_flag_11;
extern uint8_t volatile MESTRUCT_flag_12;

extern uint8_t  numofask;

static void CAN1_NVICConfig(void);
void CAN1_Config(uint32_t Baud);
void CANinit(uint32_t Baud);
void OBDTransmit_askframe(uint32_t StdId,  uint8_t IDE, uint8_t SID,uint8_t PID);
void CANTransmit_frame(uint32_t Id,  uint8_t IDE, uint8_t *data,uint8_t datalen);


void Speed_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root);
void RPM_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root);
void Engineload_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root);
void  Coolant_temperature_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root);
void SHRTFT1_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root);
void LONGFT1_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root);
void  Airflow_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root);
void  Intake_abspressure(CanRxMsg tempCANRxMSG,void *ret,cJSON * root);
void  Throttle_position(CanRxMsg tempCANRxMSG,void *ret,cJSON * root);
void  Intake_airtmep(CanRxMsg tempCANRxMSG,void *ret,cJSON * root);
void  SPARKADV_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root);
void  RUNTM_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root);
void  FRP_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root);
void  FLI_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root);
void  APP_D_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root);
void  APP_R_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root);
void  EOT_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root);
void  FUEL_TIMING_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root);
void  FUEL_RATE_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root);
void  NOx_Formula(CanRxMsg tempCANRxMSG,void *ret,cJSON * root);

void AddOBDdata_to_json(char *retarr,uint8_t VSS,uint32_t RPM,float TP,float LOAD_PCT,uint8_t MAP,float MAF,uint64_t timestamp);

#endif

