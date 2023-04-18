#ifndef __CAN_H__
#define __CAN_H__

#include "main.h"

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

#define Firc_Motor_Left 0x201
#define Firc_Motor_Right 0x202
#define Firc_Motor_UP 0x203

#define Trigger_Motor_Can 0x207

#define Gimbal_Yaw_Can 0x205
#define Gimbal_Pitch_Can 0x206

#define Chassis_Motor_RI 0x201
#define Chassis_Motor_LI 0x202
#define Chassis_Motor_LB 0x203
#define Chassis_Motor_RB 0x204


typedef enum
{
	CAN1_RX = 0,
	CAN2_RX = 1
}CAN_Set_t;

#define M2006_Speed_Change 0.0004629630

//电容相关信息
typedef struct 
{
	int16_t Cap_I;//电容电流
	int16_t Cap_V;//电容电压
	uint16_t Cap_State;//电容状态
	uint32_t Timestamp;
}Super_C_Msg_t;;

typedef struct 
{
	int16_t angle;//机械角度
	int16_t speed;//速度
	int16_t current;
	int16_t temp;
	int16_t Last_Angle;
	int Angle_Long;
}Motor_Msg_t;

void CAN1_Init(void);
void CAN2_Init(void);

void CAN1_Motor_Control(int16_t stdid,uint16_t num1,uint16_t num2,uint16_t num3,uint16_t num4);
void CAN2_Motor_Control(int16_t stdid,uint16_t num1,uint16_t num2,uint16_t num3,uint16_t num4);
void CAN2_SuperCap_Control(int16_t Stdid,uint16_t num1,uint16_t num2,uint16_t num3,uint16_t num4);

void Can1ReceiveMsgProcess(CanRxMsg *can_receive_message);
void Can2ReceiveMsgProcess(CanRxMsg *can_receive_message);

void Angle_Count_Long(Motor_Msg_t *ms);

Motor_Msg_t *Get_DJI_Motor_Data(CAN_Set_t CAN_Set,uint16_t Motor_Stdid);
Super_C_Msg_t *Get_Cap_Data(void);

#endif 
