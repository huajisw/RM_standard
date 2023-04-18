#ifndef __MODE_SET_H__
#define __MODE_SET_H__

#include "main.h"
#include "DJI_Remote_Control.h"
#include "pid.h" 

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

#define Chassis_First_Mode 
#define Chassis_First_Mode 
#define Chassis_First_Mode 

#define Chassis_First_Mode 
#define Chassis_First_Mode 
#define Chassis_First_Mode 

typedef enum
{
	Chassis_Zero = 0,//无控制，传送0
	Chassis_Motionless,//无控制，用电机角度保持当前位置
	Chassis_RC_Control,//遥控器控制
	Chassis_Follow_Gimbal,//底盘跟随云台
	Chassis_Follow_Chassis,
	Chassis_Spin//小陀螺模式
}Chassis_Mode_t;



#endif