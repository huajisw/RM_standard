/**
  ****************************(C) COPYRIGHT 2021 HRBUST_AIR****************************
  * @file    App_Set.h
	* @brief   功能设置页面           
  * @author  JackyJuu , HRBUST_AIR_TEAM , website:www.airclub.tech
	* @Note 	 进行各种功能的设置，包括执行器的功能状态模式等等
	* @Question 	 需要补充更多需要修改的执行选项【4.9】
  * @version V1.0.0
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     4-9-2021     	 JackyJuu            Done
  ****************************(C) COPYRIGHT 2021 HRBUST_AIR****************************
	* @describe 功能设置页面
*/
#ifndef __APP_SET_H__
#define __APP_SET_H__

#include "main.h"
#include "DJI_Remote_Control.h"
#include "pid.h" 

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

#define Motionless_Time_Set 3000 

#define Shoot_Speed

typedef enum
{
	Chassis_Zero = 0,//无控制，传送0
	Chassis_Motionless,//无控制，用电机角度保持当前位置
	Chassis_RC_Control,//遥控器控制
	Chassis_Follow_Gimbal,//底盘跟随云台
	Chassis_Follow_Chassis,//我跟我自己
	Chassis_Spin_Left,//小陀螺模式
	Chassis_Spin_Right,//小陀螺模式
	Chassis_Spin,//小陀螺模式
	Chassis_Variable_Speed_Spin //变速小陀螺模式
}Chassis_Mode_t;

typedef enum
{
	Gimbal_Zero = 0,//无控制，传送0
	Gimbal_Motionless,//无控制，用电机角度保持当前位置
	Gimbal_Motor_Control,//电机角度控制
	Gimbal_IMU_Control,//陀螺仪角度控制
	Gimbal_Vision_Control,//视觉控制接口
	Gimbal_Follow_Chassis,//云台跟随底盘，yaw轴固定不动
	Gimbal_Spin//小陀螺模式
}Gimbal_Mode_t;

//机器人当前模式
typedef enum
{
	Robot_Zero_Move = 0,
	Robot_Move_Now = 1
}Standard_Robot_Mode_t;

//typedef struct 
//{
//	Standard_Robot_Mode_t Standard_Robot_Mode;
//}Robot_t;



#define Chassis_First_Mode Chassis_Zero 
#define Chassis_Second_Mode Chassis_RC_Control
#define Chassis_Third_Mode Chassis_Follow_Gimbal

#define Gimbal_First_Mode Gimbal_Zero
#define Gimbal_Second_Mode Gimbal_Motor_Control
#define Gimbal_Third_Mode Gimbal_Spin



#endif

