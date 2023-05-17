#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "main.h"
#include "DJI_Remote_Control.h"
#include "pid.h" 

#include "App_Set.h"

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

#define Motionless_Time 1500

#define Gimbal_Vision_Key KEY_PRESSED_OFFSET_CTRL

#define Gimbal_Turn_Head_Key KEY_PRESSED_OFFSET_F
#define Gimbal_Turn_Head_Speed 0.006f

#define Gimbal_Spin_Key (KEY_PRESSED_OFFSET_Q | KEY_PRESSED_OFFSET_E)

#define Pitch_Submit_Limit  (-14.000f)
#define Pitch_Rise_Limit    29.500f
#define Pitch_Middle_Limt   (-1.175f)
#define Pitch_Deviation_Max  15.000f
 
typedef struct
{
	Motor_Msg_t* Shoot_Motor_Msg_Get;
	int Motoe_Now_Angle;
	int Motoe_Now_Speed;
	int Motor_Now_Dianliu;
	int Motor_Last_Speed;	
	int Motor_Last_Angle;
}Gimbal_Motor_Msg_t;


typedef struct
{
	//云台电机相关数据
	Motor_Msg_t* Gimbal_Motor_Msg_Get;	
	//陀螺仪角度获取地址
	const float* Gimbal_IMU_Angle_Data;
	//陀螺仪角速度获取地址
	const float* Gimbal_IMU_Aspeed_Data;
	//云台机械角度值
	float Gimbal_Motor_Angle_Msg;
	//云台陀螺仪角度值
	float Gimbal_IMU_Angle_Msg;
	//云台角度中点信息
	float Gimbal_Angle_Middle_Msg;	
	//云台绝对角度-机械角度差
	float Gimbal_Motor_Angle_TM;	
	//云台绝对角度-陀螺仪角度差
	float Gimbal_IMU_Angle_Set;	
	
	float Gimbal_Max_Angle;
	float Gimbal_Min_Angle;
	

	//云台角速度-机械角速度值
	float Gimbal_Motor_Aspeed;	
	//云台角度苏-陀螺仪角速度值
	float Gimbal_IMU_Aspeed;	

}Gimbal_Msg_t;


typedef struct
{
	//云台电机相关数据
	Motor_Msg_t* Gimbal_Motor_Msg_Get;
  //经过转换后的电机转速
	float Gimbal_Pitch_Speed_Get;
	//转速累计
	float Motor_Run_Long;
	//转速转化后的距离
	float Motor_Run_Dis_Get_turn_by_speed;
	//机械角度转化后的距离
	float Motor_Run_Dis_Get_turn_by_angle;
	  
	//陀螺仪角度获取地址

	const float* Gimbal_IMU_Angle_Data;
	//陀螺仪角速度获取地址
	const float* Gimbal_IMU_Aspeed_Data;
	//云台机械角度值
	float Gimbal_Motor_Angle_Msg;
	//云台陀螺仪角度值
	float Gimbal_IMU_Angle_Msg;
	//云台角度中点信息
	float Gimbal_Angle_Middle_Msg;	
	//云台绝对角度-机械角度差
	float Gimbal_Motor_Angle_TM;	
	//云台绝对角度-陀螺仪角度差
	float Gimbal_IMU_Angle_Set;	
	
	float Gimbal_Control_Pitch_IMU;   //通过外置陀螺仪获取的 Pitch角度
	float Gimbal_Middle_Angle;
	float Gimbal_Max_Angle;
	float Gimbal_Min_Angle;
	int Angle_Init_flag ;
	

	//云台角速度-机械角速度值
	float Gimbal_Motor_Aspeed;	
	//云台角度苏-陀螺仪角速度值
	float Gimbal_IMU_Aspeed;	

}Gimbal_Msg_t_Pitch;


typedef struct
{
	//云台模式
	Gimbal_Mode_t Gimbal_Mode;
	
/**********云台相关PID结构体**********/
	//YAW云台暂停pid
	PID Gimbal_Yaw_Stop_Angle_Pid;
	//云台绝对角度计算PID-机械角度
	PID Gimbal_Yaw_Motor_Angle_Pid;
	PID Gimbal_Pitch_Motor_Angle_Pid;
	//云台绝对角度计算PID-陀螺仪角度  	
	PID Gimbal_Yaw_IMU_Angle_Pid;
	PID Gimbal_Pitch_IMU_Angle_Pid;
	//云台视觉计算PID-陀螺仪角度
	PID Gimbal_Yaw_Vision_Angle_Pid;
	PID Gimbal_Pitch_Vision_Angle_Pid;	
	//云台角速度计算PID
	PID Gimbal_Yaw_Speed_Pid;	
	PID Gimbal_Pitch_Speed_Pid;
	//云台计算PID
	PID Gimbal_Yaw_Motor_Pid;	
	PID Gimbal_Pitch_Motor_Pid;	
	//3508 + 丝杆 Pitch轴云台计算PID
	PID Gimbal_Pitch_3508_Distance_Pid;
	PID Gimbal_Pitch_3508_Speed_Pid;	
	
/**********云台相关信息结构体**********/
	Gimbal_Msg_t Gimbal_Yaw_Msg_t;
	Gimbal_Msg_t Gimbal_Pitch_Msg_t;
	
	//陀螺仪相关数据获取
	const float* Gimbal_IMU_Angle;
	const float* Gimbal_IMU_Aspeed;
	//遥控器控制值获取
	const RC_Ctl_t* Gimbal_RC_Ctl_Data;	
	
	float Gimbal_Yaw_RC_Data;
	float Gimbal_Pitch_RC_Data;
	
	float Gimbal_Yaw_Add_Data;
	float Gimbal_Pitch_Add_Data;
	
	float Gimbal_Pitch_Rise_Add_Data;
	float Gimbal_Pitch_Submit_Add_Data;
		
	//云台控制值获取
	float Gimbal_Yaw_Control_Data;
	float Gimbal_Pitch_Control_Data;	
	//云台控制角度差-电机模式
	float Gimbal_Control_Motor_Angle_TM[2];	
	//云台控制角度-IMU模式
	float Gimbal_Control_IMU_Angle[2];	
	
	//云台电机角度PID输出-角速度PID输入
	float Gimbal_Yaw_Apid_Out;
	float Gimbal_Pitch_Apid_Out;
		
	//云台发送电流值
	float Gimbal_Motor_Current_Send[2];

}Gimbal_t;


void Gimbal_Task(void *pvParameters);
float* Gimbal_Yaw_Angle_To_Chassis(void);
float Pitch_Angle_Data_Add_Limit(float* Pitch_IMU_Angle,float Max_IMU_Angle,float Min_IMU_Angle);
float Pitch_Angle_Limit(float Pitch_IMU_Angle,float Max_IMU_Angle,float Min_IMU_Angle);

Gimbal_Mode_t* Return_Gimbal_Mode_Add(void);

#endif 
