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
	//��̨����������
	Motor_Msg_t* Gimbal_Motor_Msg_Get;	
	//�����ǽǶȻ�ȡ��ַ
	const float* Gimbal_IMU_Angle_Data;
	//�����ǽ��ٶȻ�ȡ��ַ
	const float* Gimbal_IMU_Aspeed_Data;
	//��̨��е�Ƕ�ֵ
	float Gimbal_Motor_Angle_Msg;
	//��̨�����ǽǶ�ֵ
	float Gimbal_IMU_Angle_Msg;
	//��̨�Ƕ��е���Ϣ
	float Gimbal_Angle_Middle_Msg;	
	//��̨���ԽǶ�-��е�ǶȲ�
	float Gimbal_Motor_Angle_TM;	
	//��̨���ԽǶ�-�����ǽǶȲ�
	float Gimbal_IMU_Angle_Set;	
	
	float Gimbal_Max_Angle;
	float Gimbal_Min_Angle;
	

	//��̨���ٶ�-��е���ٶ�ֵ
	float Gimbal_Motor_Aspeed;	
	//��̨�Ƕ���-�����ǽ��ٶ�ֵ
	float Gimbal_IMU_Aspeed;	

}Gimbal_Msg_t;


typedef struct
{
	//��̨����������
	Motor_Msg_t* Gimbal_Motor_Msg_Get;
  //����ת����ĵ��ת��
	float Gimbal_Pitch_Speed_Get;
	//ת���ۼ�
	float Motor_Run_Long;
	//ת��ת����ľ���
	float Motor_Run_Dis_Get_turn_by_speed;
	//��е�Ƕ�ת����ľ���
	float Motor_Run_Dis_Get_turn_by_angle;
	  
	//�����ǽǶȻ�ȡ��ַ

	const float* Gimbal_IMU_Angle_Data;
	//�����ǽ��ٶȻ�ȡ��ַ
	const float* Gimbal_IMU_Aspeed_Data;
	//��̨��е�Ƕ�ֵ
	float Gimbal_Motor_Angle_Msg;
	//��̨�����ǽǶ�ֵ
	float Gimbal_IMU_Angle_Msg;
	//��̨�Ƕ��е���Ϣ
	float Gimbal_Angle_Middle_Msg;	
	//��̨���ԽǶ�-��е�ǶȲ�
	float Gimbal_Motor_Angle_TM;	
	//��̨���ԽǶ�-�����ǽǶȲ�
	float Gimbal_IMU_Angle_Set;	
	
	float Gimbal_Control_Pitch_IMU;   //ͨ�����������ǻ�ȡ�� Pitch�Ƕ�
	float Gimbal_Middle_Angle;
	float Gimbal_Max_Angle;
	float Gimbal_Min_Angle;
	int Angle_Init_flag ;
	

	//��̨���ٶ�-��е���ٶ�ֵ
	float Gimbal_Motor_Aspeed;	
	//��̨�Ƕ���-�����ǽ��ٶ�ֵ
	float Gimbal_IMU_Aspeed;	

}Gimbal_Msg_t_Pitch;


typedef struct
{
	//��̨ģʽ
	Gimbal_Mode_t Gimbal_Mode;
	
/**********��̨���PID�ṹ��**********/
	//YAW��̨��ͣpid
	PID Gimbal_Yaw_Stop_Angle_Pid;
	//��̨���ԽǶȼ���PID-��е�Ƕ�
	PID Gimbal_Yaw_Motor_Angle_Pid;
	PID Gimbal_Pitch_Motor_Angle_Pid;
	//��̨���ԽǶȼ���PID-�����ǽǶ�  	
	PID Gimbal_Yaw_IMU_Angle_Pid;
	PID Gimbal_Pitch_IMU_Angle_Pid;
	//��̨�Ӿ�����PID-�����ǽǶ�
	PID Gimbal_Yaw_Vision_Angle_Pid;
	PID Gimbal_Pitch_Vision_Angle_Pid;	
	//��̨���ٶȼ���PID
	PID Gimbal_Yaw_Speed_Pid;	
	PID Gimbal_Pitch_Speed_Pid;
	//��̨����PID
	PID Gimbal_Yaw_Motor_Pid;	
	PID Gimbal_Pitch_Motor_Pid;	
	//3508 + ˿�� Pitch����̨����PID
	PID Gimbal_Pitch_3508_Distance_Pid;
	PID Gimbal_Pitch_3508_Speed_Pid;	
	
/**********��̨�����Ϣ�ṹ��**********/
	Gimbal_Msg_t Gimbal_Yaw_Msg_t;
	Gimbal_Msg_t Gimbal_Pitch_Msg_t;
	
	//������������ݻ�ȡ
	const float* Gimbal_IMU_Angle;
	const float* Gimbal_IMU_Aspeed;
	//ң��������ֵ��ȡ
	const RC_Ctl_t* Gimbal_RC_Ctl_Data;	
	
	float Gimbal_Yaw_RC_Data;
	float Gimbal_Pitch_RC_Data;
	
	float Gimbal_Yaw_Add_Data;
	float Gimbal_Pitch_Add_Data;
	
	float Gimbal_Pitch_Rise_Add_Data;
	float Gimbal_Pitch_Submit_Add_Data;
		
	//��̨����ֵ��ȡ
	float Gimbal_Yaw_Control_Data;
	float Gimbal_Pitch_Control_Data;	
	//��̨���ƽǶȲ�-���ģʽ
	float Gimbal_Control_Motor_Angle_TM[2];	
	//��̨���ƽǶ�-IMUģʽ
	float Gimbal_Control_IMU_Angle[2];	
	
	//��̨����Ƕ�PID���-���ٶ�PID����
	float Gimbal_Yaw_Apid_Out;
	float Gimbal_Pitch_Apid_Out;
		
	//��̨���͵���ֵ
	float Gimbal_Motor_Current_Send[2];

}Gimbal_t;


void Gimbal_Task(void *pvParameters);
float* Gimbal_Yaw_Angle_To_Chassis(void);
float Pitch_Angle_Data_Add_Limit(float* Pitch_IMU_Angle,float Max_IMU_Angle,float Min_IMU_Angle);
float Pitch_Angle_Limit(float Pitch_IMU_Angle,float Max_IMU_Angle,float Min_IMU_Angle);

Gimbal_Mode_t* Return_Gimbal_Mode_Add(void);

#endif 
