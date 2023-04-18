/**
  ****************************(C) COPYRIGHT 2021 HRBUST_AIR****************************
  * @file    App_Set.h
	* @brief   ��������ҳ��           
  * @author  JackyJuu , HRBUST_AIR_TEAM , website:www.airclub.tech
	* @Note 	 ���и��ֹ��ܵ����ã�����ִ�����Ĺ���״̬ģʽ�ȵ�
	* @Question 	 ��Ҫ���������Ҫ�޸ĵ�ִ��ѡ�4.9��
  * @version V1.0.0
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     4-9-2021     	 JackyJuu            Done
  ****************************(C) COPYRIGHT 2021 HRBUST_AIR****************************
	* @describe ��������ҳ��
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
	Chassis_Zero = 0,//�޿��ƣ�����0
	Chassis_Motionless,//�޿��ƣ��õ���Ƕȱ��ֵ�ǰλ��
	Chassis_RC_Control,//ң��������
	Chassis_Follow_Gimbal,//���̸�����̨
	Chassis_Follow_Chassis,//�Ҹ����Լ�
	Chassis_Spin_Left,//С����ģʽ
	Chassis_Spin_Right,//С����ģʽ
	Chassis_Spin,//С����ģʽ
	Chassis_Variable_Speed_Spin //����С����ģʽ
}Chassis_Mode_t;

typedef enum
{
	Gimbal_Zero = 0,//�޿��ƣ�����0
	Gimbal_Motionless,//�޿��ƣ��õ���Ƕȱ��ֵ�ǰλ��
	Gimbal_Motor_Control,//����Ƕȿ���
	Gimbal_IMU_Control,//�����ǽǶȿ���
	Gimbal_Vision_Control,//�Ӿ����ƽӿ�
	Gimbal_Follow_Chassis,//��̨������̣�yaw��̶�����
	Gimbal_Spin//С����ģʽ
}Gimbal_Mode_t;

//�����˵�ǰģʽ
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

