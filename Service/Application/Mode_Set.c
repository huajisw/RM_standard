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
	Chassis_Zero = 0,//�޿��ƣ�����0
	Chassis_Motionless,//�޿��ƣ��õ���Ƕȱ��ֵ�ǰλ��
	Chassis_RC_Control,//ң��������
	Chassis_Follow_Gimbal,//���̸�����̨
	Chassis_Follow_Chassis,
	Chassis_Spin//С����ģʽ
}Chassis_Mode_t;



#endif