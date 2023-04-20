 #ifndef _CHASSIS_TASK_H_
#define _CHASSIS_TASK_H_

#include "main.h"
#include "DJI_Remote_Control.h"
#include "pid.h"
#include "Judge_Data.h"
#include "Judge_Graphic.h"
#include "App_Set.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

#define Chassis_Motor_Speed_Kp 2400
#define Chassis_Motor_Speed_Ki 0
#define Chassis_Motor_Speed_Kd 0
#define Chassis_Motor_Speed_Maxout 10000
#define Chassis_Motor_Speed_IMaxout 0

#define Chassis_Motor_Turn_Kp 9
#define Chassis_Motor_Turn_Ki 0
#define Chassis_Motor_Turn_Kd 7
#define Chassis_Motor_Turn_Maxout 4
#define Chassis_Motor_Turn_IMaxout 0

//��������ٶ�����
#define Chassis_Max_Speed_Sett 6

#define Chassis_3508_Data_Change_To_Speed 10

#define Chassis_Max_Heat_Normal 60
#define Chassis_Max_Heat_Feipo 250

#define CHASSIS_TURN_SPEED_UP_KEY KEY_PRESSED_OFFSET_SHIFT

#define CHASSIS_W_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_A_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_S_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_D_KEY KEY_PRESSED_OFFSET_D

#define CHASSIS_TURN_LEFT_KEY KEY_PRESSED_OFFSET_E
#define CHASSIS_TURN_RIGHT_KEY KEY_PRESSED_OFFSET_Q
#define CHASSIS_TURN_Variable_Speed_Spin_KEY KEY_PRESSED_OFFSET_G

#define BUFFER_TOTAL_CURRENT_LIMIT_FACTOR 320
#define POWER_TOTAL_CURRENT_LIMIT_FACTOR  500

#define Chassis_Max_Speed 4
#define Spin_Speed 5
#define Chassis_Shift_Max_Speed 10                                                                                                   
#define Chassis_RC_Max_Speed 10

#define Chassis_Follow_Gimbal_Set 6



/*********�������*********/
typedef struct
{
	//�������
	Motor_Msg_t* Chassis_Motor_Msg_Get;
	//����ת����ĵ��ת��
	float Chassis_Speed_Get;
}Chassos_Motor_Msg_t;

/*********�������ݶ�ȡ����*********/
typedef struct
{
	//����ģʽ
	Chassis_Mode_t Chassis_Mode;
	
	//��������Ϣ
	Chassos_Motor_Msg_t Chassis_Motor_Msg[4];
	
	//����ϵͳ������ݻ�ȡ
	Judge_Info_t* Chassis_Judge_Msg_Get;
	
	//���̵�����pid����
	PID Chassis_Motor_Pid[4];
	PID Chassis_Motor_Turn_Pid;
	
	//������������ݻ�ȡ
	const float* Chassis_IMU_Angle;
	const float* Chassis_IMU_Aspeed;
	//ң��������ֵ��ȡ
	const RC_Ctl_t* Chassis_RC_Ctl_Data;	
	
	//�����ٶ�����
	float Chassis_Control_Speed_Set[4];
	float Chassis_X_Speed_Set;
	float Chassis_Y_Speed_Set;
	float Chassis_LR_Speed_Set;
	//���̸�����̨-��̨��е�ǶȲ�
	float* Chassis_Follow_Gimbal_Angle_TM;
	//��������ٶ�
	float Chassis_Speed_Max;
	//���̵���ٶ�����
	float Chassis_Motor_Speed_Set[4];
	
	
	//���̷��͵���ֵ
	float Chassis_Motor_Curent_Send[4];
	
	uint8_t Robot_ID;
	float Chassis_Power;	//���̹�������
	uint16_t Chassis_Power_Buffer;		//������������
	uint16_t Chassis_Max_Power;	//��ǰ���������
	
	Judge_Graphic_Obj_t* Spin_Graphic;
}Chassis_t;

void Chassis_Task(void *pvParameters);
void oled_Show_Location(int x,int y);

Chassis_Mode_t* Return_Chassis_Mode_Add(void);

#endif

