/**
  ****************************(C) COPYRIGHT 2021 HRBUST_AIR****************************
  * @file    gimbal_task.c
	* @brief   ��̨��������           
  * @author  JackyJuu , HRBUST_AIR_TEAM , website:www.airclub.tech
	* @Note 	 ��������ģʽ����App_Set.h�н�������
	* @Question 	 ��̨PID����Ҫ���в��ԡ�4.9��
  * @version V1.0.0
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     4-9-2021     	 JackyJuu            Done
  ****************************(C) COPYRIGHT 2021 HRBUST_AIR****************************
	* @describe ��̨��������
*/

#include "gimbal_task.h"
#include "IMU_Task.h"

#include "arm_math.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "App_Set.h"

#include "can.h"
#include "pid.h"
#include "led.h"

#include "DJI_Remote_Control.h"

extern RC_Ctl_t RC_Ctl_Data;
#define Gimbal_Motor_Angle_Change 0.0439453125

Gimbal_t Gimbal;

float Gimbal_Motor_Yaw_Angle_Speed_Get;
float Gimbal_Motor_Pitch_Angle_Speed_Get;

extern Motor_Msg_t ALL_Motor_Msg[16];

float Gimbal_Motor_Yaw_Set_Now;
float Gimbal_Motor_Pitch_Set_Now;

float Gimbal_Motor_Yaw_Set_Add;
float Gimbal_Motor_Pitch_Set_Add;

//�е�����
#define Gimbal_Yaw_Middle_Angle  -3.14f
#define Gimbal_Pitch_Middle_Angle  0.30f

#define Yaw_Max_Angle 1.9f
#define Yaw_Min_Angle -1.9f

#define Pitch_Max_Angle 0.65f
#define Pitch_Min_Angle 0.05f

#define Pitch_IMU_Max_Angle 0
#define Pitch_IMU_Min_Angle 0

#define Pitch_Max_Angle_Distance_By_Speed 26.000
#define Pitch_Min_Angle_Distance_By_Speed (-11.000f)

#define Pitch_Max_Angle_Distance_By_Angle 38.000f
#define Pitch_Min_Angle_Distance_By_Angle (-16.000f)



//#define Gimbal_Yaw_Middle_Angle  0.48397091
//#define Gimbal_Pitch_Middle_Angle  0.48397091


float Gimbal_Yaw_Angle_Get[2], Gimbal_Pitch_Angle_Get[2];

static float Angle_Limit(float Angle_Set);

static float Angle_Abs(float Angle_Set_To_Abs);

static float Angle_TM_Set(float Now_Angle,float Angle_Middle);

static void Angle_Control_Limit(float *Angle_Set,float Max_Angle,float Min_Angle);


#define Gimbal_Pitch_3508_Distance_Kp 1
#define Gimbal_Pitch_3508_Distance_Ki 0
#define Gimbal_Pitch_3508_Distance_Kd 0
#define Gimbal_Pitch_3508_Distance_Maxout 10
#define Gimbal_Pitch_3508_Distance_IMaxout 0

#define Gimbal_Pitch_speed_3508_Angle_Kp 2400
#define Gimbal_Pitch_speed_3508_Angle_Ki 0
#define Gimbal_Pitch_speed_3508_Angle_Kd 0
#define Gimbal_Pitch_speed_3508_Angle_Maxout 5000
#define Gimbal_Pitch_speed_3508_Angle_IMaxout 0

#define Gimbal_Yaw_Vision_Angle_Kp 1000
#define Gimbal_Yaw_Vision_Angle_Ki 0
#define Gimbal_Yaw_Vision_Angle_Kd 0
#define Gimbal_Yaw_Vision_Angle_Maxout 10
#define Gimbal_Yaw_Vision_Angle_IMaxout 10

#define Gimbal_Yaw_Motor_Angle_Kp 15
#define Gimbal_Yaw_Motor_Angle_Ki 0
#define Gimbal_Yaw_Motor_Angle_Kd 500
#define Gimbal_Yaw_Motor_Angle_Maxout 10
#define Gimbal_Yaw_Motor_Angle_IMaxout 10

#define Gimbal_Yaw_IMU_Angle_Kp 2
#define Gimbal_Yaw_IMU_Angle_Ki 0
#define Gimbal_Yaw_IMU_Angle_Kd 600
#define Gimbal_Yaw_IMU_Angle_Maxout 9
#define Gimbal_Yaw_IMU_Angle_IMaxout 0

#define Gimbal_Yaw_Speed_Kp 7000
#define Gimbal_Yaw_Speed_Ki 0
#define Gimbal_Yaw_Speed_Kd 600
#define Gimbal_Yaw_Speed_Maxout 25000
#define Gimbal_Yaw_Speed_IMaxout 5000

#define Gimbal_Yaw_Motor_Kp 9000
#define Gimbal_Yaw_Motor_Ki 0
#define Gimbal_Yaw_Motor_Kd 0
#define Gimbal_Yaw_Motor_Maxout 25000
#define Gimbal_Yaw_Motor_IMaxout 5000

#define Gimbal_Pitch_Vision_Angle_Kp 800
#define Gimbal_Pitch_Vision_Angle_Ki 0
#define Gimbal_Pitch_Vision_Angle_Kd 0
#define Gimbal_Pitch_Vision_Angle_Maxout 10
#define Gimbal_Pitch_Vision_Angle_IMaxout 10

#define Gimbal_Pitch_Motor_Angle_Kp 15
#define Gimbal_Pitch_Motor_Angle_Ki 0
#define Gimbal_Pitch_Motor_Angle_Kd 300
#define Gimbal_Pitch_Motor_Angle_Maxout 10
#define Gimbal_Pitch_Motor_Angle_IMaxout 10

#define Gimbal_Pitch_IMU_Angle_Kp 10
#define Gimbal_Pitch_IMU_Angle_Ki 0
#define Gimbal_Pitch_IMU_Angle_Kd 10
#define Gimbal_Pitch_IMU_Angle_Maxout 10
#define Gimbal_Pitch_IMU_Angle_IMaxout 0

#define Gimbal_Pitch_Speed_Kp 5000
#define Gimbal_Pitch_Speed_Ki 0
#define Gimbal_Pitch_Speed_Kd 0
#define Gimbal_Pitch_Speed_Maxout 25000
#define Gimbal_Pitch_Speed_IMaxout 5000

#define Gimbal_Pitch_Motor_Kp 20000
#define Gimbal_Pitch_Motor_Ki 0
#define Gimbal_Pitch_Motor_Kd 0
#define Gimbal_Pitch_Motor_Maxout 25000
#define Gimbal_Pitch_Motor_IMaxout 5000


#define Yaw_RC_Sen 0.000072
#define Pitch_RC_Sen 0.000018



//PID����
float Gimbal_pid_calc(PID*pid, float now, float set)
	{
    pid->now = now;
    pid->set = set;

		pid->now_error = pid->set - pid->now;	//set - measure

		pid->now_error = Angle_Limit(pid->now_error);
//		if((pid->now_error < 0.01) && (pid->now_error > -0.01))
//			return 0;
    if(pid->pid_mode == 1) //λ�û�PID
    {
	      pid->pout = pid->kp * pid->now_error;
        pid->iout = pid->ki * pid->sum_of_error;
        pid->dout = pid->kd * (pid->now_error - pid->Last_error );
				pid->sum_of_error+=pid->now_error;	
				PID_limit(&(pid->sum_of_error), 20000);
				PID_limit(&(pid->iout), pid->IntegralLimit);
        pid->out = pid->pout + pid->iout + pid->dout;
        PID_limit(&(pid->out), pid->MaxOutput);
    }			
    else if(pid->pid_mode == 2)//����ʽPID
    {
        pid->pout = pid->kp * (pid->now_error - pid->Last_error);
        pid->iout = pid->ki * pid->now_error;
        pid->dout = pid->kd * (pid->now_error - 2*pid->Last_error + pid->Last_Last_error);        
				PID_limit(&(pid->iout), pid->IntegralLimit);
        pid->plus = pid->pout + pid->iout + pid->dout;
        pid->plus_out = pid->last_plus_out + pid->plus;
			  pid->out = pid->plus_out; 
				PID_limit(&(pid->out), pid->MaxOutput);
        pid->last_plus_out = pid->plus_out;	//update last time
    }
		
    pid->Last_Last_error= pid->Last_error;
    pid->Last_error = pid->now_error;
    return pid->out;
}
	


float* Gimbal_Yaw_Angle_To_Chassis(void)
{
	return &Gimbal.Gimbal_Yaw_Msg_t.Gimbal_Motor_Angle_TM;
}


	
//PID����(���ٻ���+����ȫ΢��)
float Gimbal_Math_pid_calc(PID*pid, float now, float set)
{
    pid->now = now;
    pid->set = set;

		pid->now_error = pid->set - pid->now;	//set - measure

		pid->now_error = Angle_Limit(pid->now_error);
	
		if((pid->now_error < pid->Small_Error_Limit) && (pid->now_error > -pid->Small_Error_Limit))
			return 0;
    if(pid->pid_mode == 1) //λ�û�PID
    {
	      pid->pout = pid->kp * pid->now_error;
        pid->iout = pid->ki * pid->sum_of_error;
        pid->dout = pid->kd * (pid->now_error - pid->Last_error );
				pid->sum_of_error+=pid->now_error;	
				PID_limit(&(pid->sum_of_error), 20000);
				PID_limit(&(pid->iout), pid->IntegralLimit);
        pid->out = pid->pout + pid->iout + pid->dout;
        PID_limit(&(pid->out), pid->MaxOutput);
    }	
		
    else if(pid->pid_mode == 2)//����ʽPID
    {
        pid->pout = pid->kp * (pid->now_error - pid->Last_error);
        pid->iout = pid->ki * pid->now_error;
        pid->dout = pid->kd * (pid->now_error - 2*pid->Last_error + pid->Last_Last_error);        
				PID_limit(&(pid->iout), pid->IntegralLimit);
        pid->plus = pid->pout + pid->iout + pid->dout;
        pid->plus_out = pid->last_plus_out + pid->plus;
			  pid->out = pid->plus_out; 
				PID_limit(&(pid->out), pid->MaxOutput);
        pid->last_plus_out = pid->plus_out;	//update last time
    }
		else if(pid->pid_mode == 3)//���ٻ���/����ȫ΢��
    {
	      pid->pout = pid->kp * pid->now_error;
				if(((pid->Set_Out_Mode == 1) && (pid->now_error > 0)) || ((pid->Set_Out_Mode == -1) && (pid->now_error < 0)));
				else
				{
					if(PID_Fabs(pid->now_error) <= pid->Set_B)
					{
						pid->sum_of_error += pid->now_error;
					}
					else if(PID_Fabs(pid->now_error) >= (pid->Set_B + pid->Set_A))
					{
						pid->sum_of_error = 0;
					}
					else
					{
						pid->Set_ratio = (pid->Set_A + pid->Set_B - PID_Fabs(pid->now_error)) / pid->Set_A;
						pid->sum_of_error += pid->Set_ratio * pid->now_error;
					}
				}
			//���ٻ���
      pid->iout = pid->ki * pid->sum_of_error;			
			//����ȫ΢��	
			pid->dout = pid->kd * (pid->now_error - pid->Last_error) * (1 - pid->Set_alpha) + pid->Set_alpha * pid->Last_Ud;

        pid->out = pid->pout + pid->iout + pid->dout;
				if(pid->out > pid->MaxOutput)
				{
					pid->out = pid->MaxOutput;
					pid->Set_Out_Mode = 1;
				}
				else
				{
					pid->Set_Out_Mode = 0;
				}
				
				
				if(pid->out < -pid->MaxOutput)
				{
					pid->out = -pid->MaxOutput;
					pid->Set_Out_Mode = -1;
				}
				else
				{
					pid->Set_Out_Mode = 0;
				}
			pid->Last_Ud = pid->dout;

    }
		
    pid->Last_Last_error= pid->Last_error;
    pid->Last_error = pid->now_error;

    return pid->out;
}

float Gimbal_Yaw_Math_pid_calc(PID*pid, float now, float set)
{
    pid->now = now;
    pid->set = set;

		pid->now_error = pid->set - pid->now;	//set - measure

		pid->now_error = Angle_Limit(pid->now_error);
	
		if((pid->now_error < -0.4) || (pid->now_error > 0.4))
		{
			pid->kp = 16;
		}
		else
		{
			pid->kp = 6;
		}
		if((pid->now_error < pid->Small_Error_Limit) && (pid->now_error > -pid->Small_Error_Limit))
			pid->now_error = 0;
    if(pid->pid_mode == 1) //λ�û�PID
    {
	      pid->pout = pid->kp * pid->now_error;
        pid->iout = pid->ki * pid->sum_of_error;
        pid->dout = pid->kd * (pid->now_error - pid->Last_error );
				pid->sum_of_error+=pid->now_error;	
				PID_limit(&(pid->sum_of_error), 20000);
				PID_limit(&(pid->iout), pid->IntegralLimit);
        pid->out = pid->pout + pid->iout + pid->dout;
        PID_limit(&(pid->out), pid->MaxOutput);
    }	
		
    else if(pid->pid_mode == 2)//����ʽPID
    {
        pid->pout = pid->kp * (pid->now_error - pid->Last_error);
        pid->iout = pid->ki * pid->now_error;
        pid->dout = pid->kd * (pid->now_error - 2*pid->Last_error + pid->Last_Last_error);        
				PID_limit(&(pid->iout), pid->IntegralLimit);
        pid->plus = pid->pout + pid->iout + pid->dout;
        pid->plus_out = pid->last_plus_out + pid->plus;
			  pid->out = pid->plus_out; 
				PID_limit(&(pid->out), pid->MaxOutput);
        pid->last_plus_out = pid->plus_out;	//update last time
    }
		else if(pid->pid_mode == 3)//���ٻ���/����ȫ΢��
    {
	      pid->pout = pid->kp * pid->now_error;
				if(((pid->Set_Out_Mode == 1) && (pid->now_error > 0)) || ((pid->Set_Out_Mode == -1) && (pid->now_error < 0)));
				else
				{
					if(PID_Fabs(pid->now_error) <= pid->Set_B)
					{
						pid->sum_of_error += pid->now_error;
					}
					else if(PID_Fabs(pid->now_error) >= (pid->Set_B + pid->Set_A))
					{
						pid->sum_of_error = 0;
					}
					else
					{
						pid->Set_ratio = (pid->Set_A + pid->Set_B - PID_Fabs(pid->now_error)) / pid->Set_A;
						pid->sum_of_error += pid->Set_ratio * pid->now_error;
					}
				}
			//���ٻ���
      pid->iout = pid->ki * pid->sum_of_error;			
			//����ȫ΢��	
			pid->dout = pid->kd * (pid->now_error - pid->Last_error) * (1 - pid->Set_alpha) + pid->Set_alpha * pid->Last_Ud;

        pid->out = pid->pout + pid->iout + pid->dout;
				if(pid->out > pid->MaxOutput)
				{
					pid->out = pid->MaxOutput;
					pid->Set_Out_Mode = 1;
				}
				else
				{
					pid->Set_Out_Mode = 0;
				}
				
				
				if(pid->out < -pid->MaxOutput)
				{
					pid->out = -pid->MaxOutput;
					pid->Set_Out_Mode = -1;
				}
				else
				{
					pid->Set_Out_Mode = 0;
				}
			pid->Last_Ud = pid->dout;

    }
		
    pid->Last_Last_error= pid->Last_error;
    pid->Last_error = pid->now_error;

    return pid->out;
}


//��̨��ʼ��
void Gimbal_Init(Gimbal_t* Gimbal_Init)
{
	
	
	//pid���ݳ�ʼ��
	pid_math_init(&Gimbal_Init->Gimbal_Pitch_Motor_Angle_Pid,Gimbal_Pitch_Motor_Angle_Kp,Gimbal_Pitch_Motor_Angle_Ki,Gimbal_Pitch_Motor_Angle_Kd,Gimbal_Pitch_Motor_Angle_Maxout,Gimbal_Pitch_Motor_Angle_IMaxout,3,0.5f,0.05f,0.95f,0.0001f);
	pid_math_init(&Gimbal_Init->Gimbal_Yaw_Motor_Angle_Pid,Gimbal_Yaw_Motor_Angle_Kp,Gimbal_Yaw_Motor_Angle_Ki,Gimbal_Yaw_Motor_Angle_Kd,Gimbal_Yaw_Motor_Angle_Maxout,Gimbal_Yaw_Motor_Angle_IMaxout,3,0.5f,0.05f,0.96f,0.0001f);	
	pid_math_init(&Gimbal_Init->Gimbal_Yaw_IMU_Angle_Pid,Gimbal_Yaw_IMU_Angle_Kp,Gimbal_Yaw_IMU_Angle_Ki,Gimbal_Yaw_IMU_Angle_Kd,Gimbal_Yaw_IMU_Angle_Maxout,Gimbal_Yaw_IMU_Angle_IMaxout,3,0.5f,0.05f,0.96f,0.0001f);	
	
	 //���ٻ���΢��PID  ��ʱ�����
	//pid_math_init(&Gimbal_Init->Gimbal_Pitch_3508_Distance_Pid,Gimbal_Pitch_3508_Distance_Kp,Gimbal_Pitch_3508_Distance_Ki,Gimbal_Pitch_3508_Distance_Kd,Gimbal_Pitch_3508_Distance_Maxout,Gimbal_Pitch_3508_Distance_IMaxout,3,0.5f,0.05f,0.95f,0.0001f);
	//
	
	
	//3508 pitch�� PID���ݳ�ʼ��
	pid_init(&Gimbal_Init->Gimbal_Pitch_IMU_Angle_Pid,Gimbal_Pitch_IMU_Angle_Kp,Gimbal_Pitch_IMU_Angle_Ki,Gimbal_Pitch_IMU_Angle_Kd,Gimbal_Pitch_IMU_Angle_Maxout,Gimbal_Pitch_IMU_Angle_IMaxout,1);
	pid_init(&Gimbal_Init->Gimbal_Pitch_3508_Distance_Pid,Gimbal_Pitch_3508_Distance_Kp,Gimbal_Pitch_3508_Distance_Ki,Gimbal_Pitch_3508_Distance_Kd,Gimbal_Pitch_3508_Distance_Maxout,Gimbal_Pitch_3508_Distance_IMaxout,1);
		
	pid_init(&Gimbal_Init->Gimbal_Pitch_3508_Speed_Pid,Gimbal_Pitch_speed_3508_Angle_Kp,Gimbal_Pitch_speed_3508_Angle_Ki,Gimbal_Pitch_speed_3508_Angle_Kd,Gimbal_Pitch_speed_3508_Angle_Maxout,Gimbal_Pitch_speed_3508_Angle_IMaxout,1);
		
	pid_init(&Gimbal_Init->Gimbal_Yaw_Vision_Angle_Pid,Gimbal_Yaw_Vision_Angle_Kp,Gimbal_Yaw_Vision_Angle_Ki,Gimbal_Yaw_Vision_Angle_Kd,Gimbal_Yaw_Vision_Angle_Maxout,Gimbal_Yaw_Vision_Angle_IMaxout,1);
	
	pid_init(&Gimbal_Init->Gimbal_Yaw_Speed_Pid,Gimbal_Yaw_Speed_Kp,Gimbal_Yaw_Speed_Ki,Gimbal_Yaw_Speed_Kd,Gimbal_Yaw_Speed_Maxout,Gimbal_Yaw_Speed_IMaxout,1);
	pid_init(&Gimbal_Init->Gimbal_Pitch_Speed_Pid,Gimbal_Pitch_Speed_Kp,Gimbal_Pitch_Speed_Ki,Gimbal_Pitch_Speed_Kd,Gimbal_Pitch_Speed_Maxout,Gimbal_Pitch_Speed_IMaxout,1);

	pid_init(&Gimbal_Init->Gimbal_Pitch_Motor_Pid,Gimbal_Pitch_Motor_Kp,Gimbal_Pitch_Motor_Ki,Gimbal_Pitch_Motor_Kd,Gimbal_Pitch_Motor_Maxout,Gimbal_Pitch_Motor_IMaxout,1);
	pid_init(&Gimbal_Init->Gimbal_Yaw_Motor_Pid,Gimbal_Yaw_Motor_Kp,Gimbal_Yaw_Motor_Ki,Gimbal_Yaw_Motor_Kd,Gimbal_Yaw_Motor_Maxout,Gimbal_Yaw_Motor_IMaxout,1);

	
	//pid_init(&Gimbal_Init->Gimbal_Yaw_Motor_Pid,Gimbal_Yaw_Motor_Kp,Gimbal_Yaw_Motor_Ki,Gimbal_Yaw_Motor_Kd,Gimbal_Yaw_Motor_Maxout,Gimbal_Yaw_Motor_IMaxout,1);
	
	//������ݻ�ȡ��ַ
	Gimbal_Init->Gimbal_Yaw_Msg_t.Gimbal_Motor_Msg_Get = Get_DJI_Motor_Data(CAN2_RX,Gimbal_Yaw_Can);
	Gimbal_Init->Gimbal_Pitch_Msg_t.Gimbal_Motor_Msg_Get = Get_DJI_Motor_Data(CAN2_RX,Gimbal_Pitch_Can);
	
	//���������ݻ�ȡ��ַ
	Gimbal_Init->Gimbal_IMU_Angle = get_JY61_angle_Point();
	Gimbal_Init->Gimbal_IMU_Aspeed = get_JY61_Gyro_Data_Point();

	Gimbal_Init->Gimbal_Yaw_Msg_t.Gimbal_IMU_Angle_Data = get_JY61_angle_Point() + 2;
	Gimbal_Init->Gimbal_Pitch_Msg_t.Gimbal_IMU_Angle_Data =  get_JY61_angle_Point() + 0;
	Gimbal_Init->Gimbal_Yaw_Msg_t.Gimbal_IMU_Aspeed_Data = get_JY61_Gyro_Data_Point() + 2;
	Gimbal_Init->Gimbal_Pitch_Msg_t.Gimbal_IMU_Aspeed_Data = get_JY61_Gyro_Data_Point() + 0;

//	Gimbal_Init->Gimbal_IMU_Angle = get_INS_angle_point();
//	Gimbal_Init->Gimbal_IMU_Aspeed = get_MPU6500_Gyro_Data_Point();

//	Gimbal_Init->Gimbal_Yaw_Msg_t.Gimbal_IMU_Angle_Data = get_INS_angle_point() + 0;
//	Gimbal_Init->Gimbal_Pitch_Msg_t.Gimbal_IMU_Angle_Data =  get_INS_angle_point() + 2;
//	Gimbal_Init->Gimbal_Yaw_Msg_t.Gimbal_IMU_Aspeed_Data = get_MPU6500_Gyro_Data_Point() + 0;
//	Gimbal_Init->Gimbal_Pitch_Msg_t.Gimbal_IMU_Aspeed_Data = get_MPU6500_Gyro_Data_Point() + 2;
	
	
//	Gimbal_Init->Gimbal_Pitch_Msg_t.Angle_Init_flag = 0;
	
	
	//ң��������ֵ��ȡ��ַ
	Gimbal_Init->Gimbal_RC_Ctl_Data = Get_DJI_RC_Data_Address();
	
	//����е����ݳ�ʼ��
	Gimbal_Init->Gimbal_Yaw_Msg_t.Gimbal_Angle_Middle_Msg = Gimbal_Yaw_Middle_Angle;
	Gimbal_Init->Gimbal_Pitch_Msg_t.Gimbal_Angle_Middle_Msg = Gimbal_Pitch_Middle_Angle;
	
	//�������ֵ����
 	Gimbal_Init->Gimbal_Motor_Current_Send[0] = 0;
	Gimbal_Init->Gimbal_Motor_Current_Send[1] = 0;
}

void Gimbal_Data_Update(Gimbal_Msg_t* Gimbal_Msg_Update)
{
	Gimbal_Msg_Update->Gimbal_Motor_Angle_Msg = (Gimbal_Msg_Update->Gimbal_Motor_Msg_Get->angle - 4096.00f) /4096.00f * 3.1415926f;
	
	Gimbal_Msg_Update->Gimbal_IMU_Angle_Msg = (*(Gimbal_Msg_Update->Gimbal_IMU_Angle_Data))/180*PI;
	
	Gimbal_Msg_Update->Gimbal_Motor_Angle_TM = Angle_TM_Set(Gimbal_Msg_Update->Gimbal_Motor_Angle_Msg,Gimbal_Msg_Update->Gimbal_Angle_Middle_Msg);
	
	Gimbal_Msg_Update->Gimbal_Motor_Aspeed = (Gimbal_Msg_Update->Gimbal_Motor_Msg_Get->speed)/(60*2*3.14);

}



void Gimbal_IMU_Aspeed_Get(Gimbal_t* Gimbal_Aspeed)
{
	Gimbal_Aspeed->Gimbal_Pitch_Msg_t.Gimbal_IMU_Aspeed = -(*Gimbal_Aspeed->Gimbal_Pitch_Msg_t.Gimbal_IMU_Aspeed_Data);
	Gimbal_Aspeed->Gimbal_Yaw_Msg_t.Gimbal_IMU_Aspeed = arm_cos_f32(Gimbal_Aspeed->Gimbal_Pitch_Msg_t.Gimbal_Motor_Angle_TM) * (*(Gimbal_Aspeed->Gimbal_IMU_Aspeed + 2))
                                                        - arm_sin_f32(Gimbal_Aspeed->Gimbal_Pitch_Msg_t.Gimbal_Motor_Angle_TM) * (*(Gimbal_Aspeed->Gimbal_IMU_Aspeed + 1));
	
}

/*****������ݸ��º���*****/
void Gimbal_3508_Motor_Data_Update(Gimbal_Msg_t_Pitch* Gimbal_Pitch_Motor_Msg_Update)
{
	//3508ת�����ת��
	Gimbal_Pitch_Motor_Msg_Update->Gimbal_Pitch_Speed_Get = ((float)(Gimbal_Pitch_Motor_Msg_Update->Gimbal_Motor_Msg_Get->speed)*0.0012708333f);///60*2*3.1415926 *152.5/2/1000
	//IMU Pitch�������
	Gimbal_Pitch_Motor_Msg_Update->Gimbal_IMU_Angle_Msg = *(Gimbal_Pitch_Motor_Msg_Update->Gimbal_IMU_Angle_Data);
  //ת���ۼ�     	
	Gimbal_Pitch_Motor_Msg_Update->Motor_Run_Long += Gimbal_Pitch_Motor_Msg_Update->Gimbal_Motor_Msg_Get->speed;
	//���õ���ٶ�     ����ת����ɾ���  ̧ͷ������+  ��ͷ������-
	Gimbal_Pitch_Motor_Msg_Update->Motor_Run_Dis_Get_turn_by_speed = -(float)(Gimbal_Pitch_Motor_Msg_Update->Motor_Run_Long / 1000 / 60 / 3591 * 187 * 10);	
	//���õ����е�Ƕ� ����ת����ɾ���  ̧ͷ������+  ��ͷ������-
	Gimbal_Pitch_Motor_Msg_Update->Motor_Run_Dis_Get_turn_by_angle = -(float)((float)Gimbal_Pitch_Motor_Msg_Update->Gimbal_Motor_Msg_Get->Angle_Long / 8192 / 3591 * 187 * 14);
	
	
	//�ϰ�  �޷��������
	//���������ǵ�Pitch�������
	Gimbal_Pitch_Motor_Msg_Update->Gimbal_Control_Pitch_IMU = fAngle[2];	
	if( Gimbal_Pitch_Motor_Msg_Update->Gimbal_Control_Pitch_IMU != 0 && Gimbal_Pitch_Motor_Msg_Update->Angle_Init_flag == 0 )
	{	 
      Gimbal_Pitch_Motor_Msg_Update->Gimbal_Middle_Angle = Gimbal_Pitch_Motor_Msg_Update->Gimbal_Control_Pitch_IMU;
	    Gimbal_Pitch_Motor_Msg_Update->Gimbal_Max_Angle = Gimbal_Pitch_Motor_Msg_Update->Gimbal_Middle_Angle	+ Pitch_Deviation_Max;	
	    Gimbal_Pitch_Motor_Msg_Update->Gimbal_Min_Angle = Gimbal_Pitch_Motor_Msg_Update->Gimbal_Middle_Angle	- Pitch_Deviation_Max;	
		  Gimbal_Pitch_Motor_Msg_Update->Angle_Init_flag = 1;
	}
		
}

extern int Check_Motionless_Flag;
void Gimbal_Mode_Set(Gimbal_t* Mode_Set)
{
	static int RC_Motionless_Count = 0;
	static int Last_RC;
	
	if(DJI_RC_Motion_Check() == 1)
	{
		RC_Motionless_Count = 0;
	}
	else if(DJI_RC_Motion_Check() == 0)
	{
//		RC_Motionless_Count ++;
	}
	
	if(RC_Motionless_Count > Motionless_Time)
	{
		if((Mode_Set->Gimbal_Mode == Gimbal_Motor_Control) || (Mode_Set->Gimbal_Mode == Gimbal_IMU_Control))
			{
				Mode_Set->Gimbal_Mode = Gimbal_Motionless;
			}
		
	}
	else
	{
			if(Mode_Set->Gimbal_RC_Ctl_Data->rc.s2 == RC_SW_MID)
			{
				Mode_Set->Gimbal_Mode = Gimbal_Second_Mode;
			}
			else if(Mode_Set->Gimbal_RC_Ctl_Data->rc.s2 == RC_SW_UP)
			{
				Mode_Set->Gimbal_Mode = Gimbal_Third_Mode;
			}	
	}
	
	if((Mode_Set->Gimbal_RC_Ctl_Data->rc.s2 == RC_SW_MID) && (Last_RC == RC_SW_DOWN))
	{
		Mode_Set->Gimbal_Yaw_Control_Data = 0;
		Mode_Set->Gimbal_Pitch_Control_Data = 0;
	}
	
	if((Mode_Set->Gimbal_RC_Ctl_Data->rc.s2 == RC_SW_DOWN) || (Last_RC != Mode_Set->Gimbal_RC_Ctl_Data->rc.s2)) 
	{
			RC_Motionless_Count = 0;
	}
	
	if(Mode_Set->Gimbal_RC_Ctl_Data->rc.s2 == RC_SW_DOWN)
	{
		Mode_Set->Gimbal_Mode = Gimbal_Zero;
	}
			
	
	Last_RC = Mode_Set->Gimbal_RC_Ctl_Data->rc.s2;
}

void Gimbal_Mode_Change(Gimbal_t* Mode_Change)
{
	static Gimbal_Mode_t Last_Mode_t;
	
	if((Last_Mode_t != Gimbal_Motionless) && (Mode_Change->Gimbal_Mode == Gimbal_Motionless))
	{
		Mode_Change->Gimbal_Yaw_Control_Data = Mode_Change->Gimbal_Yaw_Msg_t.Gimbal_Motor_Angle_TM;
		Mode_Change->Gimbal_Pitch_Control_Data = Mode_Change->Gimbal_Pitch_Msg_t.Gimbal_Motor_Angle_TM;	
	}
	
	if((Last_Mode_t != Gimbal_Motor_Control) && (Mode_Change->Gimbal_Mode == Gimbal_Motor_Control) && (Last_Mode_t != Gimbal_Zero))
	{
		Mode_Change->Gimbal_Yaw_Control_Data = Mode_Change->Gimbal_Yaw_Msg_t.Gimbal_Motor_Angle_TM;
		Mode_Change->Gimbal_Pitch_Control_Data = Mode_Change->Gimbal_Pitch_Msg_t.Gimbal_Motor_Angle_TM;
	}
	
	if((Mode_Change->Gimbal_Mode == Gimbal_Motor_Control) && (Last_Mode_t = Gimbal_Zero))
	{
		Mode_Change->Gimbal_Yaw_Control_Data = 0;
		Mode_Change->Gimbal_Pitch_Control_Data = 0;
	}
		
	if((Last_Mode_t != Gimbal_Spin) && (Mode_Change->Gimbal_Mode == Gimbal_Spin))
	{
		Mode_Change->Gimbal_Yaw_Control_Data = Mode_Change->Gimbal_Yaw_Msg_t.Gimbal_IMU_Angle_Msg;
		Mode_Change->Gimbal_Pitch_Control_Data = Mode_Change->Gimbal_Pitch_Msg_t.Gimbal_Motor_Angle_TM;
	}
	
	if((Last_Mode_t != Gimbal_Vision_Control) && (Mode_Change->Gimbal_Mode == Gimbal_Vision_Control))
	{
		Mode_Change->Gimbal_Yaw_Control_Data = Mode_Change->Gimbal_Yaw_Msg_t.Gimbal_IMU_Angle_Msg;
		Mode_Change->Gimbal_Pitch_Control_Data = Mode_Change->Gimbal_Pitch_Msg_t.Gimbal_Motor_Angle_TM;
	}
	
	Last_Mode_t = Mode_Change->Gimbal_Mode;
}



float Angle_tttt;
void Gimbal_IMU_Control_Data_Check(Gimbal_t* Gimbal_Data_Check)
{
	float Old_Data,Change_Data,Add,Out_Data;
	//Yaw
	Old_Data = Gimbal_Data_Check->Gimbal_Yaw_Msg_t.Gimbal_Motor_Angle_TM;
	Change_Data = Angle_TM_Set(Gimbal_Data_Check->Gimbal_Yaw_Control_Data,Gimbal_Data_Check->Gimbal_Yaw_Msg_t.Gimbal_IMU_Angle_Msg);
	Add = Gimbal_Data_Check->Gimbal_Yaw_Add_Data;
	
	if(Old_Data + Change_Data + Add > Yaw_Max_Angle)
	{
		Add = Yaw_Max_Angle - Old_Data - Change_Data; 
	}
	
	if(Old_Data + Change_Data + Add < Yaw_Min_Angle)
	{
		Add = Yaw_Min_Angle - Old_Data - Change_Data; 
	}
	
	Out_Data = Angle_Limit(Gimbal_Data_Check->Gimbal_Yaw_Control_Data + Add);
		
	Gimbal_Data_Check->Gimbal_Yaw_Control_Data = Out_Data;
	

	
	
}

void Gimbal_IMU_Control_Data_No_Limit(Gimbal_t* Gimbal_Data_Check)
{
	float Old_Data,Change_Data,Add,Out_Data;
	//Yaw

	
	Out_Data = Angle_Limit(Gimbal_Data_Check->Gimbal_Yaw_Control_Data + Add);
		
	Gimbal_Data_Check->Gimbal_Yaw_Control_Data = Out_Data;
	

	
	Out_Data = Angle_Limit(Gimbal_Data_Check->Gimbal_Pitch_Control_Data + Add);
		
	Gimbal_Data_Check->Gimbal_Pitch_Control_Data = Out_Data;
	

}

void Gimbal_Motor_Control_Data_Check(Gimbal_t* Gimbal_Data_Check)
{
//	Gimbal_Data_Check->
}

//BUG[5.2]�޷�180��תͷ���Ƕ�������
float Mouse_X_Set = 30000.0f,Mouse_Y_Set = 40000.0f;

float Gimbal_Yaw_Now_Angle,Gimbal_Yaw_Target_Angle;

float Gimbal_Pitch_Now_Angle,Gimbal_Pitch_Target_Angle;

int Gimbal_Turn_Head_Flag,Gimbal_Turn_Head_Flag_Last,Gimbal_Key_Board_Last;
float Gimbal_Turn_Head_Angle_Add,Gimbal_Turn_Head_Angle_Limit;
float Jscope_IMU_now_yaw_angle,Jscope_IMU_set_yaw_angle;

//��̨�����ǿ��ƣ�������ģʽ
void Gimbal_IMU_Control_NL_Set(Gimbal_t* Gimbal_Control_Set)
{
	
	Gimbal_Control_Set->Gimbal_Yaw_Add_Data = - (float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->rc.ch2)/6600.000) / 180.00f * 3.1415926f - (float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->mouse.x) / Mouse_X_Set);
	Gimbal_Control_Set->Gimbal_Pitch_Add_Data = -(float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->rc.ch3)/13200.000) / 180.00f * 3.1415926f + (float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->mouse.y) / Mouse_Y_Set);

	Gimbal_Yaw_Now_Angle = Gimbal_Control_Set->Gimbal_Yaw_Msg_t.Gimbal_Motor_Angle_Msg;	
	
	if((Gimbal_Turn_Head_Flag == 0) && (!(Gimbal_Key_Board_Last  & Gimbal_Turn_Head_Key) && (Gimbal_Control_Set->Gimbal_RC_Ctl_Data->key.v & Gimbal_Turn_Head_Key)))
	{
		Gimbal_Yaw_Target_Angle = Angle_Limit(Gimbal_Yaw_Now_Angle + PI);
		Gimbal_Turn_Head_Flag = 1;
	}
	else if((Gimbal_Turn_Head_Flag == 1) && (!(Gimbal_Key_Board_Last  & Gimbal_Turn_Head_Key) && (Gimbal_Control_Set->Gimbal_RC_Ctl_Data->key.v & Gimbal_Turn_Head_Key)))
	{		
		Gimbal_Turn_Head_Flag = 0;
	}
	
	if(Angle_Abs(Angle_Limit(Gimbal_Yaw_Target_Angle - Gimbal_Yaw_Now_Angle)) < 0.1f)
	{
		Gimbal_Turn_Head_Flag = 0;
	}
		

	
	if(Gimbal_Turn_Head_Flag == 1)
	{

		if(Angle_Limit(Gimbal_Yaw_Target_Angle - Gimbal_Yaw_Now_Angle) > 0.00f)
		{
			Gimbal_Turn_Head_Angle_Add = Gimbal_Turn_Head_Speed;
		}			
		else
		{
			Gimbal_Turn_Head_Angle_Add = -Gimbal_Turn_Head_Speed;
		}

	}
	else if(Gimbal_Turn_Head_Flag == 0)
	{
		Gimbal_Turn_Head_Angle_Add = 0;
	}
	
	
	Gimbal_Turn_Head_Flag_Last = Gimbal_Turn_Head_Flag;
	Gimbal_Key_Board_Last = Gimbal_Control_Set->Gimbal_RC_Ctl_Data->key.v;
	
	Gimbal_Control_Set->Gimbal_Yaw_Add_Data += Gimbal_Turn_Head_Angle_Add;
	
	Gimbal_Control_Set->Gimbal_Yaw_Control_Data += Gimbal_Control_Set->Gimbal_Yaw_Add_Data;
	Gimbal_Control_Set->Gimbal_Pitch_Control_Data += Gimbal_Control_Set->Gimbal_Pitch_Add_Data;
	
	Gimbal_Control_Set->Gimbal_Yaw_Control_Data = Angle_Limit(Gimbal_Control_Set->Gimbal_Yaw_Control_Data);
	Angle_Control_Limit(&Gimbal_Control_Set->Gimbal_Pitch_Control_Data,Pitch_Max_Angle,Pitch_Min_Angle);
//	Gimbal_Control_Set->Gimbal_Pitch_Control_Data = Angle_Limit(Gimbal_Control_Set->Gimbal_Pitch_Control_Data);
	
	Angle_Control_Limit(&Gimbal_Control_Set->Gimbal_Pitch_Control_Data,Pitch_Max_Angle,Pitch_Min_Angle);
	
//	Angle_Control_Limit(&Gimbal_Control_Set->Gimbal_Pitch_Control_Data,Pitch_Max_Angle,Pitch_Min_Angle);
//	Gimbal_IMU_Control_Data_No_Limit(Gimbal_Control_Set);

	Gimbal_Control_Set->Gimbal_Pitch_Apid_Out = \
	Gimbal_Math_pid_calc(&Gimbal_Control_Set->Gimbal_Pitch_Motor_Angle_Pid,\
	Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Gimbal_Motor_Angle_TM,Gimbal_Control_Set->Gimbal_Pitch_Control_Data);

	Gimbal_Control_Set->Gimbal_Motor_Current_Send[1] = \
	pid_calc(&Gimbal_Control_Set->Gimbal_Pitch_Speed_Pid,\
	Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Gimbal_Motor_Aspeed,Gimbal_Control_Set->Gimbal_Pitch_Apid_Out);
	
  	
	Gimbal_Control_Set->Gimbal_Yaw_Apid_Out = \
	Gimbal_Math_pid_calc(&Gimbal_Control_Set->Gimbal_Yaw_IMU_Angle_Pid,\
	Gimbal_Control_Set->Gimbal_Yaw_Msg_t.Gimbal_IMU_Angle_Msg,Gimbal_Control_Set->Gimbal_Yaw_Control_Data);

  //��+ ��-
	Gimbal_Control_Set->Gimbal_Motor_Current_Send[0] =\
	pid_calc(&Gimbal_Control_Set->Gimbal_Yaw_Speed_Pid,\
	Gimbal_Control_Set->Gimbal_Yaw_Msg_t.Gimbal_Motor_Aspeed,Gimbal_Control_Set->Gimbal_Yaw_Apid_Out);
	
		
}

//��̨IMU���������ݿ���ģʽ
void Gimbal_IMU_Control_Set(Gimbal_t* Gimbal_Control_Set)
{
	
	Gimbal_Control_Set->Gimbal_Yaw_Add_Data = - (float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->rc.ch2)/6600.000) / 180.00f * 3.1415926f - (float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->mouse.x) / Mouse_X_Set);
	Gimbal_Control_Set->Gimbal_Pitch_Add_Data = -(float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->rc.ch3)/13200.000) / 180.00f * 3.1415926f + (float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->mouse.y) / Mouse_Y_Set);
	
	
	Gimbal_Control_Set->Gimbal_Pitch_Control_Data += Gimbal_Control_Set->Gimbal_Pitch_Add_Data;
	
	Gimbal_IMU_Control_Data_Check(Gimbal_Control_Set);
	Angle_Control_Limit(&Gimbal_Control_Set->Gimbal_Pitch_Control_Data,Pitch_Max_Angle,Pitch_Min_Angle);
	
	Gimbal_Control_Set->Gimbal_Pitch_Apid_Out = \
	Gimbal_Math_pid_calc(&Gimbal_Control_Set->Gimbal_Pitch_Motor_Angle_Pid,\
	Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Gimbal_Motor_Angle_TM,Gimbal_Control_Set->Gimbal_Pitch_Control_Data);

	Gimbal_Control_Set->Gimbal_Motor_Current_Send[1] = \
	pid_calc(&Gimbal_Control_Set->Gimbal_Pitch_Speed_Pid,\
	Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Gimbal_IMU_Aspeed,Gimbal_Control_Set->Gimbal_Pitch_Apid_Out);
	
	
	
	Gimbal_Control_Set->Gimbal_Yaw_Apid_Out = \
	Gimbal_Yaw_Math_pid_calc(&Gimbal_Control_Set->Gimbal_Yaw_IMU_Angle_Pid,\
	Gimbal_Control_Set->Gimbal_Yaw_Msg_t.Gimbal_IMU_Angle_Msg,Gimbal_Control_Set->Gimbal_Yaw_Control_Data);
	
	Gimbal_Control_Set->Gimbal_Motor_Current_Send[0] = \
	pid_calc(&Gimbal_Control_Set->Gimbal_Yaw_Speed_Pid,\
	Gimbal_Control_Set->Gimbal_Yaw_Msg_t.Gimbal_Motor_Aspeed,Gimbal_Control_Set->Gimbal_Yaw_Apid_Out);
}    

//��̨��������ݿ���ģʽ
int Pitch_Pid_Out;
float Yaw_Data_J_Scope_Get[3];

void Gimbal_Motor_Control_Set(Gimbal_t* Gimbal_Control_Set)
{	
	Gimbal_Control_Set->Gimbal_Yaw_Add_Data = - (float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->rc.ch2)/6600.000) / 180.00f * 3.1415926f - (float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->mouse.x) / Mouse_X_Set);
	Gimbal_Control_Set->Gimbal_Pitch_Add_Data = -(float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->rc.ch3)/13200.000) / 180.00f * 3.1415926f + (float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->mouse.y) / Mouse_Y_Set);

	Gimbal_Control_Set->Gimbal_Yaw_Control_Data += Gimbal_Control_Set->Gimbal_Yaw_Add_Data;
	Gimbal_Control_Set->Gimbal_Pitch_Control_Data += Gimbal_Control_Set->Gimbal_Pitch_Add_Data;
	
	Angle_Control_Limit(&Gimbal_Control_Set->Gimbal_Pitch_Control_Data,Pitch_Max_Angle,Pitch_Min_Angle);
	Angle_Control_Limit(&Gimbal_Control_Set->Gimbal_Yaw_Control_Data,Yaw_Max_Angle,Yaw_Min_Angle);
	

	Gimbal_Control_Set->Gimbal_Pitch_Apid_Out = \
	Gimbal_Math_pid_calc(&Gimbal_Control_Set->Gimbal_Pitch_Motor_Angle_Pid,\
	Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Gimbal_Motor_Angle_TM,Gimbal_Control_Set->Gimbal_Pitch_Control_Data);
	
	Gimbal_Control_Set->Gimbal_Motor_Current_Send[1] = \
	pid_calc(&Gimbal_Control_Set->Gimbal_Pitch_Speed_Pid,\
	Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Gimbal_Motor_Aspeed,Gimbal_Control_Set->Gimbal_Pitch_Apid_Out);
	
	
	Gimbal_Control_Set->Gimbal_Yaw_Apid_Out = \
	Gimbal_Yaw_Math_pid_calc(&Gimbal_Control_Set->Gimbal_Yaw_Motor_Angle_Pid,\
	Gimbal_Control_Set->Gimbal_Yaw_Msg_t.Gimbal_Motor_Angle_TM,Gimbal_Control_Set->Gimbal_Yaw_Control_Data);

	Gimbal_Control_Set->Gimbal_Motor_Current_Send[0] = \
	pid_calc(&Gimbal_Control_Set->Gimbal_Yaw_Speed_Pid,\
	Gimbal_Control_Set->Gimbal_Yaw_Msg_t.Gimbal_Motor_Aspeed,Gimbal_Control_Set->Gimbal_Yaw_Apid_Out);
}

 //˫������+����+�ٶȻ�
float J_scope_Distnace = 0;
void Gimbal_Motor_Control_Set_Final_Pitch(Gimbal_t* Gimbal_Control_Set)
{	    
//	
//	    Gimbal_Control_Set->Gimbal_Yaw_Add_Data = (float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->rc.ch2)*Yaw_RC_Sen) / 180.00f * 3.1415926f + (float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->mouse.x) / Mouse_X_Set);

//	
//			//����ÿ�μ��� ���ӵ����ֵ��ֹ����    
//				Angle_Control_Limit(&Gimbal_Control_Set->Gimbal_Yaw_Add_Data,0.003f,-0.003f);
//			//�ۼ�ÿ�ε�����ֵ ��ɿ���ֵ	
//			 Gimbal_Control_Set->Gimbal_Yaw_Control_Data += Gimbal_Control_Set->Gimbal_Yaw_Add_Data;		
//			//����YAW��� ��е�Ƕ� ��� ��Сֵ
//			Angle_Control_Limit(&Gimbal_Control_Set->Gimbal_Yaw_Control_Data,Yaw_Max_Angle,Yaw_Min_Angle);
//	    //��� ����˫�����
//		 Gimbal_Control_Set->Gimbal_Yaw_Apid_Out = \
//		 Gimbal_Math_pid_calc(&Gimbal_Control_Set->Gimbal_Yaw_Motor_Angle_Pid,\
//		 Gimbal_Control_Set->Gimbal_Yaw_Msg_t.Gimbal_Motor_Angle_TM,Gimbal_Control_Set->Gimbal_Yaw_Control_Data);

//		 Gimbal_Control_Set->Gimbal_Motor_Current_Send[0] = \
//		 pid_calc(&Gimbal_Control_Set->Gimbal_Yaw_Speed_Pid,\
//		 Gimbal_Control_Set->Gimbal_Yaw_Msg_t.Gimbal_Motor_Aspeed,Gimbal_Control_Set->Gimbal_Yaw_Apid_Out);
//	
//	   
//	   	
//	  // �޸ĺ��Ӣ����3508+˿�˿��� ������ �ǶȻ�+�ٶȻ�����
//		//ֱ������Pitch��ľ�������
////		Gimbal_Control_Set->Gimbal_Pitch_Add_Data = -(float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->rc.ch3)/660.000f * 10) + (float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->mouse.y) / Mouse_Y_Set);
//      Gimbal_Control_Set->Gimbal_Pitch_Add_Data = -(float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->rc.ch3)/660.000f * 20) + (float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->mouse.y) / Mouse_Y_Set);		
//			//����ÿ�μ��� ���ӵ����ֵ��ֹ����    
//			Angle_Control_Limit(&Gimbal_Control_Set->Gimbal_Pitch_Add_Data,0.05f,-0.05f);
//			
////		if( Gimbal_Control_Set->Gimbal_Pitch_Add_Data <= 0)
////		{
////      Gimbal_Control_Set->Gimbal_Pitch_Rise_Add_Data = Gimbal_Control_Set->Gimbal_Pitch_Add_Data;
////		}
////    if( Gimbal_Control_Set->Gimbal_Pitch_Add_Data > 0)
////    {			
////      Gimbal_Control_Set->Gimbal_Pitch_Submit_Add_Data = Gimbal_Control_Set->Gimbal_Pitch_Add_Data;			
////		}
//		
////   �������Ʒ�ֵ  ����ת���ۼƵľ��� ��  �Ƕ��ۼƵľ��� һ���ж�     ȱ���� �ϵ��ʼʱ ��̨Pitch��Ҫ���м�
////    if(Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Motor_Run_Dis_Get_turn_by_angle >= Pitch_Max_Angle_Distance_By_Angle)
////       Gimbal_Control_Set->Gimbal_Pitch_Rise_Add_Data = 0;
////		if(Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Motor_Run_Dis_Get_turn_by_angle <= Pitch_Min_Angle_Distance_By_Angle)
////			 Gimbal_Control_Set->Gimbal_Pitch_Submit_Add_Data = 0;
////		if(Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Motor_Run_Dis_Get_turn_by_speed >= Pitch_Max_Angle_Distance_By_Speed)
////			 Gimbal_Control_Set->Gimbal_Pitch_Rise_Add_Data = 0;
////		if(Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Motor_Run_Dis_Get_turn_by_speed <= Pitch_Min_Angle_Distance_By_Speed)
////			 Gimbal_Control_Set->Gimbal_Pitch_Submit_Add_Data = 0;
//		
////		�Ƕ����Ʒ�ֵ
////		if(Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Gimbal_Control_Pitch_IMU >= Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Gimbal_Max_Angle)
////					{Gimbal_Control_Set->Gimbal_Yaw_Rise_Add_Data = 0;}
////		else if(Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Gimbal_Control_Pitch_IMU <= Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Gimbal_Min_Angle)
////					{Gimbal_Control_Set->Gimbal_Yaw_Submit_Add_Data = 0;}

//    J_scope_Distnace = (-Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Motor_Run_Dis_Get_turn_by_angle);
//    
//		Gimbal_Control_Set->Gimbal_Pitch_Control_Data += Gimbal_Control_Set->Gimbal_Pitch_Add_Data;
//		
//		Angle_Control_Limit(&Gimbal_Control_Set->Gimbal_Pitch_Control_Data,Pitch_Max_Angle_Distance_By_Angle,Pitch_Min_Angle_Distance_By_Angle);
//							
//    // �޸ĺ��Ӣ����3508+˿�˿��� ������ ���뻷+�ٶȻ�����

//		Gimbal_Control_Set->Gimbal_Pitch_Apid_Out = \
//		pid_calc(&Gimbal_Control_Set->Gimbal_Pitch_3508_Distance_Pid,\
//		(-Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Motor_Run_Dis_Get_turn_by_angle),Gimbal_Control_Set->Gimbal_Pitch_Control_Data);
//		
//		//̧ͷС��0   ��ͷ����0
//		Gimbal_Control_Set->Gimbal_Motor_Current_Send[1] = \          
//		pid_calc(&Gimbal_Control_Set->Gimbal_Pitch_3508_Speed_Pid,\
//		Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Gimbal_Pitch_Speed_Get,Gimbal_Control_Set->Gimbal_Pitch_Apid_Out);
				
}




//��̨ Pitch�����ٶȻ�����
void Gimbal_Motor_Control_Set_test(Gimbal_t* Gimbal_Control_Set)
{
	
//	  //ֱ������Pitch���Ŀ���ٶ�  
//	  Gimbal_Control_Set->Gimbal_Pitch_Control_Data = (float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->rc.ch3)/660.000f * 10) - (float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->mouse.y) / Mouse_Y_Set);   
//	
//	 
//    Gimbal_Control_Set->Gimbal_Pitch_Control_Data = Pitch_Angle_Data_Add_Limit(&Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Gimbal_IMU_Angle_Msg,Pitch_IMU_Max_Angle,Pitch_IMU_Min_Angle);
//	
//    // �޸ĺ��Ӣ����3508+˿�˿��� ������ �ǶȻ�+�ٶȻ�����		
//		Gimbal_Control_Set->Gimbal_Motor_Current_Send[1] = \
//		pid_calc(&Gimbal_Control_Set->Gimbal_Pitch_3508_Speed_Pid,\
//		Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Gimbal_Pitch_Speed_Get,Gimbal_Control_Set->Gimbal_Pitch_Control_Data);
	
}



//��̨������̣�yaw�᲻��ģʽ
void Gimbal_Follow_Chassis_Set(Gimbal_t* Gimbal_Control_Set)
{
	
	Gimbal_Control_Set->Gimbal_Yaw_Add_Data = 0 ;
	Gimbal_Control_Set->Gimbal_Pitch_Add_Data = (float)((RC_Ctl_Data.rc.ch3)/6600.000) / 180.00f * 3.1415926f  - (float)((Gimbal_Control_Set->Gimbal_RC_Ctl_Data->mouse.y) / Mouse_Y_Set);

	Gimbal_Control_Set->Gimbal_Yaw_Control_Data  = 0;
	Gimbal_Control_Set->Gimbal_Pitch_Control_Data += Gimbal_Control_Set->Gimbal_Pitch_Add_Data;
	
	Angle_Control_Limit(&Gimbal_Control_Set->Gimbal_Pitch_Control_Data,Pitch_Max_Angle,Pitch_Min_Angle);
	Angle_Control_Limit(&Gimbal_Control_Set->Gimbal_Yaw_Control_Data,Yaw_Max_Angle,Yaw_Min_Angle);
	
	
	Gimbal_Control_Set->Gimbal_Pitch_Apid_Out = \
	Gimbal_pid_calc(&Gimbal_Control_Set->Gimbal_Pitch_Motor_Angle_Pid,\
	Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Gimbal_Motor_Angle_TM,Gimbal_Control_Set->Gimbal_Pitch_Control_Data);
	
	Gimbal_Control_Set->Gimbal_Motor_Current_Send[1] = \
	pid_calc(&Gimbal_Control_Set->Gimbal_Pitch_Speed_Pid,\
	Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Gimbal_Motor_Aspeed,Gimbal_Control_Set->Gimbal_Pitch_Apid_Out);


	Gimbal_Control_Set->Gimbal_Motor_Current_Send[0] = \
	Gimbal_pid_calc(&Gimbal_Control_Set->Gimbal_Yaw_Motor_Pid,\
	Gimbal_Control_Set->Gimbal_Yaw_Msg_t.Gimbal_Motor_Angle_TM,Gimbal_Control_Set->Gimbal_Yaw_Control_Data);
	

}




extern float yaw_angle_get,pitch_angle_get;
float Yaw_Vision_Plus = 0.00026,Pitch_Vision_Plus = 0.00012;//float Yaw_Vision_Plus = 0.000157999995,Pitch_Vision_Plus = 0.00013888;
void Gimbal_Vision_Control_Set(Gimbal_t* Gimbal_Control_Set)
{
	
	Gimbal_Control_Set->Gimbal_Yaw_Add_Data = yaw_angle_get * Yaw_Vision_Plus;//- (float)((RC_Ctl_Data.rc.ch2)/6600.000) / 180.00f * 3.1415926f
	Gimbal_Control_Set->Gimbal_Pitch_Add_Data = -pitch_angle_get  * Pitch_Vision_Plus ;//-(float)((RC_Ctl_Data.rc.ch3)/6600.000) / 180.00f * 3.1415926f

	Gimbal_Control_Set->Gimbal_Yaw_Control_Data += Gimbal_Control_Set->Gimbal_Yaw_Add_Data;
	Gimbal_Control_Set->Gimbal_Pitch_Control_Data += Gimbal_Control_Set->Gimbal_Pitch_Add_Data;
	
	Angle_Control_Limit(&Gimbal_Control_Set->Gimbal_Pitch_Control_Data,Pitch_Max_Angle,Pitch_Min_Angle);
	Angle_Control_Limit(&Gimbal_Control_Set->Gimbal_Yaw_Control_Data,Yaw_Max_Angle,Yaw_Min_Angle);
	
	
	Gimbal_Control_Set->Gimbal_Pitch_Apid_Out = \
	Gimbal_Math_pid_calc(&Gimbal_Control_Set->Gimbal_Pitch_Motor_Angle_Pid,\
	Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Gimbal_Motor_Angle_TM,Gimbal_Control_Set->Gimbal_Pitch_Control_Data);
	

	Gimbal_Control_Set->Gimbal_Motor_Current_Send[1] = \
	pid_calc(&Gimbal_Control_Set->Gimbal_Pitch_Speed_Pid,\
	Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Gimbal_IMU_Aspeed,Gimbal_Control_Set->Gimbal_Pitch_Apid_Out);
		
	Gimbal_Control_Set->Gimbal_Yaw_Apid_Out = \
	Gimbal_pid_calc(&Gimbal_Control_Set->Gimbal_Yaw_Vision_Angle_Pid,\
	Gimbal_Control_Set->Gimbal_Yaw_Add_Data,0);

	Gimbal_Control_Set->Gimbal_Motor_Current_Send[0] = \
	pid_calc(&Gimbal_Control_Set->Gimbal_Yaw_Speed_Pid,\
	Gimbal_Control_Set->Gimbal_Yaw_Msg_t.Gimbal_Motor_Aspeed,Gimbal_Control_Set->Gimbal_Yaw_Apid_Out);
}


void Gimbal_MotionLess_Control_Set(Gimbal_t* Gimbal_Control_Set)
{
	
	Gimbal_Control_Set->Gimbal_Yaw_Add_Data = 0;
	Gimbal_Control_Set->Gimbal_Pitch_Add_Data = 0;

	Gimbal_Control_Set->Gimbal_Yaw_Control_Data += Gimbal_Control_Set->Gimbal_Yaw_Add_Data;
	Gimbal_Control_Set->Gimbal_Pitch_Control_Data += Gimbal_Control_Set->Gimbal_Pitch_Add_Data;
	
	Angle_Control_Limit(&Gimbal_Control_Set->Gimbal_Pitch_Control_Data,Pitch_Max_Angle,Pitch_Min_Angle);
	Angle_Control_Limit(&Gimbal_Control_Set->Gimbal_Yaw_Control_Data,Yaw_Max_Angle,Yaw_Min_Angle);
	
	
	Gimbal_Control_Set->Gimbal_Pitch_Apid_Out = \
	Gimbal_pid_calc(&Gimbal_Control_Set->Gimbal_Pitch_Motor_Angle_Pid,\
	Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Gimbal_Motor_Angle_TM,Gimbal_Control_Set->Gimbal_Pitch_Control_Data);

	Gimbal_Control_Set->Gimbal_Motor_Current_Send[1] = \
	pid_calc(&Gimbal_Control_Set->Gimbal_Pitch_Speed_Pid,\
	Gimbal_Control_Set->Gimbal_Pitch_Msg_t.Gimbal_Motor_Aspeed,Gimbal_Control_Set->Gimbal_Pitch_Apid_Out);
	
	
	
	Gimbal_Control_Set->Gimbal_Yaw_Apid_Out = \
	Gimbal_pid_calc(&Gimbal_Control_Set->Gimbal_Yaw_Motor_Angle_Pid,\
	Gimbal_Control_Set->Gimbal_Yaw_Msg_t.Gimbal_Motor_Angle_TM,Gimbal_Control_Set->Gimbal_Yaw_Control_Data);

	Gimbal_Control_Set->Gimbal_Motor_Current_Send[0] = \
	pid_calc(&Gimbal_Control_Set->Gimbal_Yaw_Speed_Pid,\
	Gimbal_Control_Set->Gimbal_Yaw_Msg_t.Gimbal_Motor_Aspeed,Gimbal_Control_Set->Gimbal_Yaw_Apid_Out);
	

}



void Gimbal_Mode_Config(Gimbal_t* Gimbal_Config)
{
	if(Gimbal_Config->Gimbal_Mode == Gimbal_Zero)
	{
		Gimbal_Config->Gimbal_Motor_Current_Send[0] = 0;
		Gimbal_Config->Gimbal_Motor_Current_Send[1] = 0;
	}
	else if(Gimbal_Config->Gimbal_Mode == Gimbal_Motor_Control)
	{
		Gimbal_Motor_Control_Set(Gimbal_Config);
		//Gimbal_Motor_Control_Set_Final_Pitch(Gimbal_Config);
		//Gimbal_Motor_Control_Set_test(Gimbal_Config);  //    ����Pitch��  �����ǽǶ��� 
	}
	else if(Gimbal_Config->Gimbal_Mode == Gimbal_Motionless)
	{
		Gimbal_MotionLess_Control_Set(Gimbal_Config);
	}
	else if(Gimbal_Config->Gimbal_Mode == Gimbal_Spin)
	{
		//Gimbal_Motor_Control_Set_Final_Pitch(Gimbal_Config);
		 Gimbal_IMU_Control_NL_Set(Gimbal_Config);
		//Gimbal_Motor_Control_Set_Final_Pitch(Gimbal_Config);
	}

	
}


void Angle_Control_Limit(float *Angle_Set,float Max_Angle,float Min_Angle)
{
	if(*Angle_Set > Max_Angle)
		*Angle_Set = Max_Angle;
	else if(*Angle_Set < Min_Angle)
		*Angle_Set = Min_Angle;
}

float Angle_Limit(float Angle_Set)
{
//	  int32_t relative_ecd =  Now_Angle - Angle_Middle;
    if (Angle_Set >= 3.1415926)
    {
        Angle_Set -= 2*3.1415926;
    }
    else if (Angle_Set <= -3.1415926)
    {
        Angle_Set += 2*3.1415926;
    }
		return Angle_Set;
}

float Angle_TM_Set(float Now_Angle,float Angle_Middle)
{
	  float relative_ecd =  Now_Angle - Angle_Middle;
    if (relative_ecd > 3.1415926f)
    {
        relative_ecd -= 2*3.1415926f;
    }
    else if (relative_ecd < -3.1415926f)
    {
        relative_ecd += 2*3.1415926f;
    }
		return relative_ecd;
}

float Angle_Abs(float Angle_Set_To_Abs)
{
	if(Angle_Set_To_Abs >= 0)
	{
		return Angle_Set_To_Abs;
	}
	else if(Angle_Set_To_Abs < 0)
	{
		return -Angle_Set_To_Abs;
	}
	return Angle_Set_To_Abs;
}

void Angle_Data_Limit(float* Angle_Data)
{
	if(*Angle_Data > 3.14f)
	{
		*Angle_Data = -3.14f;
	}
	if(*Angle_Data < -3.14f)
	{
		*Angle_Data = 3.14f;
	}
}

float Pitch_Angle_Data_Add_Limit(float* Pitch_IMU_Angle,float Max_IMU_Angle,float Min_IMU_Angle)
{
  	if(*Pitch_IMU_Angle >= Max_IMU_Angle)
		{return 0;}
	else if(*Pitch_IMU_Angle <= Min_IMU_Angle)
	  {return 0;}
		 
}

float Pitch_Angle_Limit(float Pitch_IMU_Angle,float Max_IMU_Angle,float Min_IMU_Angle)
{
  	if(Pitch_IMU_Angle >= Max_IMU_Angle)
		{return 0;}
	else if(Pitch_IMU_Angle <= Min_IMU_Angle)
	  {return 0;}
		 
}


extern float INS_Angle[3];
extern float get_anglee[3],INS_gyro[3];

int Motor_Move_Check(Motor_Msg_t* Motor_Msg_Check)
{
	float Check_Angle,Check_Speed;
	static float Check_Angle_Last;
	Check_Angle = Motor_Msg_Check->angle;
	Check_Speed = Motor_Msg_Check->speed;
	if((Check_Angle == Check_Angle_Last) || (Check_Speed == 0))
	{
		Check_Angle_Last = Check_Angle;
		led_red_on();
			return 0;	
	}
	
	Check_Angle_Last = Check_Angle;
	led_red_off();
	return 1;
}	
int No_Move_Flag;
int Check,Check_Count;

int Test_Angle[2];

void Gimbal_Task(void *pvParameters)
{
	vTaskDelay(250);
	Gimbal_Init(&Gimbal);
	while(1)
	{
		Gimbal_Data_Update(&Gimbal.Gimbal_Yaw_Msg_t);
		
		Gimbal_Data_Update(&Gimbal.Gimbal_Pitch_Msg_t);
		
		Gimbal_IMU_Aspeed_Get(&Gimbal);
		
		Gimbal_Mode_Set(&Gimbal);	
		
		Gimbal_Mode_Change(&Gimbal);
		
		Gimbal_Mode_Config(&Gimbal);
				
		CAN2_Motor_Control(0x1FF,(int16_t)Gimbal.Gimbal_Motor_Current_Send[0],(int16_t)Gimbal.Gimbal_Motor_Current_Send[1],0,0);
		vTaskDelay(1);	
	}
}



Gimbal_Mode_t* Return_Gimbal_Mode_Add(void)
{
	return &Gimbal.Gimbal_Mode;
}

