/**
  ****************************(C) COPYRIGHT 2021 HRBUST_AIR****************************
  * @file    shoot_task.c
	* @brief   射击控制任务           
  * @author  JackyJuu , HRBUST_AIR_TEAM , website:www.airclub.tech
	* @Note 	 已经完成读取裁判系统热量闭环
	* @Question 	 摩擦轮PID可能需要调整，需要获取摩擦力执行曲线【4.9】
  * @version V1.0.0
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     4-9-2021     	 JackyJuu            Done
  ****************************(C) COPYRIGHT 2021 HRBUST_AIR****************************
	* @describe 射击控制任务
*/
#include <math.h>

#include "shoot_task.h"
#include "arm_math.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "key.h"

#include "can.h"
#include "pid.h"

Shoot_t Shoot;

#define Bullet_Basket_Set(Pwm_Duty)  TIM_SetCompare1(TIM4,(Pwm_Duty))
#define Shoot_Key_Read()  GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_0)

static TaskHandle_t ShootTask_Handler;

//float Shoot_Freq_To_Trigger_Motor_Speed(uint8_t Convert_Factor,uint8_t Freq)
//{
//		return Freq/Convert_Factor*60.0;
//}

void Shoot_Mode_Set(Shoot_t* Mode_Set)
{
		uint8_t Shoot_Mode_Switch = Mode_Set->Shoot_RC_Ctl_Data->rc.s1;
		uint16_t Shoot_Mode_Key = Mode_Set->Shoot_RC_Ctl_Data->key.v;
		uint8_t Shoot_Mouse_Key = Mode_Set->Shoot_RC_Ctl_Data->mouse.press_l;
	
		uint8_t Shoot_Start_Stop_Signal = (Shoot_Mode_Switch==RC_SW_UP&&Mode_Set->Last_Shoot_Mode_Switch!=RC_SW_UP)
																			||((Shoot_Mode_Key&SHOOT_START_STOP_KEY)&&!(Mode_Set->Last_Shoot_Mode_Key&SHOOT_START_STOP_KEY));
	
	
		if((Shoot_Mode_Key&BULLET_BASKET_OPEN_KEY)&&!(Mode_Set->Last_Shoot_Mode_Key&BULLET_BASKET_OPEN_KEY))
		{
					Mode_Set->Bullet_Basket_Mode = BASKET_OPEN;
		}
		else if((Shoot_Mode_Key&BULLET_BASKET_CLOSE_KEY)&&!(Mode_Set->Last_Shoot_Mode_Key&BULLET_BASKET_CLOSE_KEY))
		{
					Mode_Set->Bullet_Basket_Mode = BASKET_CLOSE;
		}
	
		//在停止射击状态下上拨,或者按下R键，进入开始射击状态
		if(Mode_Set->Shoot_Mode == SHOOT_STOP)
		{
				Laser_off();
				if(Shoot_Start_Stop_Signal)
				{
						if(Mode_Set->Shoot_Key == Mode_Set->Shoot_Key_On_Level)
								Mode_Set->Shoot_Mode = SHOOT_READY;
						else
								Mode_Set->Shoot_Mode = SHOOT_START;
				}
		}
		else 
		{
				Laser_on();
				//在除了停止射击之外的其他状态，再次上拨或者按下R键，进入停止射击状态
				if(Shoot_Start_Stop_Signal)
				{
						Mode_Set->Shoot_Mode = SHOOT_STOP;
				}
				//否则进行正常的模式判断
				else
				{
						/****************信号产生部分*****************/
						//统计按下鼠标左键和下拨时间
						if(Shoot_Mode_Switch==RC_SW_DOWN||Shoot_Mouse_Key)
						{
								if(Mode_Set->Shoot_Mode_Switch_Down_Time<SHOOT_MODE_SWITCH_DOWN_TIME_LIMIT)
										Mode_Set->Shoot_Mode_Switch_Down_Time++;
						}
						else 
								Mode_Set->Shoot_Mode_Switch_Down_Time=0;
						//长按鼠标左键或这一直处于下拨状态，产生连发信号
						uint8_t Shoot_Bullet_No_Break_Signal = Mode_Set->Shoot_Mode_Switch_Down_Time==SHOOT_MODE_SWITCH_DOWN_TIME_LIMIT;
						//点击鼠标左键，或者下拨一次，产生单发信号
						uint8_t Shoot_Bullet_Once_Signal = (Shoot_Mode_Switch==RC_SW_DOWN&&Mode_Set->Last_Shoot_Mode_Switch!=RC_SW_DOWN)
																				||(Shoot_Mouse_Key&&!Mode_Set->Last_Shoot_Mouse_Key);
						//每次检测到单发信号，需要发射的弹丸数目加一
						if(Shoot_Bullet_Once_Signal)
							Mode_Set->Need_Shoot_Count++;
						//每次检测到连发信号，需要发射的弹丸数目置一
						if(Shoot_Bullet_No_Break_Signal)
							Mode_Set->Need_Shoot_Count=1;
						
						//电机堵转信号
						uint8_t Trigger_Motor_Stall_Signal = 0;
						//如果电机速度小于一定值持续一段时间，说明电机堵转了
						if(fabs(Mode_Set->Trigger_Motor_Speed_Get)<Mode_Set->Trigger_Motor_Min_Speed
									&&Mode_Set->Trigger_Motor_Speed_Set!=0)
						{
								if(Mode_Set->Trigger_Motor_Low_Speed_Time<Mode_Set->Trigger_Motor_Low_Speed_Time_Limit)
										Mode_Set->Trigger_Motor_Low_Speed_Time++;
								else
										Trigger_Motor_Stall_Signal = 1;
						}
						else
						{
								Mode_Set->Trigger_Motor_Low_Speed_Time = 0;
						}
						/****************状态机转移部分*****************/
						
						if(Mode_Set->Shoot_Mode == SHOOT_START)
						{
								//在开始射击状态下，如果弹丸到达微动开关，那么进入准备射击状态
								if(Mode_Set->Shoot_Key == Mode_Set->Shoot_Key_On_Level)
										Mode_Set->Shoot_Mode = SHOOT_READY;
								//如果很长时间都没有加载上弹丸，判断电机是否堵转
								else if(Mode_Set->Shoot_Start_Time>Mode_Set->Shoot_Start_Time_Limit)
								{
										//如果电机堵转了，说明卡弹了，进入回拨状态
										if(Trigger_Motor_Stall_Signal)
												Mode_Set->Shoot_Mode = SHOOT_STALL;
										//否则表明没有弹丸了
//										else
//												Mode_Set->No_Bullet_Flag = 1;
								}
						}
						//在准备射击状态下，如果需要发射的弹丸数目大于0，进入发射状态
						else if(Mode_Set->Shoot_Mode == SHOOT_READY&&Mode_Set->Need_Shoot_Count)
						{
								Mode_Set->Shoot_Mode = SHOOT_BULLET;
						}
						//在卡弹状态下，如果回拨时间达到设定值，重新回到准备开始发射状态
						else if(Mode_Set->Shoot_Mode == SHOOT_STALL
									&&Mode_Set->Shoot_Stall_Time==Mode_Set->Shoot_Stall_Time_Limit)
						{
									Mode_Set->Shoot_Mode = SHOOT_START;
						}
						else if(Mode_Set->Shoot_Mode == SHOOT_BULLET)
						{
							//在发射弹丸状态，如果弹丸射出，进入开始发射状态，需要发射的弹丸数目减一
							if(Mode_Set->Shoot_Key!=Mode_Set->Shoot_Key_On_Level&&!Shoot_Bullet_No_Break_Signal)//&&Mode_Set->Last_Shoot_Key==SHOOT_KEY_ON)
							{
									Mode_Set->Shoot_Mode=SHOOT_START;
									Mode_Set->Need_Shoot_Count--;
							}
							//在发射弹丸状态下，如果长时间未发弹，检查电机是否卡住了
							if(Mode_Set->Shoot_Bullet_Time>Mode_Set->Shoot_Bullet_Time_Limit)
							{
									//如果电机卡住了，进入回拨模式
									if(Trigger_Motor_Stall_Signal)
											Mode_Set->Shoot_Mode=SHOOT_STALL;
									//否则表明没有弹丸了
//									else
//											Mode_Set->No_Bullet_Flag = 1;
							}
						}
				}
		}
			
		Mode_Set->Last_Shoot_Mode_Switch = Shoot_Mode_Switch;
		Mode_Set->Last_Shoot_Key = Mode_Set->Shoot_Key;
		Mode_Set->Last_Shoot_Mode_Key = Shoot_Mode_Key;
		Mode_Set->Last_Shoot_Mouse_Key = Shoot_Mouse_Key;
}

void Shoot_Mode_Change(Shoot_t* Mode_Change)
{
		//如果处于发射弹丸状态，统计发射时间，从其他状态切入发射弹丸状态，发射时间清零
		if(Mode_Change->Shoot_Mode==SHOOT_BULLET)
		{
				if(Mode_Change->Last_Shoot_Mode!=SHOOT_BULLET)
						Mode_Change->Shoot_Bullet_Time=0;
				else
						Mode_Change->Shoot_Bullet_Time++;
		}
		//如果处于卡弹回拨状态，统计卡弹回拨时间，从其他状态切入卡弹回拨状态，卡弹回拨时间清零
		if(Mode_Change->Shoot_Mode==SHOOT_STALL)
		{
				if(Mode_Change->Last_Shoot_Mode!=SHOOT_STALL)
						Mode_Change->Shoot_Stall_Time=0;
				else
						Mode_Change->Shoot_Stall_Time++;
		}
		//如果处于开始射击状态，统计开始射击状态的时间，从其他状态切入开始射击状态，时间清零
		if(Mode_Change->Shoot_Mode==SHOOT_START)
		{
				if(Mode_Change->Last_Shoot_Mode!=SHOOT_START)
						Mode_Change->Shoot_Start_Time=0;
				else
						Mode_Change->Shoot_Start_Time++;
		}
		//从其他模式切入停止射击模式，需要发射的数目置零
		if(Mode_Change->Shoot_Mode==SHOOT_STOP&&Mode_Change->Last_Shoot_Mode!=SHOOT_STOP)
		{
				Mode_Change->Need_Shoot_Count = 0;
		}
		Mode_Change->Last_Shoot_Mode = Mode_Change->Shoot_Mode;
}


void Shoot_Pid_Calc(Shoot_t* Pid_Calc)
{
		if(Pid_Calc->Shoot_Mode!=SHOOT_STOP)
		{
				
				Pid_Calc->Trigger_Motor_Current_Send = pid_calc(&Pid_Calc->Trigger_Motor_Pid,Pid_Calc->Trigger_Motor_Speed_Get,Pid_Calc->Trigger_Motor_Speed_Set);
		}
		else
		{
//				Pid_Calc->Fric_Motor_Current_Send[0] = 0;
//				Pid_Calc->Fric_Motor_Current_Send[1] = 0;
				Pid_Calc->Trigger_Motor_Current_Send = 0;
		}
		int8_t Factor = Pid_Calc->Fric_Reverse_Flag?-1:1;
		Pid_Calc->Fric_Motor_Current_Send[0] = pid_calc(&Pid_Calc->Fric_Motor_Pid[0],Pid_Calc->Fric_Motor_Speed_Get[0],Factor*-Pid_Calc->Fric_Motor_Speed_Set);
		Pid_Calc->Fric_Motor_Current_Send[1] = pid_calc(&Pid_Calc->Fric_Motor_Pid[1],Pid_Calc->Fric_Motor_Speed_Get[1],Factor*Pid_Calc->Fric_Motor_Speed_Set);
		
}

void Shoot_Init(Shoot_t* Data_Init)
{
		//电机数据获取
		Data_Init->Fric_Motor_Msg_Get[0] = Get_DJI_Motor_Data(CAN1_RX,FRIC_MOTOR_LEFT_ID); 
		Data_Init->Fric_Motor_Msg_Get[1] = Get_DJI_Motor_Data(CAN1_RX,FRIC_MOTOR_RIGHT_ID); 
		Data_Init->Trigger_Motor_Msg_Get = Get_DJI_Motor_Data(CAN1_RX,TRIGGER_MOTOR_ID);
		
		Data_Init->Shoot_Mode = SHOOT_STOP;
		//遥控器数据获取
		Data_Init->Shoot_RC_Ctl_Data = Get_DJI_RC_Data_Address();
		//裁判系统数据获取
		Data_Init->Shoot_Judge_Info_Get = Get_Judge_Info();
		//初始化PID
		pid_init(&Data_Init->Fric_Motor_Pid[0],FRIC_MOTOR_LEFT_KP,FRIC_MOTOR_LEFT_KI,FRIC_MOTOR_LEFT_KD,FRIC_MOTOR_LEFT_MAXOUT,FRIC_MOTOR_LEFT_IMAXOUT,1);	
		pid_init(&Data_Init->Fric_Motor_Pid[1],FRIC_MOTOR_RIGHT_KP,FRIC_MOTOR_RIGHT_KI,FRIC_MOTOR_RIGHT_KD,FRIC_MOTOR_RIGHT_MAXOUT,FRIC_MOTOR_RIGHT_IMAXOUT,1);	
		pid_init(&Data_Init->Trigger_Motor_Pid,TRIGGER_MOTOR_KP,TRIGGER_MOTOR_KI,TRIGGER_MOTOR_KD,TRIGGER_MOTOR_MAXOUT,TRIGGER_MOTOR_IMAXOUT,2);	
		
		Data_Init->Bullets_Per_Rotation = BULLETS_PER_ROTATION;
		Data_Init->Fric_Wheel_Diameter = FRIC_WHEEL_DIAMETER;
		Data_Init->Shoot_Bullet_Time_Limit = SHOOT_BULLET_TIME_LIMIT;
		Data_Init->Shoot_Start_Time_Limit = SHOOT_BULLET_TIME_LIMIT;
		Data_Init->Trigger_Motor_Min_Speed = TRIGGER_MOTOR_MIN_SPEED;
		Data_Init->Load_Bullet_Speed = LOAD_BULLET_SPEED;
		Data_Init->Unload_Bullet_Speed = UNLOAD_BULLET_SPEED;
		Data_Init->Shoot_Stall_Time_Limit = SHOOT_STALL_TIME_LIMIT;
		Data_Init->Trigger_Motor_Low_Speed_Time_Limit = TRIGGER_MOTOR_LOW_SPEED_TIME_LIMIT;
		Data_Init->Default_Shoot_Freq_Limit = DEFAULT_SHOOT_FREQ_LIMIT;
		Data_Init->Default_Shoot_Speed_Limit = DEFAULT_SHOOT_SPEED_LIMIT;
		Data_Init->Shoot_Key_On_Level = SHOOT_KEY_ON;
		Data_Init->Fric_Reverse_Flag = 1;
		
		Data_Init->Shoot_Line_1m = Judge_Graphic_Line_Create(0,540-80,1920,540-80,2);
		Judge_Graphic_Obj_Set_Color(Data_Init->Shoot_Line_1m,COLOR_ORANGE);
		Data_Init->Shoot_Line_3m = Judge_Graphic_Line_Create(0,540-50,1920,540-50,2);
		Judge_Graphic_Obj_Set_Color(Data_Init->Shoot_Line_3m,COLOR_YELLOW);
		Data_Init->Shoot_Line_3m = Judge_Graphic_Line_Create(960-10,0,960-10,1080,2);
		Judge_Graphic_Obj_Set_Color(Data_Init->Shoot_Line_ver,COLOR_GREEN);
		//Data_Init->Shoot_Line_5m = Judge_Graphic_Line_Create(0,540-50,1920,540-50,2);
		//Data_Init->Bullet_Basket_Graphic = Judge_Graphic_Circle_Create(1800,600,5,5);
		//Data_Init->Shoot_Stall_Graphic = Judge_Graphic_Circle_Create(1800,650,5,5);
}

uint8_t Get_Shoot_Freq_From_Judge_System(Shoot_t* Get_Shoot_Freq)
{
		if(Is_Judge_Online())
		{
				if(Get_Shoot_Freq->Judge_Shoot_Heat_Percent >= 0.95)
				{
						return 0;
				}
				if((Get_Shoot_Freq->Judge_Shoot_Heat_Percent >= 0) && (Get_Shoot_Freq->Judge_Shoot_Heat_Percent < 0.6))
				{
						if(Get_Shoot_Freq->Judge_Shoot_Cooling_Limit <=50)
							return 3;
						else if(Get_Shoot_Freq->Judge_Shoot_Cooling_Limit <=100)
							return 7;
						else if(Get_Shoot_Freq->Judge_Shoot_Cooling_Limit <=150)
							return 10;
						else if(Get_Shoot_Freq->Judge_Shoot_Cooling_Limit <=280)
							return 18;
						else if(Get_Shoot_Freq->Judge_Shoot_Cooling_Limit <=400)
							return 26;	
				}
				else if((Get_Shoot_Freq->Judge_Shoot_Heat_Percent >= 0.6) 
						 && (Get_Shoot_Freq->Judge_Shoot_Heat_Percent <= (1.0f-Get_Shoot_Freq->Judge_Shoot_Cool_Percent)))
				{
						if(Get_Shoot_Freq->Judge_Shoot_Cooling_Limit <=50)
							return 1;
						else if(Get_Shoot_Freq->Judge_Shoot_Cooling_Limit <=100)
							return 2;
						else if(Get_Shoot_Freq->Judge_Shoot_Cooling_Limit <=150)
							return 3;
						else if(Get_Shoot_Freq->Judge_Shoot_Cooling_Limit <=280)
							return 4;
						else if(Get_Shoot_Freq->Judge_Shoot_Cooling_Limit <=400)
							return 5;
				}
				else if(Get_Shoot_Freq->Judge_Shoot_Heat_Percent > (1.0f-Get_Shoot_Freq->Judge_Shoot_Cool_Percent))
				{
						return (Get_Shoot_Freq->Judge_Shoot_Cooling_Limit / 10);
				}
		}
		return Get_Shoot_Freq->Default_Shoot_Freq_Limit;
}

float Get_Fric_Speed_From_Judge_System(Shoot_t* Get_Fric_Speed)
{
		if(Is_Judge_Online())
		{
				if(Get_Fric_Speed->Judge_Shoot_Speed_Limit <= 15)
						return Get_Fric_Speed->Judge_Shoot_Speed_Limit;
				else if(Get_Fric_Speed->Judge_Shoot_Speed_Limit <= 18)
						return Get_Fric_Speed->Judge_Shoot_Speed_Limit*0.95;
				else if(Get_Fric_Speed->Judge_Shoot_Speed_Limit <= 22)
						return Get_Fric_Speed->Judge_Shoot_Speed_Limit*0.85;
				else if(Get_Fric_Speed->Judge_Shoot_Speed_Limit <= 30)
						return Get_Fric_Speed->Judge_Shoot_Speed_Limit*0.75;
				else
						return Get_Fric_Speed->Default_Shoot_Speed_Limit;
		}
		return Get_Fric_Speed->Default_Shoot_Speed_Limit;
}


void Shoot_Control_Data_Set(Shoot_t* Control_Data_Set)
{
		if(Control_Data_Set->Shoot_Mode == SHOOT_START)
		{
				Control_Data_Set->Trigger_Motor_Speed_Set = Control_Data_Set->Load_Bullet_Speed;
		}
		else if(Control_Data_Set->Shoot_Mode == SHOOT_READY)
		{
				Control_Data_Set->Trigger_Motor_Speed_Set = 0;
		}
		else if(Control_Data_Set->Shoot_Mode == SHOOT_BULLET)
		{
				uint8_t Freq_Set = Get_Shoot_Freq_From_Judge_System(Control_Data_Set);
				//给射频加上正负号，与加载弹丸速度的正负号一致
				if(Control_Data_Set->Load_Bullet_Speed>0)
						Control_Data_Set->Trigger_Motor_Speed_Set = Freq_Set;
				else
						Control_Data_Set->Trigger_Motor_Speed_Set = -Freq_Set;
		}
		else if(Control_Data_Set->Shoot_Mode == SHOOT_STALL)
		{
				Control_Data_Set->Trigger_Motor_Speed_Set = Control_Data_Set->Unload_Bullet_Speed;
		}
		
		if(Control_Data_Set->Shoot_Mode != SHOOT_STOP)
		{
				Control_Data_Set->Fric_Motor_Speed_Set = Get_Fric_Speed_From_Judge_System(Control_Data_Set);
		}
		else
		{
				Control_Data_Set->Fric_Motor_Speed_Set = 0;
		}
		
		//如果处在测试模式，那么正常的模式控制不起作用
		if(Control_Data_Set->Bullet_Basket_Mode==BASKET_OPEN)
		{
				if(Control_Data_Set->Bullet_Basket_Reverse)
					Bullet_Basket_Set(SERVO_MAX_DUTY);
				else
					Bullet_Basket_Set(SERVO_MIN_DUTY);
		}
		else if(Control_Data_Set->Bullet_Basket_Mode==BASKET_CLOSE)
		{
				Bullet_Basket_Set(BULLET_BASKET_CLOSE_DUTY);
		}
}

void Shoot_Data_Update(Shoot_t* Data_Update)
{
		//电机数据更新
		Data_Update->Fric_Motor_Speed_Get[0]=(float)Data_Update->Fric_Motor_Msg_Get[0]->speed/60*PI*Data_Update->Fric_Wheel_Diameter;
		Data_Update->Fric_Motor_Speed_Get[1]=(float)Data_Update->Fric_Motor_Msg_Get[1]->speed/60*PI*Data_Update->Fric_Wheel_Diameter;
		Data_Update->Trigger_Motor_Speed_Get=(float)Data_Update->Trigger_Motor_Msg_Get->speed/60/36*Data_Update->Bullets_Per_Rotation;
		Data_Update->Shoot_Key = Shoot_Key_Read();
		//裁判系统数据更新
		Data_Update->Judge_Robot_ID = Data_Update->Shoot_Judge_Info_Get->Judge_game_robot_status.robot_id;
		if(Data_Update->Judge_Robot_ID!=0)
		{
				//判断是否是英雄，如果是英雄使用42mm发射机构数据
				if(Data_Update->Judge_Robot_ID == Red_Hero||Data_Update->Judge_Robot_ID == Blue_Hero)
				{
						Data_Update->Judge_Shoot_Cooling_Limit = Data_Update->Shoot_Judge_Info_Get->Judge_game_robot_status.shooter_id1_42mm_cooling_limit;
						Data_Update->Judge_Shoot_Cooling_Rate = Data_Update->Shoot_Judge_Info_Get->Judge_game_robot_status.shooter_id1_42mm_cooling_rate;
						Data_Update->Judge_Shoot_Cooling_Heat = Data_Update->Shoot_Judge_Info_Get->Judge_power_heat_data.shooter_id1_42mm_cooling_heat;
						Data_Update->Judge_Shoot_Speed_Limit = Data_Update->Shoot_Judge_Info_Get->Judge_game_robot_status.shooter_id1_42mm_speed_limit;
				}
				else
				{
						Data_Update->Judge_Shoot_Cooling_Limit = Data_Update->Shoot_Judge_Info_Get->Judge_game_robot_status.shooter_id1_17mm_cooling_limit;
						Data_Update->Judge_Shoot_Cooling_Rate = Data_Update->Shoot_Judge_Info_Get->Judge_game_robot_status.shooter_id1_17mm_cooling_rate;
						Data_Update->Judge_Shoot_Cooling_Heat = Data_Update->Shoot_Judge_Info_Get->Judge_power_heat_data.shooter_id1_17mm_cooling_heat;
						Data_Update->Judge_Shoot_Speed_Limit = Data_Update->Shoot_Judge_Info_Get->Judge_game_robot_status.shooter_id1_17mm_speed_limit;
				}
				Data_Update->Judge_Shoot_Heat_Percent = (float)Data_Update->Judge_Shoot_Cooling_Heat/Data_Update->Judge_Shoot_Cooling_Limit;
				Data_Update->Judge_Shoot_Cool_Percent = (float)Data_Update->Judge_Shoot_Cooling_Rate/Data_Update->Judge_Shoot_Cooling_Limit;
		}
		
}

void Shoot_Draw_Graphic(Shoot_t* Draw_Graphic)
{
		if(Draw_Graphic->Bullet_Basket_Mode == BASKET_CLOSE)
		{
				Judge_Graphic_Obj_Set_Color(Draw_Graphic->Bullet_Basket_Graphic,COLOR_GREEN);
		}
		else if(Draw_Graphic->Bullet_Basket_Mode == BASKET_OPEN)
		{
				Judge_Graphic_Obj_Set_Color(Draw_Graphic->Bullet_Basket_Graphic,COLOR_ORANGE);
		}
		
		if(Draw_Graphic->Shoot_Mode==SHOOT_STALL)
		{
				Judge_Graphic_Obj_Set_Color(Draw_Graphic->Shoot_Stall_Graphic,COLOR_ORANGE);
		}
		else
		{
				Judge_Graphic_Obj_Set_Color(Draw_Graphic->Shoot_Stall_Graphic,COLOR_GREEN);
		}
}

void Shoot_Task(void *pvParameters)
{
	vTaskDelay(100);
	Shoot_Init(&Shoot);

	while(1)
	{

		Shoot_Mode_Set(&Shoot);
		Shoot_Mode_Change(&Shoot);
		//获取发射状态
		Shoot_Data_Update(&Shoot);
		//裁判系统状态更新
		Shoot_Control_Data_Set(&Shoot);
		
		Shoot_Pid_Calc(&Shoot);
		//Shoot_Draw_Graphic(&Shoot);
		CAN1_Motor_Control(SHOOT_MOTOR_ALL_ID,(int16_t)Shoot.Fric_Motor_Current_Send[0],
																					(int16_t)Shoot.Fric_Motor_Current_Send[1],
																					(int16_t)Shoot.Trigger_Motor_Current_Send,0);

		vTaskDelay(1);		
	}
}
