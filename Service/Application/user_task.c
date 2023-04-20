/**
  ****************************(C) COPYRIGHT 2020 HRBUST_AIR****************************
  * @file    user_task.c
	* @brief   用户任务           
  * @author  JackyJuu , HRBUST_AIR_TEAM , website:www.airclub.tech
	* @Note 	 测试用
  * @version V1.0.0
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     11-18-2020      JackyJuu            Done
  ****************************(C) COPYRIGHT 2020 HRBUST_AIR****************************
	* @describe 用户任务
*/
#include "User_Task.h"
#include "led.h"
#include "can.h"
#include "pid.h"

#include "flash.h"
#include "key.h"

#include "user_tool.h"
#include "Judge_Data.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "usb_usart.h"

#include "imu_bsp.h"
#include "Judge_Graphic.h"

extern float motor_set;
float motor_test_set[2];
int kkk;

int key_readdd,Last_key_readdd;

extern Osker_Remote_Control_Struct Osker_Remote_Control_Data;

uint32_t Flash_Data[2];

float Get_Imu_Data[3];
//Judge_Graphic_Obj_t* Circle;

void UserTast(void *pvParameters)
{
	vTaskDelay(100);
//	Circle = Judge_Graphic_Circle_Create(800,800,50,10);
//	Judge_Graphic_Circle_Create(900,900,50,10);
//	Judge_Graphic_Circle_Create(1000,1000,50,10);
	while(1)
	{

//		oled_key();
//		oled_Osker_show();
//		Oled_Show_Tem();	
//		vTaskDelay(100);

		
//		Flash_Wordbyte_Write(0x081E1234,Flash_Data,2);
//		Usb_Write_Data("test-test \r\n");
		
		
		LED_GREEN_ON();
		vTaskDelay(500);
		LED_GREEN_OFF();
		vTaskDelay(500);
		
	}
}

float Osker_Motor_control_Data[4];

int Motor_Mode = 2,Last_Motor_Mode;

int get_mode = 1,get_pwm;

extern Osker_Remote_Control_Data_Key_t Osker_Remote_Control_Data_Key;
extern Osker_Remote_Control_Data_Key_t Osker_Remote_Control_Data_Key_Last;

Osker_Remote_Control_Data_Key_t Osker_Remote_Control_Data_Key_control_last;

struct PID dipan1pid, dipan2pid,dipan3pid,dipan4pid;
extern struct dipandianji dipan1,dipan2,dipan3,dipan4;

int set_speed;

extern int key_last_1,key_modeeee;		

void Osker_Task(void *pvParameters)
{
	key_modeeee = 1;
	set_speed = 0;
	while(1)
	{

		if((KEY_Scan() == 1) && (key_last_1 != 1))				
			{
					if(key_modeeee == 0)
					{
						key_modeeee = 1;
					}
					else if(key_modeeee == 1)
					{		
						key_modeeee = 0;
					}
			}
		if(key_modeeee == 0)
		{
				set_speed = 5000;
		}
		else if(key_modeeee == 1)
		{		
				set_speed = 0;
		}

	//	CAN1_Motor_Control(0x200,set_speed,0,0,0);

			
			key_last_1 = KEY_Scan();
		
//		Judge_Data_Send(&DJI_Judge,0x0201,102, Judge_Test_Send_Data,5);	
//		LED_RED_TOGGLE();

		vTaskDelay(2);
	}
}


