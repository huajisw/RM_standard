/**
  ****************************(C) COPYRIGHT 2020 HRBUST_AIR****************************
  * @file    oled_task.c
	* @brief   OLED显示屏任务           
  * @author  JackyJuu , HRBUST_AIR_TEAM , website:www.airclub.tech
	* @Note 	 包括显示，按键读取
  * @version V1.0.0
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     11-18-2020      JackyJuu            Done
  ****************************(C) COPYRIGHT 2020 HRBUST_AIR****************************
	* @describe OLED显示屏任务
*/
#include "oled_task.h"
#include "oled_mode.h"
#include "oled_list.h"
#include "oled.h"

#include "adc.h"

#include "Judge_Data.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

extern Judge_Info_t DJI_Judge_Mes;

OLED_t OLED;

int Key_x = 0,Key_y = 0;


static void Oled_Key_Read(OLED_t* OLED_Key_Read);
static void Oled_Key_Show(OLED_t* OLED_Key_Show);

static void Oled_Mode_Key_Location_Change(OLED_t* OLED_Key_Mode_Change);

static void Oled_Robot_State_Show(OLED_t* OLED_Judge_Mes);

static void Oled_Mode_Key_Change(OLED_t* OLED_Key_Mode_Change);
static void Oled_Mode_Set(OLED_t* OLED_Mode);

extern float motor_set[2];



void Oled_Task(void *pvParameters)
{
	OLED.DJI_Judge_Mes = get_Judge_Mes_Add();
	OLED.OLED_Display_Mode = Oled_Display_On;
	OLED.OLED_Show_Mode = DJI_Judge_Set;
	while(1)
	{
		
		portDISABLE_INTERRUPTS();
		
//		oled_display_off();
		
//		oled_refresh_gram();
	
	
	
	
	
//		oled_clear(Pen_Clear);
// 		Oled_Key_Read(&OLED);
//		Oled_Mode_Key_Location_Change(&OLED);
//		oled_Show_Location(64 + Key_x,32 + Key_y);
		//oled_refresh_gram();	
		Oled_Key_Read(&OLED);
		Oled_Mode_Key_Change(&OLED);
		
		
		Oled_Mode_Set(&OLED);
		
		
		
//		oled_printf(3,1,"speed_set");	
//		oled_shownum(2,1,	motor_set,0,4);	
//		Oled_Key_Show(&OLED);

		portENABLE_INTERRUPTS();		
		
		vTaskDelay(50);
	}
}


static void Oled_Shoot_Show(OLED_t* OLED_Key_Show)
{
	oled_clear(Pen_Clear);

	oled_printf(2,1,"123123");		
	oled_shownum(1,1,123123,0,4);
	oled_printf(3,1,"Shoot3");		
	oled_shownum(4,1,123123,0,4);
	
//	oled_printf(1,1,"Shoot1_2");		
//	oled_shownum(2,1,	(int)motor_set[0],0,4);
//	oled_printf(3,1,"Shoot3");		
//	oled_shownum(4,1,	(int)motor_set[1],0,4);
	
	oled_refresh_gram();
}

static void Oled_Key_Read(OLED_t* OLED_Key_Read)
{
		OLED_Key_Read->Oled_Key_Adc_Read=Get_KEY_ADC();
		if(OLED_Key_Read->Oled_Key_Adc_Read<10&&OLED_Key_Read->Oled_Key_Adc_Read>=0)
		{
			OLED_Key_Read->OLED_Key_Mode = LEFT;	
		}
		else if(OLED_Key_Read->Oled_Key_Adc_Read>2600&&OLED_Key_Read->Oled_Key_Adc_Read<2900)
		{
			OLED_Key_Read->OLED_Key_Mode = IN;		
		}
		else if(OLED_Key_Read->Oled_Key_Adc_Read>900&&OLED_Key_Read->Oled_Key_Adc_Read<1200)
		{
			OLED_Key_Read->OLED_Key_Mode = UP;
		}
		else if(OLED_Key_Read->Oled_Key_Adc_Read>1800&&OLED_Key_Read->Oled_Key_Adc_Read<2100)
		{
			OLED_Key_Read->OLED_Key_Mode = RIGHT;
		}
		else if(OLED_Key_Read->Oled_Key_Adc_Read>2900&&OLED_Key_Read->Oled_Key_Adc_Read<3200)
		{
			OLED_Key_Read->OLED_Key_Mode = DOWM;
		}
		else
		{
			OLED_Key_Read->OLED_Key_Mode = NO_MOVE;
		}
}

static void Oled_Key_Show(OLED_t* OLED_Key_Show)
{
	oled_clear(Pen_Clear);
	switch(OLED_Key_Show->OLED_Key_Mode)
	{
		case UP:
			oled_printf(3,1,"up");		
		break;
		case DOWM:
			oled_printf(3,1,"down");		
		break;
		case LEFT:
			oled_printf(3,1,"left");			
		break;
		case RIGHT:
			oled_printf(3,1,"right");		
		break;
		case IN:
			oled_printf(3,1,"in");		
		break;
		case NO_MOVE:
			oled_printf(3,1,"no move");			
		break;	
	}
	oled_shownum(2,1,	OLED_Key_Show->Oled_Key_Adc_Read,0,4);
	oled_refresh_gram();
}

static void Oled_Robot_State_Show(OLED_t* OLED_Judge_Mes)
{
	oled_clear(Pen_Clear);	
	
	oled_printf(1,1,"Robot-ID:");
	oled_shownum(1,10,OLED_Judge_Mes->DJI_Judge_Mes->Judge_game_robot_status.robot_id,0,3);
	OLED_Judge_Mes->DJI_Judge_Mes->Judge_Robot_ID = OLED_Judge_Mes->DJI_Judge_Mes->Judge_game_robot_status.robot_id;
	switch(OLED_Judge_Mes->DJI_Judge_Mes->Judge_Robot_ID)
	{
		case Red_Hero:
			oled_printf(2,1,"Red_Hero");			
		break;	
		case Red_Engineer:
			oled_printf(2,1,"Red_Engineer");			
		break;
		case Red_Standard_3:
			oled_printf(2,1,"Red_Standard_3");			
		break;
		case Red_Standard_4:
			oled_printf(2,1,"Red_Standard_4");			
		break;
		case Red_Standard_5:
			oled_printf(2,1,"Red_Standard_5");			
		break;
		case Red_Aerial:
			oled_printf(2,1,"Red_Aerial");			
		break;
		case Red_Sentry:
			oled_printf(2,1,"Red_Sentry");			
		break;
		case Red_DartLaunch:
			oled_printf(2,1,"Red_DartLaunch");			
		break;
		case Red_Radar:
			oled_printf(2,1,"Red_Radar");			
		break;
		case Blue_Hero:
			oled_printf(2,1,"Blue_Hero");			
		break;
		case Blue_Engineer:
			oled_printf(2,1,"Blue_Engineer");			
		break;
		case Blue_Standard_3:
			oled_printf(2,1,"Blue_Standard_3");			
		break;
		case Blue_Standard_4:
			oled_printf(2,1,"Blue_Standard_4");			
		break;
		case Blue_Standard_5:
			oled_printf(2,1,"Blue_Standard_5");			
		break;
		case Blue_Aerial:
			oled_printf(2,1,"Blue_Aerial");			
		break;
		case Blue_Sentry:
			oled_printf(2,1,"Blue_Sentry");			
		break;
		case Blue_DartLaunch:
			oled_printf(2,1,"Blue_DartLaunch");			
		break;
		case Blue_Radar:
			oled_printf(2,1,"Blue_Radar");			
		break;
	}
//	oled_printf(2,1,"Red_Standard_4:");

	
	
	oled_printf(3,1,"HP:   /   /   J");	
	oled_shownum(3,4,OLED_Judge_Mes->DJI_Judge_Mes->Judge_game_robot_status.remain_HP,0,3);
	oled_shownum(3,8,OLED_Judge_Mes->DJI_Judge_Mes->Judge_game_robot_status.max_HP,0,3);
	oled_shownum(3,12,OLED_Judge_Mes->DJI_Judge_Mes->Judge_power_heat_data.chassis_power_buffer,0,3);
	
//	oled_shownum(4,4,OLED_Judge_Mes->DJI_Judge_Mes->Judge_game_robot_status.remain_HP,0,3);
	oled_printf(4,1,"Chassis:   /    ");	
	oled_shownum(4,9,OLED_Judge_Mes->DJI_Judge_Mes->Judge_power_heat_data.chassis_power,0,3);
	oled_shownum(4,13,OLED_Judge_Mes->DJI_Judge_Mes->Judge_game_robot_status.chassis_power_limit,0,3);
	oled_refresh_gram();
	
}

static void Oled_Mode_Key_Location_Change(OLED_t* OLED_Key_Mode_Change)
{
	if(OLED_Key_Mode_Change->OLED_Key_Mode == LEFT)
	{
		Key_x = Key_x - 1;
	}
	else if(OLED_Key_Mode_Change->OLED_Key_Mode == RIGHT)
	{
		Key_x = Key_x + 1;
	}
	else if(OLED_Key_Mode_Change->OLED_Key_Mode == UP)
	{
		Key_y = Key_y - 1;
	}
	else if(OLED_Key_Mode_Change->OLED_Key_Mode == IN)
	{
		Key_y = 0;
		Key_x = 0;
		
	}
	else if(OLED_Key_Mode_Change->OLED_Key_Mode == DOWM)
	{
		Key_y = Key_y + 1;
 	}	
			OLED.OLED_Key_Mode_Last = OLED.OLED_Key_Mode;	
}

static void Oled_Mode_Key_Change(OLED_t* OLED_Key_Mode_Change)
{
	if((OLED_Key_Mode_Change->OLED_Key_Mode == LEFT) && (OLED_Key_Mode_Change->OLED_Key_Mode_Last != LEFT))
	{
		OLED_Key_Mode_Change->OLED_Show_Mode = DJI_Judge_Set;
	}
	else if((OLED_Key_Mode_Change->OLED_Key_Mode == RIGHT) && (OLED_Key_Mode_Change->OLED_Key_Mode_Last != RIGHT))
	{
		OLED_Key_Mode_Change->OLED_Show_Mode = DJI_Judge_Oled_Mode;
	}
	else if((OLED_Key_Mode_Change->OLED_Key_Mode == UP) && (OLED_Key_Mode_Change->OLED_Key_Mode_Last != UP))
	{
		OLED_Key_Mode_Change->OLED_Show_Mode = Test_Oled_Mode;
	}
	else if((OLED_Key_Mode_Change->OLED_Key_Mode == IN) && (OLED_Key_Mode_Change->OLED_Key_Mode_Last != IN))
	{
		OLED_Key_Mode_Change->OLED_Show_Mode = Hero_Shoot_Test_Oled_Mode;
	}
	else if((OLED_Key_Mode_Change->OLED_Key_Mode == DOWM) && (OLED_Key_Mode_Change->OLED_Key_Mode_Last != DOWM))
	{
		if(OLED_Key_Mode_Change->OLED_Display_Mode == Oled_Display_On)
		{
			oled_display_off();
			OLED_Key_Mode_Change->OLED_Display_Mode = Oled_Display_Off;
			
		}
		else if(OLED_Key_Mode_Change->OLED_Display_Mode == Oled_Display_Off)
		{
			oled_display_on();
			OLED_Key_Mode_Change->OLED_Display_Mode = Oled_Display_On;
		}
		
//		oled_display_on();
//		OLED_Key_Mode_Change->OLED_Show_Mode = Test_Oled_Mode;
		
	}	
			OLED.OLED_Key_Mode_Last = OLED.OLED_Key_Mode;	
}

extern int Location_x[10],Location_y[10];
extern int Data_Width[10],Data_Sizw[10];
extern int Judge_Num_Set;
void Judge_Set_Mode(void)
{
	oled_clear(Pen_Clear);
	oled_printf(1,1,"Set_Num:");
	oled_shownum(1,9,Judge_Num_Set,0,2);
	oled_printf(2,1,"x:");
	oled_shownum(2,3,Location_x[Judge_Num_Set],0,4);
	oled_printf(2,10,"y:");
	oled_shownum(2,12,Location_y[Judge_Num_Set],0,4);
	oled_printf(3,1,"Width:");
	oled_shownum(3,8,Data_Width[Judge_Num_Set],0,2);
	oled_printf(3,11,"Size:");
	oled_shownum(3,16,Data_Sizw[Judge_Num_Set],0,2);

//	oled_LOGO();
	oled_refresh_gram();
}

static void Oled_Mode_Set(OLED_t* OLED_Mode)
{
	switch(OLED_Mode->OLED_Show_Mode)
	{
		case DJI_Judge_Oled_Mode:
			Oled_Robot_State_Show(&OLED);		
		break;
		case Key_Oled_Mode:
			Oled_Key_Show(OLED_Mode);		
		break;
		case Test_Oled_Mode:
			oled_test();		
		break;
		case Hero_Shoot_Test_Oled_Mode:
			Oled_Shoot_Show(OLED_Mode);
		break;
		case DJI_Judge_Set:
			Judge_Set_Mode();
		break;
		
		

	}
//	switch(OLED_Mode->OLED_Display_Mode)
//	{
//		case Oled_Display_On:
//			
//		break;
//		case Oled_Display_Off:
//			Oled_Key_Show(OLED_Mode);		
//		break;
//	}
}


