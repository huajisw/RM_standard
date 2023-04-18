#include "oled_list.h"
#include "oled_mode.h"
#include "oled.h"

#include "adc.h"

#include "Judge_Data.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


uint8_t* Show_Word;

extern float yaw_angle_change,pitch_angle_change;

void oled_Show_Location(int x,int y)
{
	oled_drawpoint(x-1,y-1,Pen_Write);
	oled_drawpoint(x-1,y,Pen_Write);
	oled_drawpoint(x-1,y+1,Pen_Write);
	oled_drawpoint(x,y-1,Pen_Write);
	oled_drawpoint(x,y,Pen_Write);
	oled_drawpoint(x,y+1,Pen_Write);
	oled_drawpoint(x+1,y-1,Pen_Write);
	oled_drawpoint(x+1,y,Pen_Write);
	oled_drawpoint(x+1,y+1,Pen_Write);
}

void Oled_Shoot_Show(void)
{
	oled_clear(Pen_Clear);

	oled_printf(0,1,"123123");		
	oled_shownum(2,1,123123,0,4);
	oled_printf(3,1,"Shoot3");		
	oled_shownum(4,1,123123,0,4);
	
	oled_refresh_gram();
}



char* Show_Data;
int key_last_1,key_modeeee = 0;

extern float get_anglee[3];//获取的角度数据

void Oled_List_Task(void *pvParameters)
{
	vTaskDelay(100);
	Show_Data = "dada";
	while(1)
	{

		if((KEY_Scan() == 1) && (key_last_1 != 1))				
			{
					if(key_modeeee == 0)
					{

						Show_Data = "aa";
						key_modeeee = 1;
					}
					else if(key_modeeee == 1)
					{
						Show_Data = "bbcc";				
						key_modeeee = 0;
					}
			}	
			key_last_1 = KEY_Scan();
			portDISABLE_INTERRUPTS();
		
	
//			oled_clear(Pen_Clear);
//			oled_printf(1,1,"IMU");	
////			oled_number(1,7,&pitch_angle_change,0,4);	
//			oled_printf(2,1,"Yaw:");	
//			oled_small_number(2,6,get_anglee,3,2);
//			oled_printf(3,1,"Pitch:");	
//			oled_small_number(3,6,get_anglee+1,3,2);
//			oled_printf(4,1,"Roll:");				
//			oled_small_number(4,6,get_anglee+2,3,2);
//			
				oled_refresh_gram();
//			Oled_Shoot_Show();
			
	portENABLE_INTERRUPTS();		
			
//		oled_Show_Location(64 + yaw_angle_change * 64/25,32 + pitch_angle_change * 32/12);

			
//	oled_drawpoint(64,32,Pen_Write);			
			
//	oled_printf(1,1,"PITCH:");	
//	oled_number(1,7,&pitch_angle_change,0,4);			
//			
//	oled_printf(2,1,"YAW:");	
//	oled_number(2,7,&yaw_angle_change,0,4);			

			
			
//	oled_Drwaline(0);		
//	oled_Drwaline(0);		

//	oled_printf(1,1,Show_Data);		
////	oled_shownum(2,1,123123,0,4);
////	oled_printf(3,1,"Shoot3");	
//	oled_printf_choice(4,1,Show_Data);			
//	oled_shownum(0,1,123123,0,4);
//	oled_shownum(1,1,123123,0,4);
//	oled_shownum(2,1,123123,0,4);
//	oled_shownum(3,1,123123,0,4);
//	oled_shownum(4,1,123123,0,4);
	

			
		vTaskDelay(100);			
			
	}
}

