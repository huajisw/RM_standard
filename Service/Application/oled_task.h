#ifndef OLED_TASK_H
#define OLED_TASK_H
#include "stm32f4xx.h"
#include "oled_mode.h"
#include "Judge_Data.h"

typedef enum
{
	UP = 1,
	DOWM = 2,
	LEFT = 3,
	RIGHT = 4,
	IN = 5,
	NO_MOVE
}OLED_Key_Mode_t;

typedef enum
{
	Test_Oled_Mode = 1,
	Key_Oled_Mode = 2,
	DJI_Judge_Oled_Mode = 3,
	Hero_Shoot_Test_Oled_Mode = 4,
	DJI_Judge_Set = 5	
}OLED_Show_Mode_t;

typedef enum
{
	Oled_Display_On = 1,
	Oled_Display_Off = 2
}OLED_Display_Mode_t;




typedef struct
{
  int	Oled_Key_Adc_Read;
	OLED_Key_Mode_t OLED_Key_Mode;
	OLED_Key_Mode_t OLED_Key_Mode_Last;
	OLED_Show_Mode_t OLED_Show_Mode;	
	
	OLED_Display_Mode_t OLED_Display_Mode;
	
	Judge_Info_t* DJI_Judge_Mes;
	
}OLED_t;





void Oled_Task(void *pvParameters);

#endif


