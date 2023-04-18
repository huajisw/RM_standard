#ifndef _JUDGE_TASK_H_
#define _JUDGE_TASK_H_

#include "main.h"

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

#include "gimbal_task.h"
#include "chassis_task.h"
#include "App_Set.h"

#define Judge_Set_Update_Time_Set 100
#define Judge_Super_C_Turn_Red 30



#define Judge_Gimbal_Show_Floor 0
#define Judge_Chassis_Show_Floor 1
#define Judge_Vision_Floor 2
#define Judge_Super_C_Show_Floor 3


#define Judge_First_Floor 0
#define Judge_Second_Floor 1
#define Judge_Third_Floor 2
#define Judge_Forth_Floor 3
#define Judge_Fifth_Floor 4
#define Judge_Sixth_Floor 5
#define Judge_Seventh_Floor 6
#define Judge_Eighth_Floor 7
#define Judge_Ninth_Floor 8
#define Judge_Tenth_Floor 9

																	


typedef struct
{
	uint8_t* Super_C_Name;
	uint8_t* Super_C_Word;
	Super_C_Msg_t* Super_C_Msg_Show;
	int Super_C_Limit;
}Super_C_Judge_Msg_Show_t;

typedef struct
{
	uint8_t* Chassis_Mode_Name;
	Chassis_Mode_t* Chassis_Mode_Show;
	uint8_t* Chassis_Mode_Word_Add[6];
}Chassis_Mode_Judge_Msg_Show_t;

typedef struct
{
	uint8_t* Gimbal_Mode_Name;
	Gimbal_Mode_t* Gimbal_Mode_Show;
	uint8_t* Gimbal_Mode_Word_Add[7];
}Gimbal_Mode_Judge_Msg_Show_t;



typedef struct
{
	Super_C_Judge_Msg_Show_t Super_C_Judge_Msg_Show;
	Chassis_Mode_Judge_Msg_Show_t Chassis_Mode_Judge_Msg_Show;
	Gimbal_Mode_Judge_Msg_Show_t Gimbal_Mode_Judge_Msg_Show;
}Judge_Show_Msg_t;
	

void Judge_Task(void *pvParameters);

#endif

