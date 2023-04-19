/**
  ****************************(C) COPYRIGHT 2020 HRBUST_AIR****************************
  * @file    start_task.c
	* @brief   Freertos任务初始化           
  * @author  JackyJuu , HRBUST_AIR_TEAM , website:www.airclub.tech
	* @Note 	 
  * @version V1.0.0
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     11-18-2020      JackyJuu            Done
  ****************************(C) COPYRIGHT 2020 HRBUST_AIR****************************
	* @describe Freertos任务初始化
*/
#include "Start_Task.h"
#include "User_Task.h"
#include "oled_task.h"
#include "judge_task.h"
#include "IMU_task.h"
#include "shoot_task.h"
#include "supercap_task.h"
#include "usb_task.h"


#include "gimbal_task.h"
#include "chassis_task.h"

#include "oled_list.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Judge_Data.h"
#include "Judge_Graphic.h"

#define START_TASK_PRIO 1
#define START_STK_SIZE 256
static TaskHandle_t StartTask_Handler;

#define USER_TASK_PRIO 4
#define USER_STK_SIZE 128
static TaskHandle_t UserTask_Handler;

#define OSKER_TASK_PRIO 6
#define OSKER_STK_SIZE 512
static TaskHandle_t OskerTask_Handler;


#define OLED_TASK_PRIO 8
#define OLED_STK_SIZE 512
static TaskHandle_t OledTask_Handler;

#define JUDGE_TASK_PRIO 10
#define JUDGE_STK_SIZE 512
static TaskHandle_t JudgeTask_Handler;

#define IMU_TASK_PRIO 20
#define IMU_STK_SIZE 512
static TaskHandle_t IMUTask_Handler;

#define SHOOT_TASK_PRIO 17
#define SHOOT_STK_SIZE 512
static TaskHandle_t ShootTask_Handler;

#define OLED_LIST_TASK_PRIO 18
#define OLED_LIST_STK_SIZE 256
static TaskHandle_t OledListTask_Handler;

#define GIMBAL_TASK_PRIO 19
#define GIMBAL_STK_SIZE 512
static TaskHandle_t Gimbaldask_Handler;

#define CHASSIS_TASK_PRIO 15
#define CHASSIS_STK_SIZE 512
static TaskHandle_t ChassisTask_Handler;

#define USB_TASK_PRIO 14
#define USB_STK_SIZE 512
static TaskHandle_t UsbTask_Handler;

#define CHECK_TASK_PRIO 13
#define CHECK_STK_SIZE 512
static TaskHandle_t CheckTask_Handler;


#define SUPERCAP_TASK_PRIO 25
#define SUPERCAP_STK_SIZE 512
TaskHandle_t SuperCapTask_Handler;


void start_task(void *pvParameters)
{
    Judge_Data_Init();
		Judge_Graphic_Init();
	
		taskENTER_CRITICAL();

    xTaskCreate((TaskFunction_t)UserTast,
	  (const char *)"UserTast",
	  (uint16_t)USER_STK_SIZE,(void *)NULL,
		(UBaseType_t)USER_TASK_PRIO,
		(TaskHandle_t *)&UserTask_Handler);
		
		xTaskCreate((TaskFunction_t)IMU_Task,
		(const char *)"IMU_Task",
		(uint16_t)IMU_STK_SIZE,(void *)NULL,
		(UBaseType_t)IMU_TASK_PRIO,
		(TaskHandle_t *)&IMUTask_Handler);		
			
		xTaskCreate((TaskFunction_t)Gimbal_Task,
		(const char *)"Gimbal_Task",
		(uint16_t)GIMBAL_STK_SIZE,
		(void *)NULL,(UBaseType_t)GIMBAL_TASK_PRIO,
		(TaskHandle_t *)&Gimbaldask_Handler);		
	
		xTaskCreate((TaskFunction_t)Chassis_Task,
		(const char *)"Chassis_Task",
		(uint16_t)CHASSIS_STK_SIZE,
	  (void *)NULL,(UBaseType_t)CHASSIS_TASK_PRIO,
		(TaskHandle_t *)&Gimbaldask_Handler);			
		
		xTaskCreate((TaskFunction_t)Shoot_Task,
		(const char *)"Shoot_Task",
		(uint16_t)SHOOT_STK_SIZE,(void *)NULL,
		(UBaseType_t)SHOOT_TASK_PRIO,
		(TaskHandle_t *)&ShootTask_Handler);
		
//		xTaskCreate((TaskFunction_t)Check_Task,  //   超级电容通信
//		(const char *)"Check_Task",
//		(uint16_t)CHECK_STK_SIZE,
//		(void *)NULL,
//		(UBaseType_t)CHECK_TASK_PRIO,
//		(TaskHandle_t *)&CheckTask_Handler);
		
//		xTaskCreate((TaskFunction_t)supercap_task,
//	  (const char *)"SuperCap_Task",
//	  (uint16_t)SUPERCAP_STK_SIZE,
//		(void *)NULL,
//		(UBaseType_t)SUPERCAP_TASK_PRIO,
//		(TaskHandle_t *)&SuperCapTask_Handler);	

		xTaskCreate((TaskFunction_t)Judge_Task, //   裁判系统通信任务 
		(const char *)"Judge_Task",
		(uint16_t)JUDGE_STK_SIZE,
		(void *)NULL,
		(UBaseType_t)JUDGE_TASK_PRIO,
		(TaskHandle_t *)&JudgeTask_Handler);			
		
		
				
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

void startTast(void)
{
    xTaskCreate((TaskFunction_t)start_task,          //任务函数
                (const char *)"start_task",          //任务名称
                (uint16_t)START_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&StartTask_Handler); //任务句柄
								
}

