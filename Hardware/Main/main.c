/**
  ****************************(C) COPYRIGHT 2020 HRBUST_AIR****************************
  * @file    main.c
	* @brief   stm32初始化以及开始任务freeRTOS。h文件定义相关全局宏定义            
  * @author  JackyJuu , HRBUST_AIR_TEAM , website:www.airclub.tech
	* @Note 	 
  * @version V1.0.0
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     11-18-2020      JackyJuu            Done
  ****************************(C) COPYRIGHT 2020 HRBUST_AIR****************************
	* @describe 程序启动入口
*/
#include "main.h"
#include "delay.h"
#include "flash.h"
#include "power.h"
#include "Judge_Data.h"


#include "imu_bsp.h"
#include "wit_c_sdk.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Start_Task.h"
#include "IMU_Task.h"
#include "usb_usart.h"
#include "PC_usart.h"

#include "DJI_Remote_Control.h"

extern uint32_t Flash_Data[2];
uint32_t Flash_Data_Read[2];
int main(void)
{ 
	delay_init(configTICK_RATE_HZ);
	delay_ms(10);

	__disable_irq();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
	Usb_Init();	
	Led_Init();
	
	remote_control_init();

	
//	PC_Usart_Init();
	Judge_Usart_Init();
	
	Adc_Init();
	Power_24V_Init();

	UART7_Init();
	
	WT61_Init();
//	WT61_UART8_Init();
//  WitInit(WIT_PROTOCOL_NORMAL, 0x50);	
//	WitSerialWriteRegister(SensorUartSend);
//	WitRegisterCallBack(SensorDataUpdata);
//	WitDelayMsRegister(Delayms);	

	TIM1_Init();
	TIM2_Init();
	TIM4_Init();
	TIM5_Init();
	TIM8_Init();
	
	CAN1_Init();
	CAN2_Init();
	
	key_init();
	Laser_Init();
	Shoot_key_init();
	
	__enable_irq();
	startTast();
	vTaskStartScheduler();//开启任务调度
	while(1)             
	{


	}
}
	

