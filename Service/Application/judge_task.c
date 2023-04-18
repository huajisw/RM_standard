/**
  ****************************(C) COPYRIGHT 2021 HRBUST_AIR****************************
  * @file    judge_task.c
	* @brief   裁判系统任务           
  * @author  JackyJuu , HRBUST_AIR_TEAM , website:www.airclub.tech
	* @Note 	 定期发送
  * @version V1.0.0
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     12-4-2020      JackyJuu            Done
  *  V1.2.0      4-2-2020      JackyJuu            Done	
  ****************************(C) COPYRIGHT 2021 HRBUST_AIR****************************
	* @describe OLED显示屏任务
*/
#include "judge_task.h"

#include "chassis_task.h"

#include "Judge_Data.h"
#include "Judge_Graphic.h"
#include "DJI_Remote_Control.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

//#define Robot_Com_Id 0x0103

//Judge_Show_Msg_t Judge_Show_Msg;

//uint8_t Chassis_Mode_Word[8][40] = {"Chassis_Zero",//无控制，传送0
//																	"Chassis_Motionless",//无控制，用电机角度保持当前位置
//																	"Chassis_RC_Control",//遥控器控制
//																	"Chassis_Follow_Gimbal",//底盘跟随云台
//																	"Chassis_Follow_Chassis",//我跟我自己
//																	"Chassis_Spin_Right",
//																	"Chassis_Spin_Left",
//																	"Chassis_Spin"};//小陀螺模式

//uint8_t Gimbal_Mode_Word[7][40] = {"Gimbal_Zero",//无控制，传送0
//																	"Gimbal_Motionless",//无控制，用电机角度保持当前位置
//																	"Gimbal_Motor_Control",//电机角度控制
//																	"Gimbal_IMU_Control",//陀螺仪角度控制
//																	"Gimbal_Vision_Control",//视觉控制接口
//																	"Gimbal_Follow_Chassis",//云台跟随底盘，yaw轴固定不动
//																	"Gimbal_Spin"};//小陀螺模式	

//uint8_t Super_Word[40] = {"SuperC_Limit: %"};

//extern DJI_Judge_t DJI_Judge; 
//extern RC_Ctl_t RC_Ctl_Data;

//uint8_t Judge_Test_Send_Data[6] = {0x11,0x22,0x33,0x44,0x55,0x66};
//uint8_t Judge_Test_Graphic_Name[] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99};

////uint8_t	Judge_Test_Graphic_Data[200];

//int test_size;

//uint32_t test_judge_graphic_data;

//int write_mode = 0;

////uint8_t	Judge_Test_Graphic_Data[] = "Hello World";
//uint8_t Judge_Test_Graphic_Data_Send[45];

//float test_nummmm = 66.66;
//int32_t test_nummmmmm = 12888;

//uint8_t	Judge_Super_Cap_Data_Send[] = "Super-Cap:";

//uint8_t Judge_Test_Graphic_Data[3][2][30] = {{"DN1","Super-Cap:"},\
//																						 {"DN2","Version_On"},\
//																						 {"DN3","Version_Off"}};

//int Judge_Set_Init[10][4] = {{3,20,750,780},//超级电容数据显示
//														 {3,20,750,740},//视觉自瞄是否瞄到显示
//														 {2,15,800,400},//底盘模式
//														 {3,20,800,300},//云台模式
//														 {3,20,800,300},
//														 {3,20,800,300},
//														 {3,20,800,300},
//														 {3,20,800,300},
//														 {3,20,800,300},
//														 {3,20,800,300}};

//int Location_x[10],Location_y[10];
//int Data_Width[10],Data_Sizw[10];
//int Judge_Num_Set;
//int key_last;
//														 
//uint8_t Student_Judge_Send_Data[] = "FUCK YOU";
//														 
//int Judge_Data_Send_Update;
//														 
//void Judge_Send_Data_Update(uint8_t* Graphic_Update_Name)
//{
//	Judge_Character_operate_tpye_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,Graphic_Update_Name,Graphic_Add);
//	Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);	
//	Judge_Character_operate_tpye_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,Graphic_Update_Name,Graphic_Add);
//	Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);	
//}

////uint8_t Leida_Data[14] = {}
//int Show_Num;

//void Judge_Chassis_Mode_Show_Add(Chassis_Mode_Judge_Msg_Show_t* Judge_Msg_Show,int Locate_X,int Locate_Y)
//{
//		Show_Num = (int)*Judge_Msg_Show->Chassis_Mode_Show;
//		Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,Judge_Msg_Show->Chassis_Mode_Name,\
//		Character,Graphic_Add,Judge_Chassis_Show_Floor,3,15,14,2,Locate_X,Locate_Y,Judge_Msg_Show->Chassis_Mode_Word_Add[Show_Num]);
//		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);	
//}

//void Judge_Gimbal_Mode_Show_Add(Gimbal_Mode_Judge_Msg_Show_t* Judge_Msg_Show,int Locate_X,int Locate_Y)
//{
//		Show_Num = (int)*Judge_Msg_Show->Gimbal_Mode_Show;
//		Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,Judge_Msg_Show->Gimbal_Mode_Name,\
//		Character,Graphic_Add,Judge_Gimbal_Show_Floor,3,15,14,2,Locate_X,Locate_Y,Judge_Msg_Show->Gimbal_Mode_Word_Add[Show_Num]);
//		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);	
//}


//void Judge_Chassis_Mode_Show_Set(Chassis_Mode_Judge_Msg_Show_t* Judge_Msg_Show,int Locate_X,int Locate_Y)
//{
//		Show_Num = (int)*Judge_Msg_Show->Chassis_Mode_Show;
//		Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,Judge_Msg_Show->Chassis_Mode_Name,\
//		Character,Graphic_Change,Judge_Chassis_Show_Floor,3,15,14,2,Locate_X,Locate_Y,Judge_Msg_Show->Chassis_Mode_Word_Add[Show_Num]);
//		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);	
//}

//void Judge_Gimbal_Mode_Show_Set(Gimbal_Mode_Judge_Msg_Show_t* Judge_Msg_Show,int Locate_X,int Locate_Y)
//{
//		Show_Num = (int)*Judge_Msg_Show->Gimbal_Mode_Show;
//		Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,Judge_Msg_Show->Gimbal_Mode_Name,\
//		Character,Graphic_Change,Judge_Gimbal_Show_Floor,3,15,14,2,Locate_X,Locate_Y,Judge_Msg_Show->Gimbal_Mode_Word_Add[Show_Num]);
//		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);	
//}

//void Judge_Vision_Show_Add(int Vision_flag,int Locate_X,int Locate_Y)
//{
//	if(Vision_flag == 1)
//	{
//		Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,(uint8_t*)"VSD",\
//		Character,Graphic_Add,Judge_Vision_Floor,3,15,14,2,Locate_X,Locate_Y,Judge_Test_Graphic_Data[1][1]);
//		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);
//		
//	}
//	else if(Vision_flag == 0)
//	{
//		Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,(uint8_t*)"VSD",\
//		Character,Graphic_Add,Judge_Vision_Floor,3,15,14,2,Locate_X,Locate_Y,Judge_Test_Graphic_Data[2][1]);
//		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);		
//	}
//}

//void Judge_Vision_Show_Set(int Vision_flag,int Locate_X,int Locate_Y)
//{
//	if(Vision_flag == 1)
//	{
//		Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,(uint8_t*)"VSD",\
//		Character,Graphic_Change,Judge_Vision_Floor,3,15,14,2,Locate_X,Locate_Y,Judge_Test_Graphic_Data[1][1]);
//		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);	
//	}
//	else if(Vision_flag == 0)
//	{
//		Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,(uint8_t*)"VSD",\
//		Character,Graphic_Change,Judge_Vision_Floor,3,15,14,2,Locate_X,Locate_Y,Judge_Test_Graphic_Data[2][1]);
//		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);		
//	}
//}

//void Judge_Shoot_Line_Add()
//{
//	
//			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[0],(uint8_t*)"LI1",\
//			Straight_Line,Graphic_Add,0,2,0,0,5,920,430,0,1000,430);
//			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[1],(uint8_t*)"LI2",\
//			Straight_Line,Graphic_Add,1,3,0,0,5,900,443,0,1020,443);
//			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[2],(uint8_t*)"LI3",\
//			Straight_Line,Graphic_Add,2,4,0,0,5,880,420,0,1040,420);
//			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[3],(uint8_t*)"LI4",\
//			Straight_Line,Graphic_Add,3,5,0,0,5,860,400,0,1060,400);
//			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[4],(uint8_t*)"LI5",\
//			Straight_Line,Graphic_Add,4,6,0,0,5,840,395,0,1080,395);
//			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[5],(uint8_t*)"LI6",\
//			Straight_Line,Graphic_Add,5,7,0,0,3,960,540,0,960,360);
//			Judge_Data_Send_To_Client(&DJI_Judge,0x0104,DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.Judge_client_custom_graphic_seven_Data,105);

//}

//void Judge_Shoot_Line_Set()
//{
//	
//			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[0],(uint8_t*)"LI1",\
//			Straight_Line,Graphic_Change,0,2,0,0,5,920,430,0,1000,430);
//			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[1],(uint8_t*)"LI2",\
//			Straight_Line,Graphic_Change,1,3,0,0,5,900,443,0,1020,443);
//			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[2],(uint8_t*)"LI3",\
//			Straight_Line,Graphic_Change,2,4,0,0,5,880,420,0,1040,420);
//			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[3],(uint8_t*)"LI4",\
//			Straight_Line,Graphic_Change,3,5,0,0,5,860,400,0,1060,400);
//			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[4],(uint8_t*)"LI5",\
//			Straight_Line,Graphic_Change,4,6,0,0,5,840,395,0,1080,395);
//			Judge_Graphic_Data_Set((graphic_data_struct_t*)&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.grapic_data_struct[5],(uint8_t*)"LI6",\
//			Straight_Line,Graphic_Change,5,7,0,0,3,960,540,0,960,360);
//			Judge_Data_Send_To_Client(&DJI_Judge,0x0104,DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Seven.Judge_client_custom_graphic_seven_Data,105);

//}

//void Judge_Super_C_Show_Add(Super_C_Judge_Msg_Show_t* Super_C_Judge_Msg_Show,int Locate_X,int Locate_Y)
//{

//		Super_C_Judge_Msg_Show->Super_C_Limit = (int)(((float)(Super_C_Judge_Msg_Show->Super_C_Msg_Show->Voilt_C) - 1400.00f) / ((float)(Super_C_Judge_Msg_Show->Super_C_Msg_Show->Voilt_In) - 1400.00f) * 100.00f);
//		
//		Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,Super_C_Judge_Msg_Show->Super_C_Name,\
//		Character,Graphic_Add,Judge_Super_C_Show_Floor,3,15,14,2,Locate_X,Locate_Y,Super_C_Judge_Msg_Show->Super_C_Word);
//		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);		
//		Judge_Word_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Single.Word_data_struct,Judge_Test_Graphic_Name,\
//		Integer,Graphic_Add,Judge_Super_C_Show_Floor,3,15,14,2,Locate_X + 183,Locate_Y,0,Super_C_Judge_Msg_Show->Super_C_Limit);		
//		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Single.Judge_client_custom_graphic_single_Data,15);		
//	
//}

//void Judge_Super_C_Show_Set(Super_C_Judge_Msg_Show_t* Super_C_Judge_Msg_Show,int Locate_X,int Locate_Y)
//{
//		Super_C_Judge_Msg_Show->Super_C_Limit = (int)(((float)(Super_C_Judge_Msg_Show->Super_C_Msg_Show->Voilt_C) - 1400.00f) / ((float)(Super_C_Judge_Msg_Show->Super_C_Msg_Show->Voilt_In) - 1400.00f) * 100.00f);
//		Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,Super_C_Judge_Msg_Show->Super_C_Name,\
//		Character,Graphic_Change,Judge_Super_C_Show_Floor,3,15,14,2,Locate_X,Locate_Y,Super_C_Judge_Msg_Show->Super_C_Word);
//		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);		
//		Judge_Word_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Single.Word_data_struct,Judge_Test_Graphic_Name,\
//		Integer,Graphic_Change,Judge_Super_C_Show_Floor,3,15,14,2,Locate_X + 183,Locate_Y,0,Super_C_Judge_Msg_Show->Super_C_Limit);		
//		Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Graphic_Client_Single.Judge_client_custom_graphic_single_Data,15);		
//}

//void Robot_Mode_Judge_Show(Judge_Show_Msg_t* Judge_Show_Mode)
//{
//			Judge_Character_Data_Set(&DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_client_custom_character,Judge_Show_Mode->Chassis_Mode_Judge_Msg_Show.Chassis_Mode_Name,\
//			Character,Graphic_Add,2,3,Data_Sizw[2],14,Data_Width[2],1700,700,Judge_Show_Mode->Chassis_Mode_Judge_Msg_Show.Chassis_Mode_Word_Add[1]);
//			Judge_Data_Send_To_Client(&DJI_Judge,0x0110,DJI_Judge.DJI_Judge_Send_Msg.Judge_Character_Client.Judge_Character_Data,Graphic_character_Long);	
//}

//extern int Vision_Get_Flag;
//static float Judge_Angle_Get;
//int Data_Lennnn;
//int super_C_Num_Test;

void Judge_Task(void *pvParameters)
{
		while(1)
		{
				Judge_Graphic_Handler();
				vTaskDelay(100);
		}
}
