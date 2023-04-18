#include "chassis_task.h"
#include "chassis_power_control.h"

#define POWER_LIMIT         80.0f
#define WARNING_POWER       40.0f   
#define WARNING_POWER_BUFF  50.0f   

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4,    
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f            
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f  // //因为不知道这咋换算的 所以以官方C版的80作为基准线 按比例算下去


/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data 
  * @retval         none
  */
/**
  * @brief          限制功率，主要限制电机电流
  * @param[in]      chassis_power_control: 底盘数据
  * @retval         none
  */
void chassis_power_control(Chassis_t *chassis_power_control)
{
    fp32 chassis_power = 0.0f;
    fp32 chassis_power_buffer = 0.0f;
    fp32 total_current_limit = 0.0f;
    fp32 total_current = 0.0f;
	  fp32 Power_Limit = 0.0f;
	  fp32 Waring_Power = 0.0f;
	  fp32 Waring_Power_Buff = 0.0f;	
	  fp32 No_Judeg_Total_Current_Limit = 0.0f;
	  fp32 Buffer_Total_Current_Limit = 0.0f;
	  fp32 Power_Ttoal_Current_Limit = 0.0f;
	
    //uint8_t robot_id = get_robot_id();
	  
	
	  chassis_power = chassis_power_control->Chassos_Judge_Msg.Chassis_Judge_Mes_Get->Judge_power_heat_data.chassis_power;
	  chassis_power_buffer = chassis_power_control->Chassos_Judge_Msg.Chassis_Judge_Mes_Get->Judge_power_heat_data.chassis_power_buffer;
	  Power_Limit = chassis_power_control->Chassos_Judge_Msg.Chassis_Judge_Mes_Get->Judge_game_robot_status.chassis_power_limit;
	  No_Judeg_Total_Current_Limit = 64000.0f;
	  Buffer_Total_Current_Limit = 16000.0f;
	  Waring_Power = Power_Limit/2 ;
    Waring_Power_Buff = 50.0f; 
	
    if( Power_Limit > 80)	          
	    Power_Ttoal_Current_Limit = ((Power_Limit - 80)/80)*20000.0f;
		if(Power_Limit < 80)
			Power_Ttoal_Current_Limit = ((80 - Power_Limit )/80)*20000.0f;
		if(Power_Limit == 80)
			Power_Ttoal_Current_Limit = 20000.0f;
	
				//	get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
					// power > 80w and buffer < 60j, because buffer < 60 means power has been more than 80w
					//功率超过80w 和缓冲能量小于60j,因为缓冲能量小于60意味着功率超过80w
					if(chassis_power_buffer < Waring_Power_Buff)
					{
							fp32 power_scale;               //缩小的一个百分比系数 
							if(chassis_power_buffer > 5.0f)  //    5 <  缓冲能量 < 50
							{
									//scale down WARNING_POWER_BUFF
									//缩小WARNING_POWER_BUFF
									power_scale = chassis_power_buffer / Waring_Power_Buff;    // 当前的缓冲能量 除于  警告缓冲能量   now/50
							}
							else                             //        缓冲能量 < 5
							{
									//only left 10% of WARNING_POWER_BUFF
									power_scale = 5.0f / Waring_Power_Buff;                   //
							}
							//scale down
							//缩小
							total_current_limit = Buffer_Total_Current_Limit * power_scale;
					}
					else
					{
							//power > WARNING_POWER
							//功率大于WARNING_POWER
							if(chassis_power > Waring_Power_Buff)
							{
									fp32 power_scale;
									//power < 80w
									//功率小于80w
									if(chassis_power < Power_Limit)
									{
											//scale down
											//缩小
											power_scale = (Power_Limit - chassis_power) / (Power_Limit - Waring_Power);
											
									}
									//power > 80w
									//功率大于80w
									else
									{
											power_scale = 0.0f;
									}
									
									total_current_limit = Buffer_Total_Current_Limit + Power_Ttoal_Current_Limit * power_scale;
							}
							//power < WARNING_POWER
							//功率小于WARNING_POWER
							else
							{
									total_current_limit = Buffer_Total_Current_Limit + Power_Ttoal_Current_Limit;
							}
					}
			

			
			total_current = 0.0f;
			//calculate the original motor current set
			//计算原本电机电流设定
			for(uint8_t i = 0; i < 4; i++)
			{
					total_current += fabs(chassis_power_control->Chassis_Motor_Curent_Send[i]);  //拨动拨杆  计算出来达到这个速度所需要的 四个电机的电流总和   
			}
			

			if(total_current > total_current_limit)
			{
					fp32 current_scale = total_current_limit / total_current;
					chassis_power_control->Chassis_Motor_Curent_Send[0]*=current_scale;
					chassis_power_control->Chassis_Motor_Curent_Send[1]*=current_scale;
					chassis_power_control->Chassis_Motor_Curent_Send[2]*=current_scale;
					chassis_power_control->Chassis_Motor_Curent_Send[3]*=current_scale;
			}
}