#include "chassis_task.h"
#include "chassis_power_control.h"

#define POWER_LIMIT         80.0f
#define WARNING_POWER       40.0f   
#define WARNING_POWER_BUFF  50.0f   

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4,    
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f            
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f  // //��Ϊ��֪����զ����� �����Թٷ�C���80��Ϊ��׼�� ����������ȥ


/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data 
  * @retval         none
  */
/**
  * @brief          ���ƹ��ʣ���Ҫ���Ƶ������
  * @param[in]      chassis_power_control: ��������
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
					//���ʳ���80w �ͻ�������С��60j,��Ϊ��������С��60��ζ�Ź��ʳ���80w
					if(chassis_power_buffer < Waring_Power_Buff)
					{
							fp32 power_scale;               //��С��һ���ٷֱ�ϵ�� 
							if(chassis_power_buffer > 5.0f)  //    5 <  �������� < 50
							{
									//scale down WARNING_POWER_BUFF
									//��СWARNING_POWER_BUFF
									power_scale = chassis_power_buffer / Waring_Power_Buff;    // ��ǰ�Ļ������� ����  ���滺������   now/50
							}
							else                             //        �������� < 5
							{
									//only left 10% of WARNING_POWER_BUFF
									power_scale = 5.0f / Waring_Power_Buff;                   //
							}
							//scale down
							//��С
							total_current_limit = Buffer_Total_Current_Limit * power_scale;
					}
					else
					{
							//power > WARNING_POWER
							//���ʴ���WARNING_POWER
							if(chassis_power > Waring_Power_Buff)
							{
									fp32 power_scale;
									//power < 80w
									//����С��80w
									if(chassis_power < Power_Limit)
									{
											//scale down
											//��С
											power_scale = (Power_Limit - chassis_power) / (Power_Limit - Waring_Power);
											
									}
									//power > 80w
									//���ʴ���80w
									else
									{
											power_scale = 0.0f;
									}
									
									total_current_limit = Buffer_Total_Current_Limit + Power_Ttoal_Current_Limit * power_scale;
							}
							//power < WARNING_POWER
							//����С��WARNING_POWER
							else
							{
									total_current_limit = Buffer_Total_Current_Limit + Power_Ttoal_Current_Limit;
							}
					}
			

			
			total_current = 0.0f;
			//calculate the original motor current set
			//����ԭ����������趨
			for(uint8_t i = 0; i < 4; i++)
			{
					total_current += fabs(chassis_power_control->Chassis_Motor_Curent_Send[i]);  //��������  ��������ﵽ����ٶ�����Ҫ�� �ĸ�����ĵ����ܺ�   
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