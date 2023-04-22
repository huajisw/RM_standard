#include "supercap_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#define is_cap_online()  xTaskGetTickCount()-supercap.super_c_msg->Timestamp < CAP_OFFLINE_TIME_LIMIT


supercap_t supercap;

uint8_t is_supercap_alive()
{
		return is_cap_online()
				 &&supercap.cap_receive_data.cap_v > 3
				 &&supercap.cap_send_data.cap_control.bit.cap_switch != 0;
}

void supercap_on()
{
		supercap.supercap_on = 1;
}

void supercap_off()
{
		supercap.supercap_on = 0;
}

void supercap_data_update(supercap_t* data_update)
{
		//获取robot id
		data_update->robot_id = data_update->dji_judge_info->Judge_game_robot_status.robot_id;	
}

void supercap_mode_set(supercap_t* mode_set)
{
		if(mode_set->cap_receive_data.cap_state.bit.cap_i_over||mode_set->cap_receive_data.cap_state.bit.warning||
						mode_set->cap_receive_data.cap_state.bit.cap_v_over)
		{
				mode_set->supercap_mode = SUPCAP_MODE_ERROR;
		}
		else
		{
				//判断是否进入调试模式
				if(mode_set->debug_mode||!is_cap_online())
						mode_set->supercap_mode = SUPCAP_MODE_DEBUG;
				else
				{
					if(mode_set->robot_id==Red_Hero||mode_set->robot_id==Red_Standard_3||mode_set->robot_id==Red_Standard_4||
								mode_set->robot_id==Red_Standard_5||mode_set->robot_id==Blue_Hero||mode_set->robot_id==Blue_Standard_3||
								mode_set->robot_id==Blue_Standard_4||mode_set->robot_id==Blue_Standard_5)
							mode_set->supercap_mode = SUPCAP_MODE_NORMAL;
					else
							mode_set->supercap_mode = SUPCAP_MODE_NONE;
				}
		}
		
}

void supercap_tx_msg_send(supercap_t* msg_send)
{
    uint16_t can_data_0x2E[4];
		uint16_t can_data_0x2F[4];
		can_data_0x2E[0] = msg_send->cap_send_data.chassis_power_buffer;
    can_data_0x2E[1] = msg_send->cap_send_data.chassis_volt;
    can_data_0x2E[2] = msg_send->cap_send_data.chassis_current;
    CAN2_SuperCap_Control(0x2E,can_data_0x2E[0],can_data_0x2E[1],can_data_0x2E[2],0);

    can_data_0x2F[0] = msg_send->cap_send_data.chassis_power_limit;
    can_data_0x2F[1] = msg_send->cap_send_data.output_power_limit;
    can_data_0x2F[2] = msg_send->cap_send_data.input_power_limit;
    can_data_0x2F[3] = msg_send->cap_send_data.cap_control.all;
    CAN2_SuperCap_Control(0x2F,can_data_0x2F[0],can_data_0x2F[1],can_data_0x2F[2],can_data_0x2F[3]);
}

float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min)
{
    float b = (float)(a - a_min) / (float)(a_max - a_min) * (b_max - b_min) + b_min;
    return b;
}


void supercap_rx_msg_parse(supercap_t* msg_parse)
{
		msg_parse->cap_receive_data.cap_v = int16_to_float(msg_parse->super_c_msg->Cap_V,32000, -32000, 30, 0);
		msg_parse->cap_receive_data.cap_i = int16_to_float(msg_parse->super_c_msg->Cap_I,32000, -32000, 20, -20);
		msg_parse->cap_receive_data.cap_state.state = msg_parse->super_c_msg->Cap_State;
}

void supercap_control_set(supercap_t* control_set)
{
		if(control_set->supercap_mode!=SUPCAP_MODE_NONE)
		{
				if(control_set->supercap_mode==SUPCAP_MODE_NORMAL)
				{
						control_set->cap_send_data.chassis_volt = control_set->dji_judge_info->Judge_power_heat_data.chassis_volt;
						control_set->cap_send_data.chassis_current = control_set->dji_judge_info->Judge_power_heat_data.chassis_current;
						control_set->cap_send_data.chassis_power_limit = control_set->dji_judge_info->Judge_game_robot_status.chassis_power_limit;
						control_set->cap_send_data.chassis_power_buffer = control_set->dji_judge_info->Judge_power_heat_data.chassis_power_buffer;
						control_set->cap_send_data.cap_control.bit.gamegoing = (control_set->dji_judge_info->Judge_game_status.game_progress==4);
						control_set->cap_send_data.cap_control.bit.cap_switch = 1;
						control_set->cap_send_data.cap_control.bit.gamegoing = 1;
				}
				else if(control_set->supercap_mode==SUPCAP_MODE_DEBUG)
				{
						control_set->cap_send_data.chassis_power_limit = control_set->chassis_power_limit;
						control_set->cap_send_data.chassis_power_buffer = control_set->chassis_power_buffer;
						control_set->cap_send_data.cap_control.bit.gamegoing = control_set->game_going;
						control_set->cap_send_data.cap_control.bit.cap_switch = control_set->supercap_on;
				}
				//如果超级电容出现错误，停用超级电容，直到错误清除
				else if(control_set->supercap_mode==SUPCAP_MODE_ERROR)
				{
						control_set->cap_send_data.chassis_power_limit = control_set->chassis_power_limit;
						control_set->cap_send_data.chassis_power_buffer = control_set->chassis_power_buffer;
						control_set->cap_send_data.cap_control.bit.gamegoing = 0;
						control_set->cap_send_data.cap_control.bit.cap_switch = 0;
				}
				control_set->cap_send_data.input_power_limit = CAP_CHARGE_POWER_LIMIT;
				control_set->cap_send_data.output_power_limit = CAP_DISCHARGE_POWER_LIMIT;
				control_set->cap_send_data.cap_control.bit.cap_record = 1;
			
				supercap_tx_msg_send(control_set);
				supercap_rx_msg_parse(control_set);
		}
}

void supercap_init(supercap_t* init)
{
		init->dji_judge_info = Get_Judge_Info(); 
		init->super_c_msg = Get_Cap_Data();
		
		init->chassis_power_buffer = DEFAULT_CHASSIS_POWER_BUFFER;
		init->chassis_power_limit = DEFAULT_CHASSIS_POWER_LIMIT;
	
//		init->supercap_graphic = Judge_Graphic_Arc_Create(0,1,800,800,10,5);
//		
//		init->debug_mode = 1;
//		init->game_going = 0;
//		init->supercap_on = 0;
//		supercap_control_set(init);
//		vTaskDelay(2000);
//		
//		init->debug_mode = 1;
//		init->game_going = 1;
//		init->supercap_on  =1;
//		supercap_control_set(init);
		//vTaskDelay(2000);
}

void supercap_graphic_draw(supercap_t* graphic_draw)
{
		if(graphic_draw->supercap_mode != SUPCAP_MODE_NORMAL)
		{
				Judge_Graphic_Obj_Set_Color(graphic_draw->supercap_graphic,COLOR_ORANGE);
				Judge_Graphic_Obj_Set_End_Angle(graphic_draw->supercap_graphic,360);
		}
		else
		{
				uint32_t end_angle = graphic_draw->cap_receive_data.cap_v/CAP_MAX_VOLT*360;
				Judge_Graphic_Obj_Set_End_Angle(graphic_draw->supercap_graphic,end_angle);
				if(graphic_draw->cap_receive_data.cap_v < CAP_MIN_VOLT)
				{
						Judge_Graphic_Obj_Set_Color(graphic_draw->supercap_graphic,COLOR_YELLOW);
				}
				else
				{
						Judge_Graphic_Obj_Set_Color(graphic_draw->supercap_graphic,COLOR_GREEN);
				}	
		}
}

void supercap_task(void *pvParameters)
{
	vTaskDelay(100);
	supercap_init(&supercap);
	while(1)
	{
			supercap_data_update(&supercap);
			supercap_mode_set(&supercap);
			supercap_control_set(&supercap);
			vTaskDelay(15);
	}
}
