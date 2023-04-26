#include "Judge_Graphic.h"

DJI_Judge_Graphic_t DJI_Judge_Graphic;

#define JUDGE_GRAPHIC_STK_SIZE 256
#define JUDGE_GRAPHIC_TASK_PRIO 21
TaskHandle_t JudgeGraphicTask_Handler;

#define JUDGE_GRAPHIC_DATA_LEN 15
#define JUDGE_GRAPHIC_CHARACTER_LEN 30

#define Judge_Graphic_Obj_Lock(Obj) xSemaphoreTake((Obj)->Graphic_Obj_Mutex,portMAX_DELAY)
#define Judge_Graphic_Obj_Poll(Obj) xSemaphoreTake((Obj)->Graphic_Obj_Mutex,0)
#define Judge_Graphic_Obj_Unlock(Obj)  			xSemaphoreGive((Obj)->Graphic_Obj_Mutex)

//图形操作优先级查找表,由高到低依次是：删除操作、添加操作、修改操作、空操作
uint8_t Judge_Graphic_Opt_Prio[4] = {0,2,1,3};
//要求操作的优先级大于当前操作的优先级，那么覆盖当前操作

uint8_t Judge_Graphic_Obj_Apply_Opt(Judge_Graphic_Obj_Handle Obj,Judge_Graphic_Operation_t Opt)
{
	BaseType_t Ret = pdTRUE;
	if(Obj->Graphic_Data.operate_tpye == OPT_NONE)
	{
			if(Obj->Graphic_Data.graphic_tpye == CHARACTER)
					Ret = xQueueSend(DJI_Judge_Graphic.Judge_Graphic_Character_Queue,&Obj,0);	
			else
					Ret = xQueueSend(DJI_Judge_Graphic.Judge_Graphic_CommObj_Queue,&Obj,0);	
	}
	
	if(Ret == pdTRUE && Judge_Graphic_Opt_Prio[(Opt)]>Judge_Graphic_Opt_Prio[(Obj)->Graphic_Data.operate_tpye])
			Obj->Graphic_Data.operate_tpye = Opt;	
}

/********基础操作函数**********/

uint8_t Judge_Graphic_Obj_Init(Judge_Graphic_Obj_Handle Obj)
{
		if(DJI_Judge_Graphic.Judge_Graphic_Obj_Counter > 0xFFF)
			return 0;
	
		//创建信号量
		SemaphoreHandle_t Mutex = xSemaphoreCreateBinary();
		if(!Mutex)
				return 0;
		
		memset(Obj,0,sizeof(Judge_Graphic_Obj_t));
		vListInitialiseItem((ListItem_t*)Obj);
		Obj->Graphic_Data.graphic_name = DJI_Judge_Graphic.Judge_Graphic_Obj_Counter++;
		Obj->Graphic_Obj_Mutex = Mutex;
	
		//List_Insert_End(&DJI_Judge_Graphic.Graphic_Obj_List,(ListItem_t*)Obj,portMAX_DELAY);
		return 1;
}

void Judge_Graphic_Obj_Set_Color(Judge_Graphic_Obj_Handle Obj,Judge_Graphic_Color_t Color)
{
		if(!Obj)
			return;
		
		Judge_Graphic_Obj_Lock(Obj);
	
		Obj->Graphic_Data.color = Color;

		Judge_Graphic_Obj_Apply_Opt(Obj,OPT_CHANGE);
		Judge_Graphic_Obj_Unlock(Obj);
}

uint32_t Judge_Graphic_Obj_Get_Width(Judge_Graphic_Obj_Handle Obj)
{
		if(!Obj)
			return 0;
		
		Judge_Graphic_Obj_Lock(Obj);
	
		uint32_t W = 0;
		if(Obj->Graphic_Data.graphic_tpye == ELLIPSE||Obj->Graphic_Data.graphic_tpye == ARC)
		{
				W = Obj->Graphic_Data.graphic_union.graphic_data.end_x;
		}
		else if(Obj->Graphic_Data.graphic_tpye == CIRCLE)
		{
				W = Obj->Graphic_Data.graphic_union.graphic_data.radius;
		}
		else if(Obj->Graphic_Data.graphic_tpye == RECTANGLE||Obj->Graphic_Data.graphic_tpye == LINE)
		{
				W = Obj->Graphic_Data.graphic_union.graphic_data.end_x - Obj->Graphic_Data.start_x;
		}
		
		Judge_Graphic_Obj_Unlock(Obj);
		return W;
}

uint32_t Judge_Graphic_Obj_Get_Height(Judge_Graphic_Obj_Handle Obj)
{
		if(!Obj)
			return 0;
		
		Judge_Graphic_Obj_Lock(Obj);
	
		uint32_t H = 0;
		if(Obj->Graphic_Data.graphic_tpye == ELLIPSE||Obj->Graphic_Data.graphic_tpye == ARC)
		{
				H = Obj->Graphic_Data.graphic_union.graphic_data.end_y;
		}
		else if(Obj->Graphic_Data.graphic_tpye == CIRCLE)
		{
				H = Obj->Graphic_Data.graphic_union.graphic_data.radius;
		}
		else if(Obj->Graphic_Data.graphic_tpye == RECTANGLE||Obj->Graphic_Data.graphic_tpye == LINE)
		{
				H = Obj->Graphic_Data.graphic_union.graphic_data.end_y - Obj->Graphic_Data.start_y;
		}
		
		Judge_Graphic_Obj_Unlock(Obj);
		return H;
}

void Judge_Graphic_Obj_Set_Width(Judge_Graphic_Obj_Handle Obj,uint32_t W)
{
	  if(!Obj)
			return;
		Judge_Graphic_Obj_Lock(Obj);
		
		if(Obj->Graphic_Data.graphic_tpye == ELLIPSE||Obj->Graphic_Data.graphic_tpye == ARC)
		{
				uint32_t Ra = Obj->Graphic_Data.graphic_union.graphic_data.end_x;
				uint32_t X = Obj->Graphic_Data.start_x - Ra/2;
				Obj->Graphic_Data.start_x = X + W/2;
				Obj->Graphic_Data.graphic_union.graphic_data.end_x = W;
		}
		else if(Obj->Graphic_Data.graphic_tpye == CIRCLE)
		{
				uint32_t R = Obj->Graphic_Data.graphic_union.graphic_data.radius;
				uint32_t X = Obj->Graphic_Data.start_x - R/2;
				Obj->Graphic_Data.start_x = X + W/2;
				Obj->Graphic_Data.graphic_union.graphic_data.radius = W;
		}
		else if(Obj->Graphic_Data.graphic_tpye == RECTANGLE||Obj->Graphic_Data.graphic_tpye == LINE)
		{
				Obj->Graphic_Data.graphic_union.graphic_data.end_x = Obj->Graphic_Data.start_x + W;
		}
		
		Judge_Graphic_Obj_Apply_Opt(Obj,OPT_CHANGE);
		Judge_Graphic_Obj_Unlock(Obj);
}

void Judge_Graphic_Obj_Set_Height(Judge_Graphic_Obj_Handle Obj,uint32_t H)
{
	  if(!Obj)
			return;
		Judge_Graphic_Obj_Lock(Obj);
		
		if(Obj->Graphic_Data.graphic_tpye == ELLIPSE||Obj->Graphic_Data.graphic_tpye == ARC)
		{
				uint32_t Rb = Obj->Graphic_Data.graphic_union.graphic_data.end_y;
				uint32_t Y = Obj->Graphic_Data.start_y - Rb/2;
				Obj->Graphic_Data.start_y = Y + H/2;
				Obj->Graphic_Data.graphic_union.graphic_data.end_y = H;
		}
		else if(Obj->Graphic_Data.graphic_tpye == CIRCLE)
		{
				uint32_t R = Obj->Graphic_Data.graphic_union.graphic_data.radius;
				uint32_t Y = Obj->Graphic_Data.start_y - R/2;
				Obj->Graphic_Data.start_x = Y + H/2;
				Obj->Graphic_Data.graphic_union.graphic_data.radius = H;
		}
		else if(Obj->Graphic_Data.graphic_tpye == RECTANGLE||Obj->Graphic_Data.graphic_tpye == LINE)
		{
				Obj->Graphic_Data.graphic_union.graphic_data.end_y = Obj->Graphic_Data.start_y + H;
		}
		
		Judge_Graphic_Obj_Apply_Opt(Obj,OPT_CHANGE);
		Judge_Graphic_Obj_Unlock(Obj);
}

void Judge_Graphic_Obj_Set_Size(Judge_Graphic_Obj_Handle Obj,uint32_t W,uint32_t H)
{   if(!Obj)
	     return;
		Judge_Graphic_Obj_Set_Width(Obj,W);
		Judge_Graphic_Obj_Set_Height(Obj,H);
}

uint32_t Judge_Graphic_Obj_Get_X(Judge_Graphic_Obj_Handle Obj)
{   
	  if(!Obj)
			return 0;
		Judge_Graphic_Obj_Lock(Obj);
	
		uint32_t X = 0;
		if(Obj->Graphic_Data.graphic_tpye == ELLIPSE||Obj->Graphic_Data.graphic_tpye == ARC)
		{
				uint32_t Ra = Obj->Graphic_Data.graphic_union.graphic_data.end_x;
				X = Obj->Graphic_Data.start_x - Ra/2;
		}
		else if(Obj->Graphic_Data.graphic_tpye == CIRCLE)
		{
				uint32_t R = Obj->Graphic_Data.graphic_union.graphic_data.radius;
				X = Obj->Graphic_Data.start_x - R/2;
		}
		else
		{
				X = Obj->Graphic_Data.start_x;
		}
		
		Judge_Graphic_Obj_Unlock(Obj);
		return X;
}

uint32_t Judge_Graphic_Obj_Get_Y(Judge_Graphic_Obj_Handle Obj)
{   
		if(!Obj)
			return 0;
		Judge_Graphic_Obj_Lock(Obj);
	
		uint32_t Y = 0;
		if(Obj->Graphic_Data.graphic_tpye == ELLIPSE||Obj->Graphic_Data.graphic_tpye == ARC)
		{
				uint32_t Rb = Obj->Graphic_Data.graphic_union.graphic_data.end_y;
				Y = Obj->Graphic_Data.start_y - Rb/2;
		}
		else if(Obj->Graphic_Data.graphic_tpye == CIRCLE)
		{
				uint32_t R = Obj->Graphic_Data.graphic_union.graphic_data.radius;
				Y = Obj->Graphic_Data.start_y - R/2;
		}
		else
		{
				Y = Obj->Graphic_Data.start_y;
		}
		
		Judge_Graphic_Obj_Unlock(Obj);
		return Y;
}


void Judge_Graphic_Obj_Set_X(Judge_Graphic_Obj_Handle Obj,uint32_t X)
{   
	  if(!Obj)
			return;
		Judge_Graphic_Obj_Lock(Obj);
		
		if(Obj->Graphic_Data.graphic_tpye == ELLIPSE||Obj->Graphic_Data.graphic_tpye == ARC)
		{
				uint32_t Ra = Obj->Graphic_Data.graphic_union.graphic_data.end_x;
				Obj->Graphic_Data.start_x = X + Ra/2;
		}
		else if(Obj->Graphic_Data.graphic_tpye == CIRCLE)
		{
				uint32_t R = Obj->Graphic_Data.graphic_union.graphic_data.radius;
				Obj->Graphic_Data.start_x = X + R/2;
		}
		else
		{
				uint32_t W = Obj->Graphic_Data.graphic_union.graphic_data.end_x - Obj->Graphic_Data.start_x;
				Obj->Graphic_Data.start_x = X;
				Obj->Graphic_Data.graphic_union.graphic_data.end_x = X+W;
		}
		Judge_Graphic_Obj_Apply_Opt(Obj,OPT_CHANGE);
		Judge_Graphic_Obj_Unlock(Obj);
}

void Judge_Graphic_Obj_Set_Y(Judge_Graphic_Obj_Handle Obj,uint32_t Y)
{    
	  if(!Obj)
			return;
		Judge_Graphic_Obj_Lock(Obj);
		
		if(Obj->Graphic_Data.graphic_tpye == ELLIPSE||Obj->Graphic_Data.graphic_tpye == ARC)
		{
				uint32_t Ra = Obj->Graphic_Data.graphic_union.graphic_data.end_y;
				Obj->Graphic_Data.start_y = Y + Ra/2;
		}
		else if(Obj->Graphic_Data.graphic_tpye == CIRCLE)
		{
				uint32_t R = Obj->Graphic_Data.graphic_union.graphic_data.radius;
				Obj->Graphic_Data.start_y = Y + R/2;
		}
		else
		{
				uint32_t H = Obj->Graphic_Data.graphic_union.graphic_data.end_y - Obj->Graphic_Data.start_y;
				Obj->Graphic_Data.start_y = Y;
				Obj->Graphic_Data.graphic_union.graphic_data.end_y = Y+H;
		}
		
		Judge_Graphic_Obj_Apply_Opt(Obj,OPT_CHANGE);
		Judge_Graphic_Obj_Unlock(Obj);
}

void Judge_Graphic_Obj_Set_Pos(Judge_Graphic_Obj_Handle Obj,uint32_t X,uint32_t Y)
{   
	if(!Obj)
		return;
		Judge_Graphic_Obj_Set_X(Obj,X);
		Judge_Graphic_Obj_Set_Y(Obj,Y);
}

uint32_t Judge_Graphic_Obj_Get_Center_X(Judge_Graphic_Obj_Handle Obj)
{   
		if(!Obj)
			return 0;
		Judge_Graphic_Obj_Lock(Obj);
		uint32_t X = 0;
		
		if(Obj->Graphic_Data.graphic_tpye == RECTANGLE||Obj->Graphic_Data.graphic_tpye == LINE)
		{
				X = (Obj->Graphic_Data.start_x+Obj->Graphic_Data.graphic_union.graphic_data.end_x)/2;
		}
		else if(Obj->Graphic_Data.graphic_tpye == ELLIPSE||Obj->Graphic_Data.graphic_tpye == ARC||
						Obj->Graphic_Data.graphic_tpye == CIRCLE)
		{
				X = Obj->Graphic_Data.start_x;
		}
		
		Judge_Graphic_Obj_Unlock(Obj);
		return X;
}

uint32_t Judge_Graphic_Obj_Get_Center_Y(Judge_Graphic_Obj_Handle Obj)
{   
		if(!Obj)
			return 0;
		Judge_Graphic_Obj_Lock(Obj);
		uint32_t Y;
	
		if(Obj->Graphic_Data.graphic_tpye == RECTANGLE||Obj->Graphic_Data.graphic_tpye == LINE)
		{
				Y = (Obj->Graphic_Data.start_y+Obj->Graphic_Data.graphic_union.graphic_data.end_y)/2;
		}
		else if(Obj->Graphic_Data.graphic_tpye == ELLIPSE||Obj->Graphic_Data.graphic_tpye == ARC||
						Obj->Graphic_Data.graphic_tpye == CIRCLE)
		{
				Y = Obj->Graphic_Data.start_y;
		}
		
		Judge_Graphic_Obj_Unlock(Obj);
		return Y;
}

void Judge_Graphic_Obj_Set_Center_X(Judge_Graphic_Obj_Handle Obj,uint32_t Cx)
{   
	  if(!Obj)
			return;
		Judge_Graphic_Obj_Lock(Obj);
		
		if(Obj->Graphic_Data.graphic_tpye == RECTANGLE||Obj->Graphic_Data.graphic_tpye == LINE)
		{
				uint32_t W = Obj->Graphic_Data.graphic_union.graphic_data.end_x - Obj->Graphic_Data.start_x;
				Obj->Graphic_Data.start_x = Cx - W/2;
				Obj->Graphic_Data.graphic_union.graphic_data.end_x = Cx + W/2;
		}
		else if(Obj->Graphic_Data.graphic_tpye == ELLIPSE||Obj->Graphic_Data.graphic_tpye == ARC||
						Obj->Graphic_Data.graphic_tpye == CIRCLE)
		{
				Obj->Graphic_Data.start_x = Cx;
		}
		else if(Obj->Graphic_Data.graphic_tpye == INTEGER||Obj->Graphic_Data.graphic_tpye == FLOAT)
		{
				uint8_t Digits_Count = Get_Int_Digits_Count(Obj->Graphic_Data.graphic_union.number);
				uint32_t W = Obj->Graphic_Data.start_angle * Digits_Count;
				Obj->Graphic_Data.start_x = Cx - W/2;
		}
		else
		{
				uint32_t Character_Addr = (uint32_t)Obj+sizeof(Judge_Graphic_Obj_t);
				uint8_t Character_Len = strlen((char*)Character_Addr);
				uint32_t W = Obj->Graphic_Data.start_angle * Character_Len;
				Obj->Graphic_Data.start_x = Cx - W/2;
		}
		
		Judge_Graphic_Obj_Apply_Opt(Obj,OPT_CHANGE);
		Judge_Graphic_Obj_Unlock(Obj);
}

void Judge_Graphic_Obj_Set_Center_Y(Judge_Graphic_Obj_Handle Obj,uint32_t Cy)
{   
	  if(!Obj)
			return ;

		Judge_Graphic_Obj_Lock(Obj);
		
		if(Obj->Graphic_Data.graphic_tpye == RECTANGLE||Obj->Graphic_Data.graphic_tpye == LINE)
		{
				uint32_t H = Obj->Graphic_Data.graphic_union.graphic_data.end_y - Obj->Graphic_Data.start_y;
				Obj->Graphic_Data.start_y = Cy - H/2;
				Obj->Graphic_Data.graphic_union.graphic_data.end_y = Cy + H/2;
		}
		else if(Obj->Graphic_Data.graphic_tpye == ELLIPSE||Obj->Graphic_Data.graphic_tpye == ARC||
						Obj->Graphic_Data.graphic_tpye == CIRCLE)
		{
				Obj->Graphic_Data.start_y = Cy;
		}
		else
		{
				uint32_t H = Obj->Graphic_Data.start_angle;
				Obj->Graphic_Data.start_y = Cy + H/2;
		}
		
		Judge_Graphic_Obj_Apply_Opt(Obj,OPT_CHANGE);
		Judge_Graphic_Obj_Unlock(Obj);
}

void Judge_Graphic_Obj_Set_Center_Pos(Judge_Graphic_Obj_Handle Obj,uint32_t Cx,uint32_t Cy)
{    
	  if(!Obj)
			return;
		Judge_Graphic_Obj_Set_Center_X(Obj,Cx);
		Judge_Graphic_Obj_Set_Center_Y(Obj,Cy);
}

uint32_t Judge_Graphic_Obj_Get_Radius_A(Judge_Graphic_Obj_Handle Obj)
{   
	  if(!Obj)
			return 0;
		Judge_Graphic_Obj_Lock(Obj);
	
		uint32_t Ra = 0;
		if(Obj->Graphic_Data.graphic_tpye == ELLIPSE||Obj->Graphic_Data.graphic_tpye == ARC)
		{
				Ra = Obj->Graphic_Data.graphic_union.graphic_data.end_x;
		}
		else if(Obj->Graphic_Data.graphic_tpye == CIRCLE)
		{
				Ra = Obj->Graphic_Data.graphic_union.graphic_data.radius;
		}
		else if(Obj->Graphic_Data.graphic_tpye == RECTANGLE||Obj->Graphic_Data.graphic_tpye == LINE)
		{
				Ra = (Obj->Graphic_Data.graphic_union.graphic_data.end_x-Obj->Graphic_Data.start_x)/2;
		}
		
		Judge_Graphic_Obj_Unlock(Obj);
		return Ra;
}

uint32_t Judge_Graphic_Obj_Get_Radius_B(Judge_Graphic_Obj_Handle Obj)
{   
	  if(!Obj)
			return 0;
		Judge_Graphic_Obj_Lock(Obj);
		
		uint32_t Rb;
		if(Obj->Graphic_Data.graphic_tpye == ELLIPSE||Obj->Graphic_Data.graphic_tpye == ARC)
		{
				Rb = Obj->Graphic_Data.graphic_union.graphic_data.end_y;
		}
		else if(Obj->Graphic_Data.graphic_tpye == CIRCLE)
		{
				Rb = Obj->Graphic_Data.graphic_union.graphic_data.radius;
		}
		else if(Obj->Graphic_Data.graphic_tpye == RECTANGLE||Obj->Graphic_Data.graphic_tpye == LINE)
		{
				Rb = (Obj->Graphic_Data.graphic_union.graphic_data.end_y-Obj->Graphic_Data.start_y)/2;
		}
		
		Judge_Graphic_Obj_Unlock(Obj);
		return Rb;
}

void Judge_Graphic_Obj_Set_Radius_A(Judge_Graphic_Obj_Handle Obj,uint32_t Ra)
{   
	  if(!Obj)
			return;
		Judge_Graphic_Obj_Lock(Obj);
	
		if(Obj->Graphic_Data.graphic_tpye == ELLIPSE||Obj->Graphic_Data.graphic_tpye == ARC)
		{
				Obj->Graphic_Data.graphic_union.graphic_data.end_x = Ra;
				
		}
		else if(Obj->Graphic_Data.graphic_tpye == CIRCLE)
		{
			 Obj->Graphic_Data.graphic_union.graphic_data.radius = Ra;
		}
		else if(Obj->Graphic_Data.graphic_tpye == RECTANGLE||Obj->Graphic_Data.graphic_tpye == LINE)
		{
				uint32_t Cx = (Obj->Graphic_Data.start_x+Obj->Graphic_Data.graphic_union.graphic_data.end_x)/2;
				Obj->Graphic_Data.start_x = Cx - Ra/2;
				Obj->Graphic_Data.graphic_union.graphic_data.end_x = Cx + Ra/2;
		}
		
		Judge_Graphic_Obj_Apply_Opt(Obj,OPT_CHANGE);
		Judge_Graphic_Obj_Unlock(Obj);
}

void Judge_Graphic_Obj_Set_Radius_B(Judge_Graphic_Obj_Handle Obj,uint32_t Rb)
{   
	  if(!Obj)
			return;
		Judge_Graphic_Obj_Lock(Obj);
		
		if(Obj->Graphic_Data.graphic_tpye == ELLIPSE||Obj->Graphic_Data.graphic_tpye == ARC)
		{
				Obj->Graphic_Data.graphic_union.graphic_data.end_y = Rb;
		}
		else if(Obj->Graphic_Data.graphic_tpye == CIRCLE)
		{
				Obj->Graphic_Data.graphic_union.graphic_data.radius = Rb;
		}
		else if(Obj->Graphic_Data.graphic_tpye == RECTANGLE||Obj->Graphic_Data.graphic_tpye == LINE)
		{
				uint32_t Cy = (Obj->Graphic_Data.start_y+Obj->Graphic_Data.graphic_union.graphic_data.end_y)/2;
				Obj->Graphic_Data.start_x = Cy - Rb/2;
				Obj->Graphic_Data.graphic_union.graphic_data.end_x = Cy + Rb/2;
		}
		
		Judge_Graphic_Obj_Apply_Opt(Obj,OPT_CHANGE);
		Judge_Graphic_Obj_Unlock(Obj);
}

uint32_t Judge_Graphic_Obj_Get_Start_Angle(Judge_Graphic_Obj_Handle Obj,uint32_t Start_Angle)
{   
		if(!Obj)
			return 0;
		Judge_Graphic_Obj_Lock(Obj);
		
		uint32_t As = 0;
		if(Obj->Graphic_Data.graphic_tpye == ARC)
		{
				As = Obj->Graphic_Data.start_angle;
		}
		
		Judge_Graphic_Obj_Unlock(Obj);
		return As;
}

uint32_t Judge_Graphic_Obj_Get_End_Angle(Judge_Graphic_Obj_Handle Obj,uint32_t End_Angle)
{
	  if(!Obj)
			return 0;
		Judge_Graphic_Obj_Lock(Obj);
	
		uint32_t Ae = 0;
		if(Obj->Graphic_Data.graphic_tpye == ARC)
		{
				Ae = Obj->Graphic_Data.end_angle;
		}
		
		Judge_Graphic_Obj_Unlock(Obj);
		return Ae;
} 


void Judge_Graphic_Obj_Set_Start_Angle(Judge_Graphic_Obj_Handle Obj,uint32_t Start_Angle)
{   
	  if(!Obj)
			return;
		if(Obj->Graphic_Data.graphic_tpye == ARC)
		{
				Judge_Graphic_Obj_Lock(Obj);
			
				Obj->Graphic_Data.start_angle = Start_Angle;
			
				Judge_Graphic_Obj_Apply_Opt(Obj,OPT_CHANGE);
				Judge_Graphic_Obj_Unlock(Obj);
		}
}

void Judge_Graphic_Obj_Set_End_Angle(Judge_Graphic_Obj_Handle Obj,uint32_t End_Angle)
{   
	  if(!Obj)
			return;
		if(Obj->Graphic_Data.graphic_tpye == ARC)
		{
				Judge_Graphic_Obj_Lock(Obj);
			
				Obj->Graphic_Data.end_angle = End_Angle;
			
				Judge_Graphic_Obj_Apply_Opt(Obj,OPT_CHANGE);
				Judge_Graphic_Obj_Unlock(Obj);
		}
} 

void Judge_Graphic_Obj_Set_Radius(Judge_Graphic_Obj_Handle Obj,uint32_t R)
{   
	  if(!Obj)
			return;
	
		Judge_Graphic_Obj_Set_Radius_B(Obj,R);
		Judge_Graphic_Obj_Set_Radius_A(Obj,R);
	
}

void Judge_Graphic_Obj_Set_Line_Width(Judge_Graphic_Obj_Handle Obj,uint32_t W)
{
	  if(!Obj)
			return;
		Judge_Graphic_Obj_Lock(Obj);
	
		Obj->Graphic_Data.width = W;
		Judge_Graphic_Obj_Apply_Opt(Obj,OPT_CHANGE);
	
		Judge_Graphic_Obj_Unlock(Obj);
}

void Judge_Graphic_Obj_Set_Val(Judge_Graphic_Obj_Handle Obj,int32_t Val)
{
		if(!Obj)
			return;
		
		if(Obj->Graphic_Data.graphic_tpye == FLOAT||Obj->Graphic_Data.graphic_tpye == INTEGER)
		{
				Judge_Graphic_Obj_Lock(Obj);
			
				Obj->Graphic_Data.graphic_union.number = Val;
			
				Judge_Graphic_Obj_Apply_Opt(Obj,OPT_CHANGE);
				Judge_Graphic_Obj_Unlock(Obj);
		}
}

void Judge_Graphic_Obj_Del(Judge_Graphic_Obj_Handle Obj)
{
	  if(!Obj)
			return;
		Judge_Graphic_Obj_Lock(Obj);
		Judge_Graphic_Obj_Apply_Opt(Obj,OPT_DELETE);
		Judge_Graphic_Obj_Unlock(Obj);
}
/********图形创建函数**********/

Judge_Graphic_Obj_Handle Judge_Graphic_Ellipse_Create(uint32_t Cx,uint32_t Cy,uint32_t Ra,uint32_t Rb,uint32_t W,Judge_Graphic_Color_t Color)
{
		Judge_Graphic_Obj_Handle Ellipse = pvPortMalloc(sizeof(Judge_Graphic_Obj_t));
		if(!Ellipse)
			return NULL;
		
		if(!Judge_Graphic_Obj_Init(Ellipse))
		{
			vPortFree(Ellipse);
			return NULL;
		}
	  
		Ellipse->Graphic_Data.graphic_tpye = ELLIPSE;
		Ellipse->Graphic_Data.start_x = Cx - Ra/2;
		Ellipse->Graphic_Data.start_y = Cy - Ra/2;
		Ellipse->Graphic_Data.graphic_union.graphic_data.end_x = (uint32_t)Cx + Ra/2;
		Ellipse->Graphic_Data.graphic_union.graphic_data.end_y = (uint32_t)Cy + Rb/2;	
		Ellipse->Graphic_Data.width = W;
		Ellipse->Graphic_Data.color = Color;
	
		Judge_Graphic_Obj_Apply_Opt(Ellipse,OPT_ADD);
		Judge_Graphic_Obj_Unlock(Ellipse);
		return  Ellipse;
}

Judge_Graphic_Obj_Handle Judge_Graphic_Circle_Create(uint32_t Cx,uint32_t Cy,uint32_t R,uint32_t W,Judge_Graphic_Color_t Color)
{
		Judge_Graphic_Obj_Handle Circle = pvPortMalloc(sizeof(Judge_Graphic_Obj_t));
		if(!Circle)
			return NULL;
		
		if(!Judge_Graphic_Obj_Init(Circle))
		{
			vPortFree(Circle);
			return NULL;
		}
		
		Circle->Graphic_Data.graphic_tpye = CIRCLE;
		Circle->Graphic_Data.start_x = Cx;
		Circle->Graphic_Data.start_y = Cy;
		Circle->Graphic_Data.graphic_union.graphic_data.radius = R;
		Circle->Graphic_Data.width = W;
		Circle->Graphic_Data.color = Color;
	
		Judge_Graphic_Obj_Apply_Opt(Circle,OPT_ADD);
		Judge_Graphic_Obj_Unlock(Circle);
		return Circle;
}

Judge_Graphic_Obj_Handle Judge_Graphic_Line_Create(uint32_t X1,uint32_t Y1,uint32_t X2,uint32_t Y2,uint32_t W,Judge_Graphic_Color_t Color)
{
		Judge_Graphic_Obj_Handle Line = pvPortMalloc(sizeof(Judge_Graphic_Obj_t));
		if(!Line)
			return NULL;
		
		if(!Judge_Graphic_Obj_Init(Line))
		{
			vPortFree(Line);
			return NULL;
		}
		
		Line->Graphic_Data.graphic_tpye = LINE;
		Line->Graphic_Data.start_x = X1;
		Line->Graphic_Data.start_y = Y1;
		Line->Graphic_Data.graphic_union.graphic_data.end_x = X2;
		Line->Graphic_Data.graphic_union.graphic_data.end_y = Y2;	
		Line->Graphic_Data.width = W;
		Line->Graphic_Data.color = Color;
	
		Judge_Graphic_Obj_Apply_Opt(Line,OPT_ADD);
		Judge_Graphic_Obj_Unlock(Line);
		return Line;
}

Judge_Graphic_Obj_Handle Judge_Graphic_Rect_Create(uint32_t X1,uint32_t Y1,uint32_t X2,uint32_t Y2,uint32_t W,Judge_Graphic_Color_t Color)
{
		Judge_Graphic_Obj_Handle Rect = pvPortMalloc(sizeof(Judge_Graphic_Obj_t));
		if(!Rect)
			return NULL;
		
		if(!Judge_Graphic_Obj_Init(Rect))
		{
			vPortFree(Rect);
			return NULL;
		}
		
		Rect->Graphic_Data.graphic_tpye = RECTANGLE;
		Rect->Graphic_Data.start_x = X1;
		Rect->Graphic_Data.start_y = Y1;
		Rect->Graphic_Data.graphic_union.graphic_data.end_x = X2;
		Rect->Graphic_Data.graphic_union.graphic_data.end_y = Y2;	
		Rect->Graphic_Data.width = W;
		Rect->Graphic_Data.color = Color;
	
		Judge_Graphic_Obj_Apply_Opt(Rect,OPT_ADD);
		Judge_Graphic_Obj_Unlock(Rect);
		return Rect;
}

Judge_Graphic_Obj_Handle Judge_Graphic_Arc_Create(uint32_t A1,uint32_t A2,uint32_t Cx, uint32_t Cy,uint32_t R,uint32_t W,Judge_Graphic_Color_t Color)
{
		Judge_Graphic_Obj_Handle Arc = pvPortMalloc(sizeof(Judge_Graphic_Obj_t));
		if(!Arc)
			return NULL;
		
		if(!Judge_Graphic_Obj_Init(Arc))
		{
			vPortFree(Arc);
			return NULL;
		}
		
		Arc->Graphic_Data.graphic_tpye = ARC;
		Arc->Graphic_Data.start_angle = A1;
		Arc->Graphic_Data.end_angle = A2;
		Arc->Graphic_Data.start_x = Cx;
		Arc->Graphic_Data.start_y = Cy;
		Arc->Graphic_Data.graphic_union.graphic_data.end_x = R;
		Arc->Graphic_Data.graphic_union.graphic_data.end_y = R;
		Arc->Graphic_Data.width = W;
		Arc->Graphic_Data.color = Color;
	
		Judge_Graphic_Obj_Apply_Opt(Arc,OPT_ADD);
		Judge_Graphic_Obj_Unlock(Arc);
		return Arc;
}

Judge_Graphic_Obj_Handle Judge_Graphic_Float_Create(uint32_t X,uint32_t Y,uint32_t Font_Size,float Val,Judge_Graphic_Color_t Color)
{
		Judge_Graphic_Obj_Handle Float = pvPortMalloc(sizeof(Judge_Graphic_Obj_t));
		if(!Float)
			return NULL;
		
		if(!Judge_Graphic_Obj_Init(Float))
		{
			vPortFree(Float);
			return NULL;
		}
		
		Float->Graphic_Data.graphic_tpye = FLOAT;
		Float->Graphic_Data.start_x = X;
		Float->Graphic_Data.start_y = Y;
		Float->Graphic_Data.end_angle = 3;
		Float->Graphic_Data.color = Color;
		Float->Graphic_Data.start_angle = Font_Size<10?10:Font_Size;
		Float->Graphic_Data.width = Font_Size/10<4?4:Font_Size/10;
	
		Judge_Graphic_Obj_Apply_Opt(Float,OPT_ADD);
		Judge_Graphic_Obj_Unlock(Float);
		return Float;
}

Judge_Graphic_Obj_Handle Judge_Graphic_Integer_Create(uint32_t X,uint32_t Y,uint32_t Font_Size,int32_t Val,Judge_Graphic_Color_t Color)
{
		Judge_Graphic_Obj_Handle Int = pvPortMalloc(sizeof(Judge_Graphic_Obj_t));
		if(!Int)
			return NULL;
		
		if(!Judge_Graphic_Obj_Init(Int))
		{
			vPortFree(Int);
			return NULL;
		}
		
		Int->Graphic_Data.graphic_tpye = INTEGER;
		Int->Graphic_Data.start_x = X;
		Int->Graphic_Data.start_y = Y;
		Int->Graphic_Data.color = Color;
		Int->Graphic_Data.start_angle = Font_Size<10?10:Font_Size;
		Int->Graphic_Data.width = Font_Size/10<4?4:Font_Size/10;
		
		Int->Graphic_Data.graphic_union.number = Val;

		Judge_Graphic_Obj_Apply_Opt(Int,OPT_ADD);
		Judge_Graphic_Obj_Unlock(Int);
		return Int;
}

Judge_Graphic_Obj_Handle Judge_Graphic_Character_Create(uint32_t X,uint32_t Y,uint32_t Font_Size,char* Str,Judge_Graphic_Color_t Color)
{    
	  if(!Str)
			return NULL;
	
		Judge_Graphic_Obj_Handle Character = pvPortMalloc(sizeof(Judge_Graphic_Obj_t)+JUDGE_GRAPHIC_CHARACTER_LEN);
		if(!Character)
			return NULL;
		
		if(!Judge_Graphic_Obj_Init(Character))
		{
			vPortFree(Character);
			return NULL;
		}
		
		uint32_t Character_Len = strlen(Str);
		Character->Graphic_Data.graphic_tpye = CHARACTER;
		Character->Graphic_Data.start_x = X;
		Character->Graphic_Data.start_y = Y;
		Character->Graphic_Data.end_angle = Character_Len;
		Character->Graphic_Data.color = Color;
		Character->Graphic_Data.start_angle = Font_Size<10?10:Font_Size;
		Character->Graphic_Data.width = Font_Size/10<2?2:Font_Size/10;
			
		char* Character_Addr = (char*)((uint32_t)Character+sizeof(Judge_Graphic_Obj_t));
		memset(Character_Addr,0,JUDGE_GRAPHIC_CHARACTER_LEN);
		memcpy(Character_Addr,Str,Character_Len);
	
		Judge_Graphic_Obj_Apply_Opt(Character,OPT_ADD);
		Judge_Graphic_Obj_Unlock(Character);
		return Character;
}


void Judge_Graphic_Init()
{
		DJI_Judge_Graphic.Judge_Graphic_Obj_Counter = 1;
		DJI_Judge_Graphic.Current_Step = 0;
		DJI_Judge_Graphic.Judge_Graphic_Character_Queue = xQueueCreate(50,sizeof(Judge_Graphic_Obj_Handle));
		DJI_Judge_Graphic.Judge_Graphic_CommObj_Queue = xQueueCreate(50,sizeof(Judge_Graphic_Obj_Handle));
}

static uint8_t Judge_Graphic_Character_Send(void)
{
		Judge_Graphic_Obj_Handle Judge_Graphic_Character;
		if(xQueueReceive(DJI_Judge_Graphic.Judge_Graphic_Character_Queue,&Judge_Graphic_Character,0) != pdTRUE)
				return 0;
		
		uint8_t* Send_Buff = pvPortMalloc(JUDGE_GRAPHIC_DATA_LEN+JUDGE_GRAPHIC_CHARACTER_LEN);
		if(!Send_Buff)
				return 0;
		
		Judge_Graphic_Obj_Lock(Judge_Graphic_Character);
		memcpy(Send_Buff,&Judge_Graphic_Character->Graphic_Data,JUDGE_GRAPHIC_DATA_LEN);
		uint32_t Character_Addr = (uint32_t)Judge_Graphic_Character+sizeof(Judge_Graphic_Obj_t);
		memset(Send_Buff+JUDGE_GRAPHIC_DATA_LEN,0,JUDGE_GRAPHIC_CHARACTER_LEN);
		memcpy(Send_Buff+JUDGE_GRAPHIC_DATA_LEN,(uint8_t*)(Character_Addr),JUDGE_GRAPHIC_CHARACTER_LEN);
		
		Judge_Graphic_Character->Graphic_Data.operate_tpye = OPT_NONE;
		Judge_Graphic_Obj_Unlock(Judge_Graphic_Character);
		
		Judge_Student_Data_Send(Send_Buff,JUDGE_GRAPHIC_DATA_LEN+JUDGE_GRAPHIC_CHARACTER_LEN,GRAPHIC_CMDID_CHARACTER,Target_Client);
		vPortFree(Send_Buff);
		return 1;
}

static uint8_t Judge_Graphic_CommObj_Send(void)
{
		uint32_t Total = uxQueueMessagesWaiting(DJI_Judge_Graphic.Judge_Graphic_CommObj_Queue);
		if(Total == 0)
			return 0;
		//决定要发送多少个对象
		uint8_t Num_To_Send;
		uint16_t CmdID = 0;
		if(Total>=7)
		{
			Num_To_Send = 7;
			CmdID = GRAPHIC_CMDID_SEVEN;
		}
		else if(Total>=5)
		{
			Num_To_Send = 5;
			CmdID = GRAPHIC_CMDID_FIVE;
		}
		else if(Total>=2)
		{
			Num_To_Send = 2;
			CmdID = GRAPHIC_CMDID_TWO;
		}
		else if(Total>=1)
		{
			Num_To_Send = 1;
			CmdID = GRAPHIC_CMDID_ONE;
		}
		
		//申请发送缓冲区
		uint8_t* Send_Buff = pvPortMalloc(JUDGE_GRAPHIC_DATA_LEN*Num_To_Send);
		if(!Send_Buff)
			return 0;
		
		for(uint32_t i = 0; i < Num_To_Send; i++)
		{
				//从队列中取出对象，如果失败，发送一个空操作
				Judge_Graphic_Obj_Handle Judge_Graphic_CommObj;
				if(xQueueReceive(DJI_Judge_Graphic.Judge_Graphic_CommObj_Queue,&Judge_Graphic_CommObj,0) == pdTRUE)
				{
						//锁定对象
						Judge_Graphic_Obj_Lock(Judge_Graphic_CommObj);
						//拷贝对象数据到发送缓冲区
						memcpy(&Send_Buff[i*JUDGE_GRAPHIC_DATA_LEN],&Judge_Graphic_CommObj->Graphic_Data,JUDGE_GRAPHIC_DATA_LEN);
						//如果该对象需要被删除，那么直接删除，否则解锁对象
						if(Judge_Graphic_CommObj->Graphic_Data.operate_tpye==OPT_DELETE)
						{
							//删除互斥量
							vSemaphoreDelete(Judge_Graphic_CommObj->Graphic_Obj_Mutex);
							//释放对象占用的内存
							vPortFree(Judge_Graphic_CommObj);
						}
						else
						{
							Judge_Graphic_CommObj->Graphic_Data.operate_tpye = OPT_NONE;
							Judge_Graphic_Obj_Unlock(Judge_Graphic_CommObj);
						}
				}
				else
				{
						Judge_Graphic_CommObj = (Judge_Graphic_Obj_Handle)(&Send_Buff[i*JUDGE_GRAPHIC_DATA_LEN]);
						Judge_Graphic_CommObj->Graphic_Data.operate_tpye = OPT_NONE;
				}
		}
		
		Judge_Student_Data_Send(Send_Buff,Num_To_Send*JUDGE_GRAPHIC_DATA_LEN,CmdID,Target_Client);
		vPortFree(Send_Buff);
		return 1;
}


void Judge_Graphic_Handler(void)
{
		if(Is_Judge_Online())
		{
				if(DJI_Judge_Graphic.Current_Step == 1)
				{
						if(Judge_Graphic_CommObj_Send()==0)
							Judge_Graphic_Character_Send();
						
						DJI_Judge_Graphic.Current_Step = 0;
				}
				else
				{
						if(Judge_Graphic_Character_Send()==0)
							Judge_Graphic_CommObj_Send();
						
						DJI_Judge_Graphic.Current_Step = 1;
				}
		}
}