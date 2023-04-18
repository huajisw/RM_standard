#ifndef _OLED_LIST_TASK_H_
#define _OLED_LIST_TASK_H_

#include "main.h"

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"


void Oled_List_Task(void *pvParameters);
void oled_Show_Location(int x,int y);

#endif

