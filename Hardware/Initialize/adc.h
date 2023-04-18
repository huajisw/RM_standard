#ifndef ADC_H
#define ADC_H
#include "stm32f4xx.h"

void Adc_Init(void);
void Oled_Key_Init(void);


uint16_t Get_KEY_ADC(void);

float Motor_Power_Read(int num);
float Board_Power_Read(int num);
float Cpu_Temperature_Read(int num);

#endif
