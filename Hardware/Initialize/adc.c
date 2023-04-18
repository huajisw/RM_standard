#include "adc.h"

static void Power_Limit_Init(void);
static void Temperture_Init(void);

static uint16_t Get_Temperature_ADC(uint8_t ch);
static int Get_Temperature_Average(int num);
static void Temperature_ADC_Reset(void);
static int Get_Power_Average(int num);
static int Power_Detect(void);
static int Get_Motor_Power_Average(int num);
static int Motor_Power_Detect(void);

void Adc_Init(void)
{
		Temperture_Init();
		Power_Limit_Init();
}

void Oled_Key_Init(void)
{
    ADC_InitTypeDef adc;
    GPIO_InitTypeDef gpio;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
   
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
//	GPIO_PinAFConfig(GPIOH,GPIO_PinSource9,GPIO_AF_TIM12);
		gpio.GPIO_Pin = GPIO_Pin_6;
		gpio.GPIO_Mode = GPIO_Mode_AN;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(GPIOA,&gpio);

		ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInit(&ADC_CommonInitStructure);
	
    adc.ADC_Resolution = ADC_Resolution_12b;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = ENABLE;
    adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
//  adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1,&adc);
    
   	ADC_RegularChannelConfig(ADC1,ADC_Channel_6,1,ADC_SampleTime_15Cycles);
		
		ADC_Cmd(ADC1,ENABLE);
		ADC_SoftwareStartConv(ADC1);
}

uint16_t Get_KEY_ADC(void)
{

    ADC_ClearFlag(ADC1,ADC_FLAG_STRT|ADC_FLAG_OVR|ADC_FLAG_EOC);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_15Cycles);

    ADC_SoftwareStartConv(ADC1);

    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
    {
        ;
    }
    return ADC_GetConversionValue(ADC1);
}



static void Power_Limit_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		ADC_InitTypeDef ADC_InitStructure;
	
    RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC3, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC3, DISABLE);
	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
		GPIO_Init(GPIOF, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
		GPIO_Init(GPIOF, &GPIO_InitStructure);
		
		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC3, &ADC_InitStructure);
	
		ADC_RegularChannelConfig(ADC3,ADC_Channel_9,1,ADC_SampleTime_144Cycles);
		ADC_RegularChannelConfig(ADC3,ADC_Channel_14,1,ADC_SampleTime_144Cycles);
	
		ADC_Cmd(ADC3, ENABLE);	//使能指定的ADC3
	 
		ADC_SoftwareStartConv(ADC3);		//使能指定的ADC3的软件转换启动功能	
}

static void Temperture_Init(void)
{
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);
	
		ADC_CommonInitStructure.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled;
		ADC_CommonInitStructure.ADC_Mode=ADC_Mode_Independent;
		ADC_CommonInitStructure.ADC_Prescaler=ADC_Prescaler_Div4;
		ADC_CommonInitStructure.ADC_TwoSamplingDelay=ADC_TwoSamplingDelay_5Cycles;;
	  ADC_CommonInit(&ADC_CommonInitStructure);
	
		ADC_TempSensorVrefintCmd(ENABLE);//使能温度传感器
	
    ADC_RegularChannelConfig(ADC1,ADC_Channel_18,1,ADC_SampleTime_15Cycles);


		ADC_Cmd(ADC1,ENABLE);	
		ADC_SoftwareStartConv(ADC1);			
}

static int Motor_Power_Detect(void)
{
		int AD_Value;
		ADC_RegularChannelConfig(ADC3, ADC_Channel_14, 1, ADC_SampleTime_144Cycles);
    ADC_SoftwareStartConv(ADC3);
    while(!ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC));
    AD_Value = ADC_GetConversionValue(ADC3);
		return AD_Value;
}

static int Get_Motor_Power_Average(int num)
{
	int pow_sum=0;
	int i;
	for(i=0;i<num;i++)
	{
		pow_sum+=Motor_Power_Detect();
	}
	return pow_sum/num;
}

float Motor_Power_Read(int num)
{
	int power_read;
	float Board_Power;
	power_read=Get_Motor_Power_Average(num);
	Board_Power = (float)(power_read*(3.3f/4096.0f));
	Board_Power = (Board_Power/10.0f*110.0f);
	return Board_Power;	
}

static int Power_Detect(void)
{
    int AD_Value;
		ADC_RegularChannelConfig(ADC3, ADC_Channel_9, 1, ADC_SampleTime_144Cycles);
    ADC_SoftwareStartConv(ADC3);
    while(!ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC));
    AD_Value = ADC_GetConversionValue(ADC3);
		return AD_Value;
	
}

static int Get_Power_Average(int num)
{
	int pow_sum=0;
	int i;
	for(i=0;i<num;i++)
	{
		pow_sum+=Power_Detect();
	}
	return pow_sum/num;
}

float Board_Power_Read(int num)
{
	int power_read;
	float Board_Power;
	power_read=Get_Power_Average(num);
	Board_Power = ((float)power_read*(3.3f/4096.0f));
	Board_Power = (Board_Power/4.7f*104.7f);
	return Board_Power;	
}

static void Temperature_ADC_Reset(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

}

static int Get_Temperature_Average(int num)
{
	int tem_sum=0;
	int i;
	for(i=0;i<num;i++)
	{
		tem_sum+=Get_Temperature_ADC(ADC_Channel_18);;
	}
	return tem_sum/num;
}

float Cpu_Temperature_Read(int num)
{
	int tem_read;
	float Board_Temperature;
	Temperature_ADC_Reset();
	tem_read=Get_Temperature_Average(num);
	Board_Temperature = (float)((tem_read*(3300.0f/4096.0f) - 760.0f) / 2.5f + 25.0f);
	return Board_Temperature;	
}

static uint16_t Get_Temperature_ADC(uint8_t ch)
{

    ADC_ClearFlag(ADC1,ADC_FLAG_STRT|ADC_FLAG_OVR|ADC_FLAG_EOC);
    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_15Cycles);

    ADC_SoftwareStartConv(ADC1);

    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
    {
        ;
    }
    return ADC_GetConversionValue(ADC1);
}

