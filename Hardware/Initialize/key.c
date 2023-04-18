#include "key.h"

void key_init(void)//PB2
{
	GPIO_InitTypeDef GPIO_InitStructure;	
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

int KEY_Scan(void)
{
	return KEY_READ;
}

void Shoot_key_init(void)//PF0
{
	GPIO_InitTypeDef GPIO_InitStructure;	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOI, &GPIO_InitStructure);
}

void Laser_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE); //

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      
    GPIO_Init(GPIOG, &GPIO_InitStructure);            

    Laser_off();
}

void Laser_on(void)
{
    GPIO_SetBits(GPIOG, GPIO_Pin_13);
}
void Laser_off(void)
{
    GPIO_ResetBits(GPIOG, GPIO_Pin_13);
}