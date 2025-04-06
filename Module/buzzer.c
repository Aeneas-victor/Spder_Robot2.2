#include "buzzer.h"
#include "stm32f4xx.h"
#include "Delay.h"
void Buzzer_Init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	
	GPIO_InitTypeDef buzzer;
	buzzer.GPIO_Mode=GPIO_Mode_OUT;
	buzzer.GPIO_OType=GPIO_OType_PP;
	buzzer.GPIO_Pin=GPIO_Pin_3;
	buzzer.GPIO_PuPd=GPIO_PuPd_UP;
	buzzer.GPIO_Speed=GPIO_High_Speed;
	GPIO_Init(GPIOF,&buzzer);
	
}
void BuzzerDebug()
{
	GPIO_SetBits(GPIOF,GPIO_Pin_3);
	Delay_Ms(100);
	GPIO_ResetBits(GPIOF,GPIO_Pin_3);
}