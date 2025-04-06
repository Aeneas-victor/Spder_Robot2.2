#ifndef __OBSTACLE_H__
#define __OBSTACLE_H__
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"
#include <stdio.h>

#define DOUT1_GPIO_PORT GPIOC
#define DOUT1_GPIO_PIN  GPIO_Pin_0
#define AOUT1_GPIO_PORT GPIOC
#define AOUT1_GPIO_PIN  GPIO_Pin_2
#define AOUT1_ADC_CHANNEL ADC_Channel_12

#define DOUT2_GPIO_PORT GPIOC
#define DOUT2_GPIO_PIN  GPIO_Pin_1
#define AOUT2_GPIO_PORT GPIOC
#define AOUT2_GPIO_PIN  GPIO_Pin_3
#define AOUT2_ADC_CHANNEL ADC_Channel_13
// 初始化ADC
extern volatile uint16_t adc_value;
void Obstacle_Init(void);

void Start_ADC_Conversion(uint8_t channel);

// ADC中断服务函数
void ADC_IRQHandler(void);
// 获取最新ADC值
uint16_t Get_ADC_Value(void);

#endif
