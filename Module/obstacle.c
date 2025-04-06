#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"
#include <stdio.h>
#include "obstacle.h"
volatile uint16_t adc_value = 0;

// 初始化ADC
void Obstacle_Init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

    // 使能GPIOC时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    // 配置DOUT1和DOUT2引脚为输入模式
    GPIO_InitStructure.GPIO_Pin = DOUT1_GPIO_PIN | DOUT2_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(DOUT1_GPIO_PORT, &GPIO_InitStructure);

    // 配置AOUT1和AOUT2引脚为模拟输入模式
    GPIO_InitStructure.GPIO_Pin = AOUT1_GPIO_PIN | AOUT2_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(AOUT1_GPIO_PORT, &GPIO_InitStructure);

    ADC_InitTypeDef ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;

    // 配置ADC通用设置
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);
    // 使能ADC1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // 配置ADC1
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    // 配置ADC通用设置
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);
	
    // 使能ADC
    ADC_Cmd(ADC1, ENABLE);
	
	NVIC_InitTypeDef adc1nvic;
	adc1nvic.NVIC_IRQChannel=ADC_IRQn;
	adc1nvic.NVIC_IRQChannelCmd=ENABLE;
	adc1nvic.NVIC_IRQChannelPreemptionPriority=2;
	adc1nvic.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&adc1nvic);
	
	ADC_Cmd(ADC1,ENABLE);
}
void Start_ADC_Conversion(uint8_t channel) {
    // 配置ADC通道
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_15Cycles);

    // 启动转换
    ADC_SoftwareStartConv(ADC1);
}

// ADC中断服务函数
void ADC_IRQHandler(void) {
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC)) {
        // 读取转换结果
        adc_value = ADC_GetConversionValue(ADC1);

        // 清除中断标志
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    }
}

// 获取最新ADC值
uint16_t Get_ADC_Value(void) {
    return adc_value;
}
