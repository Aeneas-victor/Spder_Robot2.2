/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-02-18     杨金鹏       the first version
 */
#include "aboard.h"
#include "PrintfSerial.h"
#include "GaitController.h"
#include "Frame.h"
#include "Delay.h"
extern uint8_t bule_actnumber;
extern Action actionenum[15];
extern Action kinematicsnum[15];
extern Action* instruct;
void Board_Init(void)
{
	
	GPIO_InitTypeDef keygpio;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	keygpio.GPIO_Mode=GPIO_Mode_IN;
	keygpio.GPIO_OType=GPIO_OType_PP;
	keygpio.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_3;
	keygpio.GPIO_PuPd=GPIO_PuPd_UP;
	keygpio.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOE,&keygpio);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource3);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource4);
	
	EXTI_InitTypeDef keyEXTI;
	keyEXTI.EXTI_Line=EXTI_Line3|EXTI_Line4;
	keyEXTI.EXTI_LineCmd=ENABLE;
	keyEXTI.EXTI_Mode=EXTI_Mode_Interrupt;
	keyEXTI.EXTI_Trigger=EXTI_Trigger_Falling;
	EXTI_Init(&keyEXTI);
	
	
	 NVIC_InitTypeDef nvicKey;

    // 配置 EXTI3 中断
    nvicKey.NVIC_IRQChannel = EXTI3_IRQn;
    nvicKey.NVIC_IRQChannelCmd = ENABLE;
    nvicKey.NVIC_IRQChannelPreemptionPriority = 2;
    nvicKey.NVIC_IRQChannelSubPriority = 3;
    NVIC_Init(&nvicKey);

    // 配置 EXTI4 中断
    nvicKey.NVIC_IRQChannel = EXTI4_IRQn;
    nvicKey.NVIC_IRQChannelCmd = ENABLE;
    nvicKey.NVIC_IRQChannelPreemptionPriority = 2;
    nvicKey.NVIC_IRQChannelSubPriority = 3;
    NVIC_Init(&nvicKey);
}

void EXTI4_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) == Bit_RESET)
        {
            bule_actnumber = 14;
#ifdef _DEBUG
			aprintf("storage mode...\n");
#endif
        }
        EXTI_ClearITPendingBit(EXTI_Line4); // 清除中断标志
    }
}

void EXTI3_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line3) != RESET)
    {
        if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) == Bit_RESET)
        {
			if(instruct==actionenum)
			{
				//bule_actnumber = 14;
				//Delay_Ms(200);
				Gait_Init();
				instruct=kinematicsnum;
				#ifdef _DEBUG
				aprintf("kinematics mode...\n");
				#endif
			}else {
				bule_actnumber = 14;
				Delay_Ms(100);
				bule_actnumber=0;
				Robot_Action();	
				instruct=actionenum;
				#ifdef _DEBUG
				aprintf("action mode ...\n");
				#endif
			}
            

        }
        EXTI_ClearITPendingBit(EXTI_Line3); // 清除中断标志
    }
}
