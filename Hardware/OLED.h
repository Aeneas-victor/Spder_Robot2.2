/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : OLED.h
  * @brief          : OLED显示屏定义
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024-aeneas
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE BEGIN Header */
#ifndef __OLED_H
#define __OLED_H
#include<stdint.h>
#ifdef __cplusplus
extern "C"{
#endif
	
#define RCC_AHB1Clock_PORT     RCC_AHB1Periph_GPIOB
#define OLED_GPIO_PORT         GPIOB
#define OLED_SCL_PIN           GPIO_Pin_8
#define OLED_SDA_PIN           GPIO_Pin_9

extern uint8_t debug_Line_Number;
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line,uint8_t Column,char Char);
void OLED_ShowString(uint8_t Line,uint8_t Column,char *string);
void OLED_ShowNum(uint8_t Line,uint8_t Column,uint32_t Number,uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column,int32_t Number,uint8_t Length);
void Show_HexNum(uint8_t Line,uint8_t Column,uint32_t Number,uint8_t Length);
void OLED_ShowBinNum(uint8_t Line,uint8_t Column,uint32_t Number,uint8_t Length);
void debug(char* string);


#ifdef __cplusplus
}
#endif

#endif
