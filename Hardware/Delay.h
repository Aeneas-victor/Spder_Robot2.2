/**
  ******************************************************************************
  * @file    Delay.h
  * @author  李航兵
  * @version V1.2
  * @date    18-June-2018
  * @brief   这个头文件包含了STM32F407ZGT6开发板的延时相关的函数. 
  ******************************************************************************
  * @attention
  *		官方库文件里有参考代码
  *		为保证延时系统的精准，要求系统主频率越高越好
  *		为保证延时系统的完整支持，系统主频率至少8MHz
  *		使用CM4系统定时器，中断优先级最低
  *
  * @更新说明
  *		支持us、ms级延时，且精度大大提高
  *		支持OS，基本不影响OS的使用（下个版本添加）
  *
  ******************************************************************************
  */
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DELAY_H
#define __DELAY_H
 
 
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_rcc.h"
#include "core_cm4.h"
#include "stm32f4xx_it.h"
#include "misc.h"
 

 
void Delay_Init(void);					//延时系统初始化
void Delay_Us(uint16_t us);				//微秒延时
void Delay_Ms(uint16_t ms);				//毫秒延时
void Delay_Us_2(uint32_t us);				//微秒延时，范围更大
 
 
 
 
#endif /* __DELAY_H */


