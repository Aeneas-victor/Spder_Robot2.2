#ifndef __SERIAL_H__
#define __SERIAL_H__
#include "stm32f4xx.h"
#include<stdio.h>
#include<stdarg.h>
#include<string.h>
#define BUFFER_SIZE 128
void Printf_Init(void);
void USART2_Send_Byte(uint8_t ch);
void USART2_Send_Array(uint8_t *Array,uint16_t Length);
void aprintf(const char* format, ...);
#endif
