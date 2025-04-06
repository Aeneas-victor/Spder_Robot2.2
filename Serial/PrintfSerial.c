/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : PrintfSerial.c
  * @brief          : printf Serial file
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

const char space = ' ';
#include "PrintfSerial.h"
void Printf_Init(void)
{
	GPIO_InitTypeDef printfgpio;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//GPIOA 时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//USART 时钟
	
										 //初始化 GPIO
	printfgpio.GPIO_Mode=GPIO_Mode_AF;
	printfgpio.GPIO_OType=GPIO_OType_PP;
	printfgpio.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;
	printfgpio.GPIO_PuPd =GPIO_PuPd_UP;
	printfgpio.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&printfgpio);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);  //配置服用模式
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
									   //初始化USART 
	USART_DeInit(USART2);//接触该串口的其他配置
	USART_InitTypeDef usart2;
	usart2.USART_BaudRate=115200;
	usart2.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	usart2.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	usart2.USART_Parity =USART_Parity_No;
	usart2.USART_StopBits= USART_StopBits_1;
	usart2.USART_WordLength =USART_WordLength_8b;
	
	USART_Init(USART2,&usart2);
	USART_Cmd(USART2,ENABLE);
	
	while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET) {
        USART_ReceiveData(USART2);
    }
	
	USART_ClearFlag(USART2,USART_FLAG_RXNE);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_ClearFlag(USART2, USART_FLAG_TXE);
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	
	NVIC_InitTypeDef usart2nvic;
	usart2nvic.NVIC_IRQChannel=USART2_IRQn;
	usart2nvic.NVIC_IRQChannelCmd=ENABLE;
	usart2nvic.NVIC_IRQChannelPreemptionPriority=3;
	usart2nvic.NVIC_IRQChannelSubPriority=3;
	NVIC_Init(&usart2nvic);
}

void USART2_Send_Byte(uint8_t ch)
{
	USART_SendData(USART2,(uint8_t)ch);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
	
}

void USART2_Send_Array(uint8_t *Array,uint16_t Length)
{
	uint16_t i=0;
	for(;i<Length&&Array[i]!='\0';i++)
	{
		USART2_Send_Byte(Array[i]);
	}
}
/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    USART_SendData(USART2, (uint8_t)ch);
	
	while( RESET == USART_GetFlagStatus(USART2, USART_FLAG_TXE) ){}
	
    return ch;
}
void aprintf(const char* format,...)
{
    char buffer[BUFFER_SIZE];
    va_list args;
    va_start(args,format);
    vsnprintf(buffer,(unsigned int)sizeof(buffer),format,args);
    va_end(args);
    USART2_Send_Array((uint8_t*)buffer,strlen(buffer));
}
//void aprintf(const char* format, ...)
//{
//	va_list args;
//	va_start(args, format);
//	char buffer[BUFFER_SIZE]={0};
//	int buffer_index = 0;
//	while (*format!='\0'&&buffer_index<BUFFER_SIZE-1)
//	{
//		if (*format == '%')
//		{
//			format++;
//			int width=0;
//			int precision = -1;
//			if(*format >= '0' && *format <= '9')
//			{
//				width = 0;
//				while (*format >= '0' && *format <= '9')
//				{
//					width = width * 10 + (*format - '0');
//					format++;
//				}
//			}
//			if (*format == '.')
//			{
//				format++;
//				precision = 0;
//				if (*format >= '0' && *format <= '9')
//				{
//					while (*format >= '0' && *format < '9')
//					{
//						precision=precision*10 + (*format - '0');
//						format++;
//					}
//				}
//			}
//			if (*format == 'd')
//			{
//				int value = va_arg(args, int);
//				char svalue[10];
//				if (precision >= 0) {
//					snprintf(svalue, sizeof(svalue), "%.*d", precision, value);
//				}
//				else {
//					snprintf(svalue, sizeof(svalue), "%d", value);
//				}
//				int len = strlen(svalue);
//				if (width > len) {
//					int padding = width - len;
//					for (int i = 0; i < padding && buffer_index < BUFFER_SIZE - 1; i++) {
//						buffer[buffer_index++] = space;
//					}
//				}
//				for (int i = 0; svalue[i] != '\0' && buffer_index < BUFFER_SIZE - 1; i++) {
//					buffer[buffer_index++] = svalue[i];
//				}
//			}
//			else if (*format == 'f')
//			{
//				double value = va_arg(args, double);
//				char svalue[24];  
//				if (precision >= 0) {
//					snprintf(svalue, sizeof(svalue), "%.*f", precision, value);
//				}
//				else {
//					snprintf(svalue, sizeof(svalue), "%f", value);
//				}
//				int len = strlen(svalue);
//				if (width > len) {
//					int padding = width - len;
//					for (int i = 0; i < padding && buffer_index < BUFFER_SIZE - 1; i++) {
//						buffer[buffer_index++] = space;
//					}
//				}
//				for (int i = 0; svalue[i] != '\0' && buffer_index < BUFFER_SIZE - 1; i++) {
//					buffer[buffer_index++] = svalue[i];
//				}
//			}
//			else if(*format=='c')
//			{
//				char value=(char)va_arg(args,int);
//				if(width>1){
//					int padding =width-1;
//					for(int i=0;i<padding&&buffer_index<BUFFER_SIZE-1;i++)
//					{
//						buffer[buffer_index]=space;
//					}
//				}
//				if(buffer_index<BUFFER_SIZE-1)
//				{
//					buffer[buffer_index++]=value;
//				}
//			}
//			else if (*format == 's') {
//				char* value = va_arg(args, char*);
//				int len = strlen(value);
//				if (width > len) {
//					int padding = width - len;
//					for (int i = 0; i < padding && buffer_index < BUFFER_SIZE - 1; i++) {
//						buffer[buffer_index++] = space;
//					}
//				}
//				for (int i = 0; value[i] != '\0' && buffer_index < BUFFER_SIZE - 1; i++) {
//					buffer[buffer_index++] = value[i];
//				}
//			}
//			format++;
//		}
//		else {
//			buffer[buffer_index++] = *format++;
//		}
//	}
//	buffer[buffer_index] = '\0';
//	va_end(args);
//	//printf("%s", buffer);
//	USART2_Send_Array((uint8_t*)buffer,strlen(buffer));
//}
