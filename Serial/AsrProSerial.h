#ifndef __ASRPROSERIAL_H
#define __ASRPROSERIAL_H
#include "stm32f4xx.h"                  // Device header

#define ASR_BAUDRATE          115200
#define ASR_RCCGPIO           RCC_AHB1Periph_GPIOC
#define ASR_RCCUSART          RCC_APB2Periph_USART6

#define ASR_GPIO_TX           GPIO_Pin_6
#define ASR_GPIO_RX           GPIO_Pin_7
#define ASR_GPIO_PORT         GPIOC

#define ASR_USARTX            USART6
#define ASR_USART_AF          GPIO_AF_USART6
#define ASR_AF_TX_SOURCE      GPIO_PinSource6
#define ASR_AF_RX_SOURCE      GPIO_PinSource7
#define ASR_BUFFER_SIZE_MAX   100
void AsrPro_Init(void);

#endif

