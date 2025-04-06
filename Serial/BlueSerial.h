/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : BlueSerial.h
  * @brief          : Bluetooch Serial 
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
#ifndef __BLUESERIAL_H__
#define __BLUESERIAL_H__
#include <stdint.h>
#define BLUE_USART_RCC                RCC_APB1Periph_USART3
#define BLUE_USART_TX_GPIO_RCC        RCC_AHB1Periph_GPIOB
#define BLUE_USART_RX_GPIO_RCC		  RCC_AHB1Periph_GPIOB

#define BLUE_USARTx                   USART3
#define BLUE_USART_TX_PORT            GPIOB
#define BLUE_USART_TX_PIN             GPIO_Pin_10
#define BLUE_USART_RX_PORT            GPIOB
#define BLUE_USART_RX_PIN             GPIO_Pin_11
#define BLUE_USART_BaudRate           9600
#define BLUE_USART_AF                 GPIO_AF_USART3
#define BLUE_USART_TX_AF_PIN          GPIO_PinSource10
#define BLUE_USART_RX_AF_PIN          GPIO_PinSource11

#define BLUE_BUFFER_SIZE_MAX          100

void Blue_Serial_Init(void);
void Blue_Send_Byte(uint8_t data);
void Blue_Send_String(uint8_t* ucstr);
void Blue_SendArray(uint8_t* Array,uint16_t Length);

#endif
