/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : BlueSerial.c
  * @brief          : Bluetooch Serial file
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


#include "stm32f4xx.h"                  // Device header
#include "BlueSerial.h"
#include<stdio.h>
#include<stdarg.h>

char Bluetooch_RxPacket[BLUE_BUFFER_SIZE_MAX];
uint8_t Bluetooch_RxFlag;

void Blue_Serial_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(BLUE_USART_RCC,ENABLE);
	RCC_AHB1PeriphClockCmd(BLUE_USART_TX_GPIO_RCC,ENABLE);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin=BLUE_USART_TX_PIN|BLUE_USART_RX_PIN;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(BLUE_USART_TX_PORT,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(BLUE_USART_TX_PORT,BLUE_USART_TX_AF_PIN,BLUE_USART_AF);
	GPIO_PinAFConfig(BLUE_USART_RX_PORT,BLUE_USART_RX_AF_PIN,BLUE_USART_AF);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate=BLUE_USART_BaudRate;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	
	USART_Init(BLUE_USARTx,&USART_InitStructure);
	
	USART_Cmd(BLUE_USARTx,ENABLE);
	
	USART_ITConfig(BLUE_USARTx,USART_IT_RXNE,ENABLE);
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStrucuture;
	
	NVIC_InitStrucuture.NVIC_IRQChannel=USART3_IRQn;
	NVIC_InitStrucuture.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStrucuture.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStrucuture.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStrucuture);
}

void Blue_Send_Byte(uint8_t data)
{
	USART_SendData(BLUE_USARTx,(uint8_t)data);
	while(RESET==USART_GetFlagStatus(BLUE_USARTx,USART_FLAG_TXE)){}
}

void Blue_Send_String(uint8_t* ucstr)
{
	while(ucstr&&*ucstr)
	{
		Blue_Send_Byte(*ucstr++);
	}
}

void Blue_SendArray(uint8_t*Array,uint16_t Length)
{
	uint16_t i=0;
	for(i=0;i<Length;i++)
	{
		Blue_Send_Byte(Array[i]);
	}
}
 
void USART3_IRQHandler(void)//BLUE_USARTx串口中断服务函数
{
	static uint8_t flag = 0;
	static uint8_t pRxPacket = 0;
	if (USART_GetITStatus(BLUE_USARTx, USART_IT_RXNE) == SET)//先通过USART_GetITStatus()函数检查BLUE_USARTx接收中断标志位是否置位，如果RXNE置一了，就进if判断
	{
		uint8_t RxData = USART_ReceiveData(BLUE_USARTx);//调用USART_ReceiveData()函数读取接收寄存器中的数据到变量RxData中
		if (flag == 0)
		{
			if (RxData == 'b' )//RxData为0，表示接收到了数据帧的起始符号
			{
				flag = 1;
				pRxPacket = 0;
			}
		}
		else if (flag == 1)
		{
			if (RxData == 'e')
			{
				flag = 0;
				Bluetooch_RxPacket[pRxPacket] = '\0';//表示接收到了数据帧的终止符号，将状态切换为0，并在数据包末尾添加字符串结束符号’\0’
				Bluetooch_RxFlag = 1;//表示接收到了完整的数据包
			}
			else//否则，将接收到的数据存储到Bluetooch_RxPacket数组中
			{
				Bluetooch_RxPacket[pRxPacket] = RxData;
				pRxPacket ++;
			}
		}
		USART_ClearITPendingBit(BLUE_USARTx, USART_IT_RXNE);//手动清除BLUE_USARTx接收中断标志位，以结束中断处理过程
	}
	
}




