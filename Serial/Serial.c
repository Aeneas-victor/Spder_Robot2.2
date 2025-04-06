/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Serial.c
  * @brief          : Robot serial file
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
#include "Serial.h" 
#include "stdio.h"
#include "stm32f4xx.h"                  // Device header

#define pi 3.14159f // 圆周率
extern int16_t Robot_Low_Offset[18];
void Robot_Serial_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	

	RCC_AHB1PeriphClockCmd(USART_TX_GPIO_RCC,ENABLE);
	// RCC_AHB1PeriphClockCmd(BSP_USART_RX_RCC,ENABLE);

	//IO口用作串口引脚要配置复用模式
	GPIO_PinAFConfig(USART_TX_PORT,USART_TX_AF_PIN,USART_AF);
	GPIO_PinAFConfig(USART_RX_PORT,USART_RX_AF_PIN,USART_AF);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin           = USART_TX_PIN;	//TX引脚
	GPIO_InitStructure.GPIO_Mode          = GPIO_Mode_AF;		//IO口用作串口引脚要配置复用模式
	GPIO_InitStructure.GPIO_Speed         = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType         = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd          = GPIO_PuPd_UP;
	GPIO_Init(USART_PORT,&GPIO_InitStructure);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin           = USART_RX_PIN;	//RX引脚
	GPIO_InitStructure.GPIO_Mode          = GPIO_Mode_AF;		//IO口用作串口引脚要配置复用模式
	GPIO_InitStructure.GPIO_Speed         = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType         = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd          = GPIO_PuPd_UP;
	GPIO_Init(USART_PORT,&GPIO_InitStructure);

	USART_InitTypeDef USART_InitStructure;//定义配置串口的结构体变量

	RCC_APB2PeriphClockCmd(USART_RCC, ENABLE);//开启串口1的时钟

	USART_DeInit(USARTx);//大概意思是解除此串口的其他配置

	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate              = RobotUSART_BaudRate;//设置波特率
	USART_InitStructure.USART_WordLength            = USART_WordLength_8b;//字节长度为8bit
	USART_InitStructure.USART_StopBits              = USART_StopBits_1;//1个停止位
	USART_InitStructure.USART_Parity                = USART_Parity_No ;//没有校验位
	USART_InitStructure.USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx;//将串口配置为收发模式
	USART_InitStructure.USART_HardwareFlowControl   = USART_HardwareFlowControl_None; //不提供流控 
	USART_Init(USARTx,&USART_InitStructure);//将相关参数初始化给串口1
	
	USART_ClearFlag(USARTx,USART_FLAG_RXNE);//初始配置时清除接受置位
	
	USART_Cmd(USARTx,ENABLE);//开启串口1

	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);//初始配置接受中断

	
	NVIC_InitTypeDef NVIC_InitStructure;//中断控制结构体变量定义

	NVIC_InitStructure.NVIC_IRQChannel                    = USART1_IRQn;//中断通道指定为USART1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 0;//主优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority         = 1;//次优先级为1
	NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;//确定使能
	NVIC_Init(&NVIC_InitStructure);//初始化配置此中断通道
		
}


void Robot_Send_Byte(uint8_t data)
{
    USART_SendData(USARTx, (uint8_t)data);
	
	// 等待发送数据缓冲区标志置位
    while( RESET == USART_GetFlagStatus(USARTx, USART_FLAG_TXE) ){} 
}


void Robot_Send_String(uint8_t *ucstr)
{   
      while(ucstr && *ucstr)  // 地址为空或者值为空跳出   
      {     
			Robot_Send_Byte(*ucstr++);    
      }
}


void Robot_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		//遍历数组
	{
		Robot_Send_Byte(Array[i]);		//依次调用Robot_Send_Byte发送每个字节数据
	}
}


#if !defined(__MICROLIB)
//不使用微库的话就需要添加下面的函数
#if (__ARMCLIB_VERSION <= 6000000)
//如果编译器是AC5  就定义下面这个结构体
struct __FILE
{
	int handle;
};
#endif

FILE __stdout;

//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
	x = x;
}
#endif



uint16_t Servo_SetAngle(float angle_rad,uint8_t i)	
{
	uint16_t Servo_Value = (uint16_t)(angle_rad * 750 / pi + 500);	//弧度值转换为多值
	Servo_Value+=Robot_Low_Offset[i];
	//uint16_t Servo_Value = (uint16_t)(angle_rad * 1000/ pi);	//弧度值转换为多值
	return Servo_Value;
}
uint16_t Servo_uint16Angle(float angle_rad)
{
	uint16_t Servo_Value=(uint16_t)angle_rad;
	return Servo_Value;
}

//设置舵机id
uint8_t Get_ID(uint8_t id)
{
	id = id;
	return id;
}

//设置舵机到达指定位置时间
uint16_t Servo_SetTime(uint16_t move_time)
{
	move_time = move_time;
	return move_time;
}	

//控制舵机个数
uint8_t Servo_Control_Num(uint8_t Control_Num)
{
	uint8_t i = Control_Num;
	return i;
}

//发送的数据长度
uint8_t Servo_DataLength(uint8_t Control_Num)
{
	uint8_t N = Control_Num * 3 + 5;
	return N;
}
