#include "AsrProSerial.h"
#include "stm32f4xx.h"                  // Device header
#include "OLED.h"
#include "buzzer.h"
char Asr_RxPacket[ASR_BUFFER_SIZE_MAX];
uint8_t Asr_Rx;
extern uint8_t bule_actnumber;
void AsrPro_Init(void)
{
	RCC_AHB1PeriphClockCmd(ASR_RCCGPIO,ENABLE);
	RCC_APB2PeriphClockCmd(ASR_RCCUSART,ENABLE);
	
    GPIO_InitTypeDef asrprogpiostruct;
	asrprogpiostruct.GPIO_Mode=GPIO_Mode_AF;
	asrprogpiostruct.GPIO_OType=GPIO_OType_PP;
	asrprogpiostruct.GPIO_Pin=ASR_GPIO_TX|ASR_GPIO_RX;
	asrprogpiostruct.GPIO_PuPd=GPIO_PuPd_UP;
	asrprogpiostruct.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(ASR_GPIO_PORT,&asrprogpiostruct);

	GPIO_PinAFConfig(ASR_GPIO_PORT,ASR_AF_TX_SOURCE,ASR_USART_AF);
	GPIO_PinAFConfig(ASR_GPIO_PORT,ASR_AF_RX_SOURCE,ASR_USART_AF);

	USART_InitTypeDef asrproinitstruct;
	asrproinitstruct.USART_BaudRate=ASR_BAUDRATE;
	asrproinitstruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	asrproinitstruct.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	asrproinitstruct.USART_Parity=USART_Parity_No;
	asrproinitstruct.USART_StopBits=USART_StopBits_1;
	asrproinitstruct.USART_WordLength=USART_WordLength_8b;
	USART_Init(ASR_USARTX,&asrproinitstruct);
	USART_Cmd(ASR_USARTX,ENABLE);
	
	USART_ITConfig(ASR_USARTX,USART_IT_RXNE,ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_Initstruct;
	NVIC_Initstruct.NVIC_IRQChannel=USART6_IRQn;
	NVIC_Initstruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Initstruct.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_Initstruct.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_Initstruct);
	#ifdef _DEBUG
	aprintf("AsrPro Init...\n");
	#endif
}

void Asr_Send_Byte(uint8_t data)
{
	USART_SendData(ASR_USARTX,(uint8_t)data);
	while(RESET==USART_GetFlagStatus(ASR_USARTX,USART_FLAG_TXE)){}
}

void Asr_Send_String(uint8_t*ucstr)
{
	while(ucstr&&*ucstr)
	{
		Asr_Send_Byte(*ucstr++);
	}
}

void Asr_Send_Array(uint8_t*Array,uint16_t Length)
{
	uint16_t i=0;
	for(i=0;i<Length;i++)
	{
		Asr_Send_Byte(Array[i]);
	}
}

void USART6_IRQHandler(void)
{
	uint8_t ucTemp;
	OLED_ShowString(1,1,"USART6FUnc");
	if(USART_GetFlagStatus(ASR_USARTX,USART_IT_RXNE)!=RESET)
	{
		ucTemp=USART_ReceiveData(ASR_USARTX);
		OLED_ShowString(2,1,"IRQ");

	}
	Asr_Rx=ucTemp;
	bule_actnumber=Asr_Rx;
	//USART_ClearITPendingBit(ASR_USARTX,USART_IT_RXNE);
	BuzzerDebug();
		
		
}
