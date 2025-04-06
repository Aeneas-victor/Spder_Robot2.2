#include "cloud.h"
#include "PrintfSerial.h"
// PWM 参数
#define PWM_FREQUENCY 50       // 50Hz (周期 20ms)
#define TIMER_CLOCK  84000000  // APB1 定时器时钟 84MHz
#define PWM_PERIOD   (TIMER_CLOCK / PWM_FREQUENCY / 2000) // 计算周期（20ms）

// 舵机角度范围 (0.5ms - 2.5ms)
#define SERVO_MIN (PWM_PERIOD * 0.5 / 20)   // 0.5ms -> 最小角度
#define SERVO_MAX (PWM_PERIOD * 2.5 / 20)   // 2.5ms -> 最大角度
void Cloud_Init()
{    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    // 使能 TIM3 & GPIOA 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // 配置 GPIOA6 (TIM3_CH1) 和 GPIOA7 (TIM3_CH2) 为复用功能
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 复用映射
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3); // PA6 -> TIM3_CH1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3); // PA7 -> TIM3_CH2

    // TIM3 配置
    TIM_TimeBaseStruct.TIM_Period = PWM_PERIOD - 1;
    TIM_TimeBaseStruct.TIM_Prescaler = 0;  
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);

    // 配置 PWM 输出模式
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = SERVO_MIN; // 默认最小角度 (0.5ms)
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM3, &TIM_OCInitStruct); // TIM3 CH1 (PA6)
    TIM_OC2Init(TIM3, &TIM_OCInitStruct); // TIM3 CH2 (PA7)

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // 启动 TIM3
    TIM_Cmd(TIM3, ENABLE);
	for (volatile int i = 0; i < 100000; i++);

	// 读取 TIM3 计数器的值
	#ifdef _DEBUG
	uint16_t counter = TIM_GetCounter(TIM3);
	if (counter == 0) {
		aprintf("TIM3 可能未运行 (Counter = 0)\n");
	} else {
		aprintf("TIM3 正在运行 (Counter = %d)\n", counter);
	}
	aprintf("cloud init ...\n");
	#endif
}

void Servo_Pitch(uint16_t angle) //俯仰角
{
	uint16_t pulse_width=(uint16_t)(SERVO_MIN + ((SERVO_MAX - SERVO_MIN) * (angle / 180.0)));
	TIM_SetCompare1(TIM3, pulse_width);
	#ifdef _DEBUG
	aprintf("Pitch  %d...\n",pulse_width);
	#endif
}

void Servo_Yaw(uint16_t angle)   //偏航角
{
	uint16_t pulse_width=(uint16_t)(SERVO_MIN + ((SERVO_MAX - SERVO_MIN) * (angle / 180.0)));
	TIM_SetCompare2(TIM3,pulse_width);
	#ifdef _DEBUG
	aprintf("Yaw  %d...\n",pulse_width);
	#endif
}



