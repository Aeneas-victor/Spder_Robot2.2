/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : main
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

#include<stdint.h>
extern  uint16_t Robot_Low_Start[18];

#include "PrintfSerial.h"
#include "BlueSerial.h"
#include "Frame.h"
#include <string.h>
#include "Delay.h"
#include "Serial.h"
#include <stdio.h>
#include <stdlib.h>
#include "RobotServoController.h"
#include "Action.h"
#include "OLED.h"
#include "AsrProSerial.h"
#include "GaitController.h"
#include "aboard.h"
#include "obstacle.h"
#include "cloud.h"
#include "buzzer.h"
#define RT_VERSION 2
#define RT_SUBVERSION 2
#define RT_REVISION 0

extern char Bluetooch_RxPacket[BLUE_BUFFER_SIZE_MAX]; // 数据接收包
extern uint8_t Bluetooch_RxFlag; // 数据标志位
extern double Move_Theta0,Move_Theta1,Move_Theta2;//重新缓存输出角度
extern double Move_Position3_X,Move_Position3_Y,Move_Position3_Z;//重新缓存P3坐标
extern double Move_Theta_Foreleg[3],Move_Theta_Middleleg[3],Move_Theta_Hindleg[3];//重新缓存步态规划下的解算角度

uint8_t bule_actnumber;
float Pitch,Roll,Yaw;
enum SelectAction{
    Stop=0,Forward=1, Retreat=2,Rotation=3,Speed_High=4,
    Speed_Midium=5,Speed_Low=6,Posture_Midium=7,Posture_Low=8,Posture_High=9,
    Turn_Left=10,Turn_Right=11,Side_Left=12,Side_Right
}SelectAction;
enum CtrlMode
{
    BlueTooch=0,
    WiFi,
    Hander
}CtrlMode;

/**
  * @brief  action callback 函数指针枚举
  * @param  none
  * @retval none
  */

Action actionenum[15]=
{
    Move_Stop,Move_Advance,Move_Retreat,Move_Rotation,
    Set_Speed_High,Set_Speed_Midium,Set_Speed_Low,
    Set_Posture_Midium,Set_Posture_Low,Set_Posture_High,
    Move_Left_Turn,Move_Right_Turn,Move_Side_Left,Move_Side_Right,
    Move_Restoration
};
/**
* @brief  action callback 函数指针枚举 //机器人运动学版本
  * @param  none
  * @retval none
  */

Action kinematicsnum[15]=
{
	Move_Stop,Demo_DefaultAdvance,Demo_DefaultBack,NULL,
	NULL,NULL,NULL,
	NULL,NULL,NULL,
	Demo_DefaultTurn_Left,Demo_DefaultTurn_Right,Demo_DefaultSidestep_L,Demo_DefaultSidestep_R,
	NULL
};
//指令指针/默认指向非运动学版本的移动代码
Action* instruct=kinematicsnum;
/**
  * @brief  框架入口函数
  * @param  none
  * @retval  none
  */

int main()
{
	
//#define begin
#define _DEBUG

//#define end
    //rt_kprintf("main...\n");
//Module Init begin
	Printf_Init();
	aprintf("  //  \\\\\n");
    aprintf("  \\\\  //\n");
    aprintf("   (  )      Spider Robot\n");
    aprintf(" //    \\\\    %d.%d.%d build %s\n",RT_VERSION,RT_SUBVERSION,RT_REVISION,__DATE__);
    aprintf(" \\\\    //    VERSION : STM32F4Robot2.0-Keil Version\n");
    aprintf("  \\\\  //     Author : aeneas\n");
#ifdef _DEBUG
	aprintf("\n main...\n");
#endif
    for(uint8_t i=0;i<18;i++)
    {
        Robot_Posture_Begin[i]=Robot_Low_Start[i];
    }

    OLED_Init();
    Delay_Init();
    Robot_Serial_Init();
	Board_Init();
	Cloud_Init();
	Buzzer_Init();
    //Robot_Action();
	Gait_Init();

    //Board_Init();
    //Module Init end
	AsrPro_Init();
    Delay_Ms(2);
    OLED_ShowString(1,1,"Select Mode");
#ifdef _DEBUG
    aprintf("Select Mode\n");
#endif
    while(1)                                             
    {
        //PidTest();
        //test();
        TestMode();
        //BluetoochMode();
        //HAL_Delay(100);
    }
}

/**
  * @brief  Action
* @param  action:函数数组索引
  * @retval  none
  */
void ActionCallBack(uint8_t action)
{
    instruct[action]();
}
/**
  * @brief  蓝牙模式
  * @param  none
  * @retval  none
  */
void BluetoochMode(void)
{
#ifdef _DEBUG
     aprintf("Bluetooch Mode succeed...\n");
#endif
    Blue_Serial_Init();
	BuzzerDebug();
        while (1){
            if (Bluetooch_RxFlag == 1) {
                bule_actnumber = atoi(Bluetooch_RxPacket);
				#ifdef _DEBUG
                aprintf("exti is %d...\n", bule_actnumber);
				BuzzerDebug();
				#endif
                Bluetooch_RxFlag = 0;
            }
            ActionCallBack(bule_actnumber);
            Delay_Ms(10);
        }
}
void BalanceMode(void)
{
//  pitch=0;roll=0;yaw=0;
//  while( mpu_dmp_init() )
//  {
//      OLED_ShowString(2,1,"dmp error");
//      delay_1ms(200);
//  }
//  while(1)
//  {
//      if( mpu_dmp_get_data(&pitch,&roll,&yaw) == 0 )
//      {
//          OLED_ShowString(1,1,"pitch");
//          OLED_ShowString(2,1,"roll");
//          OLED_ShowString(3,1,"yaw");
//          OLED_ShowNum(1,6,pitch,5);
//          OLED_ShowNum(2,6,roll,5);
//          OLED_ShowNum(3,6,yaw,5);
//      }
//  }
}


#ifdef _DEBUG
void PidTest(void)
{
   // rt_kprintf("PidTest succeed...\n");
    Robot_Serial_Init();
    //USART_DMA_Init();
    OLED_Init();
    //MPU6050_Init();
    Gait_Init();
    //ActionCallBack(0);
    //uint8_t errorcode=mpu_dmp_init();
    //Delay_Ms(600);
//  Leg_BDF_Move(Move_Theta_Foreleg[0],Move_Theta_Foreleg[1],Move_Theta_Foreleg[2],
//                  Move_Theta_Middleleg[0],Move_Theta_Middleleg[1],Move_Theta_Middleleg[2],
//                  Move_Theta_Hindleg[0] ,Move_Theta_Hindleg[1],Move_Theta_Hindleg[2],Gait_Time);
    //MoveServo(3,1000,400);
    while(1)
    {
        Delay_Ms(100);
        //X_Advance_Gait();//纵向移动前进步态规划
        //X_Back_Gait();//纵向移动后退步态规划
//      Y_RTrans_Gait();//横向移动右移步态规划
//      Y_LTrans_Gait();//横向移动左移步态规划
        //Body_Balance();//机身姿态控制(有三种姿态平衡,具体可看Mathematical文件里面姿态解算部分的公式)
//      TurnLeft_Gait();//转向(左自旋)步态规划
//      TurnRight_Gait();//转向(右自旋)步态规划
        //Body_Move(float x,float y,float z);//机身位置控制
//      mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
//      OLED_ShowNum(1,7,errorcode,3);
//      OLED_ShowSignedNum(2,7,Pitch,3);
//      OLED_ShowSignedNum(3,7,Roll,3);
//      OLED_ShowSignedNum(4,7,Yaw,3);
//      Body_Balance();
//
    }
}

/**
  * @brief  测试函数
  * @param  void
  * @retval  none
  */

void TestMode(void)
{
	aprintf("test mode");
	Delay_Ms(500);
	//Demo_AdjustPosture(0,0,0,0,0,40);
	//Delay_Ms(5000);
	Demo_AdjustPosture(0,0,0,0,-40,0);
	//Delay_Ms(5000);
	//Demo_AdjustPosture(0,0,0,40,0,0);
	//Delay_Ms(5000);
	//Demo_AdjustPosture(0,0,-40,0,0,0);
	//Delay_Ms(5000);
	//Demo_AdjustPosture(0,40,0,0,0,0);
	//Delay_Ms(5000);
	//Demo_AdjustPosture(40,0,0,0,0,0);
	//Delay_Ms(5000);

    while(1)
    {
		//ActionCallBack(13);
//		Start_ADC_Conversion(ADC_Channel_10);  // 例如：ADC 通道10 (PC0)

//        // 等待一段时间，确保中断已经处理
//        for (volatile int i = 0; i < 100000; i++);

//        // 读取ADC值
//        uint16_t value = Get_ADC_Value();

//        // 打印ADC值（假设使用串口）
//        printf("ADC Value: %d \n", value);
        // 根据DOUT2状态和AOUT2值进行避障逻辑
//        if (dout2_state == 0) {
//            aprintf("Module 2: Obstacle detected! DOUT: 0, AOUT: %.2f V\n", aout2_value * 3.3 / 4095);
//        } else {
//            aprintf("Module 2: No obstacle. DOUT: 1, AOUT: %.2f V\n", aout2_value * 3.3 / 4095);
//        }
        //Delay_Ms(100);
      // Set_Servo_Angle(150);
    }
}
#endif
/**
  * @brief  步态选择函数
  * @param  none
  * @retval  none
  */  
void SwitchGait()
{

    return;
}
