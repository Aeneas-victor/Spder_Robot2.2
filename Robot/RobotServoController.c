#include "stm32f4xx.h"                  // Device header
#include "RobotServoController.h"
#include "Serial.h"
#include <stdarg.h>
#include <string.h>

#define GET_LOW_BYTE(A) ((uint8_t)(A))
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))

uint8_t RobotTxBuf[128];  //发送缓存

/**
* @brief  控制单个舵机运动
* @param  servoID:舵机ID
* @param  Angle：转动角度
* @param  Time: 转动时间
  * @retval  无
  */
void MoveServo(uint8_t servoID, int16_t Angle, uint16_t Time)
{
	if (servoID > 18 || !(Time > 0)) {  //舵机ID不能大于15时间不能小于0,可根据对应控制板修改
		return;
	}
	uint16_t Position;
	//Position = 500 + 2000*Angle/270;          //实现角度和位置的转换，这里的270°可以改为180°/360°
	//Position = 500 + 2000*Angle/1000; 
	Position=Angle;
	RobotTxBuf[0] = RobotTxBuf[1] = FRAME_HEADER;    //填充帧头0x55
	RobotTxBuf[2] = 8;                        //数据长度=要控制舵机数*3+5，此处=1*3+5
	RobotTxBuf[3] = CMD_SERVO_MOVE;           //填充舵机移动指令0x03
	RobotTxBuf[4] = 1;                        //要控制的舵机个数
	RobotTxBuf[5] = GET_LOW_BYTE(Time);       //取得时间的低八位
	RobotTxBuf[6] = GET_HIGH_BYTE(Time);      //取得时间的高八位
	RobotTxBuf[7] = servoID;                  //舵机ID
	RobotTxBuf[8] = GET_LOW_BYTE(Position);   //取得目标位置的低八位
	RobotTxBuf[9] = GET_HIGH_BYTE(Position);  //取得目标位置的高八位

	Robot_SendArray(RobotTxBuf, 10);
}

/**
* @brief  控制多个舵机运动
* @param  serves[]:舵机结构体数组
* @param  Num:舵机个数
* @param  Time: 转动时间
  * @retval  无
  */
void MoveServosByArray(RobotServo servos[], uint8_t Num, uint16_t Time)
{
	uint8_t index = 7;
	uint8_t i = 0;

	if (Num < 1 || Num > 32 || !(Time > 0)) {
		return;                                          //舵机数不能为零和大与32，时间不能为零
	}
	RobotTxBuf[0] = RobotTxBuf[1] = FRAME_HEADER;      //填充帧头
	RobotTxBuf[2] = Num * 3 + 5;                       //数据长度 = 要控制舵机数*3+5
	RobotTxBuf[3] = CMD_SERVO_MOVE;                    //填充舵机移动指令
	RobotTxBuf[4] = Num;                               //要控制的舵机个数
	RobotTxBuf[5] = GET_LOW_BYTE(Time);                //取得时间的低八位
	RobotTxBuf[6] = GET_HIGH_BYTE(Time);               //取得时间的高八位

	for (i = 0; i < Num; i++) {                        //循环填充舵机ID和对应目标位置
		RobotTxBuf[index++] = servos[i].ID;              //填充舵机ID
		RobotTxBuf[index++] = GET_LOW_BYTE(servos[i].Position); //填充目标位置低八位
		RobotTxBuf[index++] = GET_HIGH_BYTE(servos[i].Position);//填充目标位置高八位
	}

	Robot_SendArray(RobotTxBuf, RobotTxBuf[2] + 2);             //发送
}
/**
* @brief  控制舵机运行指定动作组
* @param  numOfAction:动作组序号
* @param  Time: 转动时间
  * @retval  无
  */
void runActionGroup(uint8_t numOfAction, uint16_t Times)
{
	RobotTxBuf[0] = RobotTxBuf[1] = FRAME_HEADER;  //填充帧头
	RobotTxBuf[2] = 5;                      //数据长度，此命令固定为5
	RobotTxBuf[3] = CMD_ACTION_GROUP_RUN;   //填充运行动作组命令
	RobotTxBuf[4] = numOfAction;            //填充要运行的动作组号
	RobotTxBuf[5] = GET_LOW_BYTE(Times);    //取得要运行次数的低八位
	RobotTxBuf[6] = GET_HIGH_BYTE(Times);   //取得要运行次数的高八位
	
	Robot_SendArray(RobotTxBuf, 7);           
}
