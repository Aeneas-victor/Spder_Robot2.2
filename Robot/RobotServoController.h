#ifndef _ROBOTSERVOCONTROLLER_H__
#define _ROBOTSERVOCONTROLLER_H__
#include "stm32f4xx.h"                  // Device header
#include "Serial.h"
#ifdef __cplusplus
extern "C"{
	#endif
typedef struct _Robot_Servo
{
	uint8_t ID;
	uint16_t Position;
}RobotServo;

/********************************************************************/
//发送给控制板
#define FRAME_HEADER           		0x55 //帧头
#define CMD_SERVO_MOVE       		0x03 //舵机移动指令
#define CMD_ACTION_GROUP_RUN		0x06 //运行动作组指令
#define CMD_ACTION_GROUP_STOP       0x07 //停止正在运行的动作组
#define CMD_ACTION_GROUP_SPEED      0x0B //控制动作组的速度
#define CMD_GET_BATTERY_VOLTAGE     0x0F //获取控制板电压
#define CMD_MULT_SERVO_UNLOAD       0x14 //控制多个舵机马达掉电
#define CMD_MULT_SERVO_POS_READ     0x15 //读取多个舵机角度位置值

/********************************************************************/

//控制板发回


/********************************************************************/
void MoveServo(uint8_t servoID, int16_t Angle, uint16_t Time);
void MoveServosByArray(RobotServo servos[], uint8_t Num, uint16_t Time);
void moveServo(uint8_t servoID, int16_t Angle, uint16_t Time);
void runActionGroup(uint8_t numOfAction, uint16_t Times);
#ifdef __cplusplus
}
#endif 

#endif


