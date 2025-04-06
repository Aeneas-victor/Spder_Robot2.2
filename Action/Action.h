/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Action.h
  * @brief          : 动作组定义文件
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
#ifndef __ACTION_H__
#define __ACTION_H__
#include "common.h"

#define SPEED_HIGH 100
#define SPEED_MIDIUM 200
#define SPEED_LOW 300

#define FrontLeftJoint_front			-30
#define FrontLeftJoint_rear				 80
#define FrontRightJoint_front			 30
#define FrontRightJoint_rear			-80

#define MiddleLeftJoint_front			-60
#define MiddleLeftJoint_rear 			 80
#define MiddleRightJoint_front			 60
#define MiddleRightJoint_rear			-80

#define RearLeftJoint_front				-30
#define RearLeftJoint_rear  			 80
#define RearRightJoint_front			 30
#define RearRightJoint_rear				-80

#define FrontLeftLeg_UP					-60
#define FrontRightLeg_UP				 60
#define MiddleLeftLeg_UP				-60
#define MiddleRightLeg_UP				 60
#define RearLeftLeg_UP					-60
#define RearRightLeg_UP					 60

#define FrontLeftFoot_UP				 0
#define FrontRightFoot_UP				 0
#define MiddleLeftFoot_UP				 0
#define MiddleRightFoot_UP				 0
#define RearLeftFoot_UP					 0
#define RearRightFoot_UP				 0

/*********** Side Move Joint Angles ***********/
#define SideFrontLeftJoint_Left    -30  // 左前关节向左移动角度
#define SideFrontLeftJoint_Right    30  // 左前关节向右移动角度
#define SideMiddleLeftJoint_Left   -60  // 左中关节向左移动角度
#define SideMiddleLeftJoint_Right   60  // 左中关节向右移动角度
#define SideRearLeftJoint_Left     -30  // 左后关节向左移动角度
#define SideRearLeftJoint_Right     30  // 左后关节向右移动角度

#define SideFrontRightJoint_Left   -30  // 右前关节向左移动角度
#define SideFrontRightJoint_Right   30  // 右前关节向右移动角度
#define SideMiddleRightJoint_Left  -60  // 右中关节向左移动角度
#define SideMiddleRightJoint_Right  60  // 右中关节向右移动角度
#define SideRearRightJoint_Left    -30  // 右后关节向左移动角度
#define SideRearRightJoint_Right    30  // 右后关节向右移动角度

extern uint16_t SPEED;
extern const uint8_t Robot_ModuleArray[18];
extern uint16_t Robot_Low_Start[18];
extern uint16_t Robot_Posture_Begin[18];

typedef struct Direction{
	int16_t* LeftJointUPAngle;//左边joint 抬起角度数组
	int16_t* LeftJointDOWNAngle;//左边joint 放下角度数组
	int16_t* RightJointUPAngle;//右边joint 抬起角度数组
	int16_t* RightJointDOWNAngle;//右边joint放下角度数组
	
}Dire;

void Robot_Action(void);
void Move_Restoration(void);
void _Move_Robot(uint8_t* Robot_Module,uint16_t* AttitudeMode,int16_t*Angle,uint8_t number,uint16_t Time);
void Move_Advance(void);
void Move_Retreat(void);
void Move_Rotation(void);
void Move_Left_Turn(void);
void Move_Right_Turn(void);
void Move_Side_Left(void);
void Move_Side_Right(void);

void Set_Posture_High(void);
void Set_Posture_Midium(void);
void Set_Posture_Low(void);
void Set_Speed_High(void);
void Set_Speed_Midium(void);
void Set_Speed_Low(void);
void Move_Stop(void);

#endif


