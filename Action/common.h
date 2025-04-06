/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : common.h
  * @brief          : common
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
#ifndef __COMMON_H__
#define __COMMON_H__
#include<stdint.h>

typedef void(*Callback)(float);

enum Robot_Module{
	rear_Left_joint=1,rear_Left_leg,rear_Left_foot,
	middle_Left_joint,middle_Left_leg,middle_Left_foot,
	front_Left_joint,front_Left_leg,front_Left_foot,
	
	rear_Right_joint,rear_Right_leg,rear_Right_foot,
	middle_Right_joint,middle_Right_leg,middle_Right_foot,
	front_Right_joint,front_Right_leg,front_Right_foot
	
};


#endif
