/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-12-09     杨金鹏       the first version
 */
#ifndef HEXAPOD_KINEMATICS_MATHEMATICAL_H_
#define HEXAPOD_KINEMATICS_MATHEMATICAL_H_
#include <stdint.h>
#define pi 3.14159f // 圆周率
#define L1 45.0f        //腿部第一连杆的长度(mm)
#define L2 75.0f        //腿部第二连杆的长度(mm)
#define L3 135.0f   //腿部第三连杆的长度(mm)

#define Chassis_Len 118.0f   //底盘长度y轴方向
#define Chassis_center_Len 182.0f //底盘中间长度y轴方向(注:中间长度和上下长度不一样)
#define Chassis_Width 240.0f //底盘宽度x轴方向


//设立坐标轴
typedef struct
{
    float x;
    float y;
    float z;

}CoordinateAxis;

//腿部关节成员
typedef struct
{
    float Radian_Write[3];//用于输出舵机控制信号(反向运动学)
    float Radian_Read[18];//用于读取舵机角度(正向运动学)

    float Radian[3];//弧度
    float Angle[3]; //角度
    uint8_t servo_low[18],servo_high[18];//高低位数据接收(测试用)

}EntireShank;

//RPY角成员
typedef struct
{

    float angle_alpha;
    float angle_beta;
    float angle_gamma;

}Turn_EntireShank;


void InverseKinematics(float x,float y,float z,uint8_t i);
void ForwardKinematics(float servo_theta1,float servo_theta2,float servo_theta3,uint8_t i);
//void Hexapod_GetReadDate(EntireShank *RadianRead);
void Geo_center_coordinate_Sys_Init(CoordinateAxis *csys);
void Pi0_coordinate_Sys_Init(CoordinateAxis *csys);
void thetas_Init(EntireShank *theta_point);
float Radian_Limit(float theta,float theta_min_limit,float theta_max_limit);
void Geo_center_Position03_Init(float X_Tran,float Y_Tran,float R_Angle,uint8_t i);
void body_move_Position03_Init(float x_move,float y_move,float z_move,uint8_t i);



#endif /* HEXAPOD_KINEMATICS_MATHEMATICAL_H_ */
