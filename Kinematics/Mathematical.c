/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-12-09     杨金鹏       the first version
 */
#include "stdint.h"
#include "Mathematical.h"
#include "math.h"
#include "OLED.h"
#include "stdio.h"
#include "stdio.h"
#include "Serial.h"

#define pi 3.14159f // 圆周率

//获取舵机回传角度信息，混合数据转换为弧度值

//void Hexapod_GetReadDate(EntireShank *RadianRead)
//{
//  SERVO_POS_READ_2();//读取全部舵机角度
//  uint8_t i;
//  uint8_t Pos_Low,Pos_High;
//  for(i = 1;i <= 18;i++)
//  {
//      Pos_Low = 6 + (i - 1) * 3;//角度低8位数据包编号
//      Pos_High = 7 + (i - 1) * 3;//角度高8位数据包编号
//      //将回传角度数据组合为舵机值,再将舵机值转换为角度的弧度制
//      RadianRead->Radian_Read[i - 1] = (((uint16_t)Serial_RxPacket[Pos_Low] |
//                                   ((uint16_t)Serial_RxPacket[Pos_High]<< 8)) - 500) * pi / 750;
//  }
//}

//建立六足机器人几何中心的基准坐标系
void Geo_center_coordinate_Sys_Init(CoordinateAxis *csys)
{

    csys->x = 0;
    csys->y = 0;
    csys->z = 0;

}
//建立腿部髋关节的基准坐标系
void Pi0_coordinate_Sys_Init(CoordinateAxis *csys)
{
    csys->x = 0;
    csys->y = 0;
    csys->z = 0;
}

//初始化腿部关节角度值
void thetas_Init(EntireShank *theta_point)
{
    theta_point->Radian[0] = 0;
    theta_point->Radian[1] = 0;
    theta_point->Radian[2] = 0;
}

/**
  * @brief  正运动解算
  * @param  servo_theta1,servo_theta2,servo_theta3 ,i
  * @retval  none
  */
CoordinateAxis Pi3_csys[6];//定义以6条腿的髋关节为腿部基准坐标系参考下的Pi3坐标,i为腿部编号(0-5:A-F)
//Forward Kinematics 正运动学
void ForwardKinematics(float servo_theta1,float servo_theta2,float servo_theta3,uint8_t i)
{
    Pi0_coordinate_Sys_Init(&Pi3_csys[i]);//初始化坐标系
    Pi3_csys[i].x = -((L3 * cos(servo_theta2 + servo_theta3) + L2 * cos(servo_theta2) + L1) * sin(servo_theta1));
    Pi3_csys[i].y = (L3 * cos(servo_theta2 + servo_theta3) + L2 * cos(servo_theta2) + L1) * cos(servo_theta1);
    Pi3_csys[i].z = -L3 * sin(servo_theta2 + servo_theta3) - L2 * sin(servo_theta2);
}

/**
  * @brief  逆运动解算
  * @param  x轴，y轴，z轴，i
  * @retval  none
  */
EntireShank Tissue_leg[6];//定义6条腿部的对应自身的3个关节角度,i为腿部编号(0-5:A-F)
void InverseKinematics(float x,float y,float z,uint8_t i)
{
    thetas_Init(&Tissue_leg[i]);//初始化坐标系
    static float R13,R03;//R13——P1点到P3点的直线距离,R03——P0点到P3点的直线距离
    static float fai,phi,u;//逆运动几何法解算引入的辅助角

    Tissue_leg[i].Radian[0] = atan2(x,y);//髋关节角度

    R03 = sqrt(pow(x,2) + pow(y,2));
    fai = atan((- z) / (R03 - L1));
    R13 = sqrt(pow(z,2) + pow((R03 - L1),2));
    phi = acos((pow(L2,2) + pow(R13,2) - pow(L3,2)) / (2 * R13 * L2));


    Tissue_leg[i].Radian[1] = -(phi - fai);//股关节角度
    Radian_Limit(Tissue_leg[i].Radian[1],- (pi / 2),0);//设置比较合理的步幅后不需要再限制关节角度
    u = acos((pow(R13,2) + pow(L3,2) - pow(L2,2)) / (2 * R13 * L3));

    Tissue_leg[i].Radian[2] = phi + u;//基关节角度

}//(注:此函数输出的角度是弧度值,用显示屏显示检测测试数据改为了角度值(弧度值 * 180 / pi))

//Inverse kinematics 逆运动学
//void InverseKinematics(float x,float y,float z,uint8_t i)
//{
//  double angle;
//  angle=atan2(y,x);
//  Tissue_leg[i].Radian[0] = atan2(y,x);
//
//  float data= L1-sqrt(pow(x,2)+pow(y,2)) ;
//  float a = (pow(L2,2)-pow(L3,2)+pow(data,2)+pow(z,2) )/(2*L2*sqrt( pow(data,2) +pow(z,2)));
//  float b = z/data;
//  Tissue_leg[i].Radian[1] = acos(a)-atan(b);

//  float data2= L1-sqrt(pow(x,2)+pow(y,2)) ;
//  float a2 = (pow(L2,2)-pow(L3,2)+pow(data2,2)+pow(z,2) )/(2*L2*sqrt( pow(data2,2) +pow(z,2)));
//  float b2 = (pow(L3,2)-pow(L2,2)+pow(data2,2)+pow(z,2) )/(2*L3*sqrt( pow(data2,2) +pow(z,2)));
//  Tissue_leg[i].Radian[2]=-acos(a2)-acos(b2);

//}
//******************机身六自由度与腿部自由度混合********************//
//************************机身姿态控制******************************//

//R_Angle:机身坐标系相对腿部基准坐标系的旋转角度(弧度值);
//X_Tran:机身坐标系相对腿部基准坐标系的在X轴上的平移分量;
//Y_Tran:机身坐标系相对腿部基准坐标系的在Y轴上的平移分量;
//以初始站立姿态为固定坐标系参数(theta0:0;theta1:-pi/4;theta2:2*pi/3)

CoordinateAxis CPi3_csys[6];                //定义以机身几何中心为绝对坐标系参考下的Pi3,i为腿部编号(0-5:A-F)
CoordinateAxis Pi0_Pi3_csys[6];             //定义以机身几何中心为绝对坐标系参考下的Pi0Pi3向量
CoordinateAxis CPi3_balance_csys[6];        //定义机身平衡时Pi3相对几何中心的位置
CoordinateAxis Pi0_Pi3_balance_csys[6];     //定义机身平衡的向量Pi0Pi3
CoordinateAxis Pi0_Pi3_body_move_csys[6];   //定义机身移动的向量Pi0Pi3
extern float Pitch,Roll,Yaw;

void Geo_center_Position03_Init(float X_Tran,float Y_Tran,float R_Angle,uint8_t i)
{
    float alpha,beta,gamma;
//  mpu_dmp_init();
//  mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
    Geo_center_coordinate_Sys_Init(&CPi3_csys[i]);//初始化坐标系
    Geo_center_coordinate_Sys_Init(&CPi3_balance_csys[i]);//初始化坐标系
    Geo_center_coordinate_Sys_Init(&Pi0_Pi3_balance_csys[i]);//初始化坐标系
    Geo_center_coordinate_Sys_Init(&Pi0_Pi3_csys[i]);//初始化坐标系


//  CPi3_csys[i].x = X_Tran - cos(R_Angle)*(sin(0+R_Angle)*(L3*cos(-pi/4+2*pi/3)+L2*cos(-pi/4)+L1)-X_Tran) - sin(R_Angle)*(cos(0+R_Angle)*(L3*cos(-pi/4+2*pi/3)+L2*cos(-pi/4)+L1)+Y_Tran);
//  CPi3_csys[i].y = Y_Tran + cos(R_Angle)*(cos(0+R_Angle)*(L3*cos(-pi/4+2*pi/3)+L2*cos(-pi/4)+L1)+Y_Tran) - sin(R_Angle)*(sin(0+R_Angle)*(L3*cos(-pi/4+2*pi/3)+L2*cos(-pi/4)+L1)-X_Tran);
//  CPi3_csys[i].z = - 77.3670;
    CPi3_csys[i].x = X_Tran + 132.9736 * sin(R_Angle);
    CPi3_csys[i].y = Y_Tran + 132.9736 * cos(R_Angle);  //132.9736为初始站立姿态关节P0到关节P3的直线距离
    CPi3_csys[i].z = - 77.3670;                         //77.3670为初始站立姿态高度

    /*将初始状态下腿部基准坐标系的混合至机身几何中心*/
    Pi0_Pi3_csys[i].x = CPi3_csys[i].x - X_Tran;
    Pi0_Pi3_csys[i].y = CPi3_csys[i].y - Y_Tran;
    Pi0_Pi3_csys[i].z = CPi3_csys[i].z;

    /*xOy平面姿态平衡用着三个公式*/
    gamma = - Roll * pi / 180;//(水平姿态平衡x轴的横滚角要取反)
    beta = Pitch * pi / 180;//(水平姿态平衡不启用俯仰角;跟随地面平行才启用)
    alpha = 0.00f;  //alpha = Yaw * pi /180不使用偏航角(z轴零漂太严重了)，直接代入0度

    /*x轴方向平衡用着三个公式
    //  beta = 0.00f;
    //  gamma = - Roll * pi / 180;
    //  alpha = 0.00f;
    */

    /*跟随地面平行用着三个公式
    //  beta = - Pitch * pi / 180;
    //  gamma = Roll * pi / 180;
    //  alpha = 0.00f;
    */

    /*姿态解算*/
    CPi3_balance_csys[i].x = CPi3_csys[i].z*(sin(alpha)*sin(gamma) + cos(alpha)*sin(beta)*cos(gamma))
                            - (sin(alpha)*cos(gamma)-cos(alpha)*sin(beta)*sin(gamma))*CPi3_csys[i].y
                            + CPi3_csys[i].x*cos(alpha)*cos(beta);
    CPi3_balance_csys[i].y = (cos(alpha)*cos(gamma)+sin(alpha)*sin(beta)*sin(gamma))*CPi3_csys[i].y
                            - CPi3_csys[i].z*(cos(alpha)*sin(gamma)-sin(alpha)*sin(beta)*cos(gamma))
                            + CPi3_csys[i].x*sin(alpha)*cos(beta);
    CPi3_balance_csys[i].z = CPi3_csys[i].y*cos(beta)*sin(gamma) - CPi3_csys[i].x*sin(beta)
                            + CPi3_csys[i].z*cos(beta)*cos(gamma);

    /*姿态解算后的Pi0Pi3向量*/
    Pi0_Pi3_balance_csys[i].x = CPi3_balance_csys[i].x - X_Tran;
    Pi0_Pi3_balance_csys[i].y = CPi3_balance_csys[i].y - Y_Tran;
    Pi0_Pi3_balance_csys[i].z = CPi3_balance_csys[i].z;

}

/*位置解算与控制*/
void body_move_Position03_Init(float x_move,float y_move,float z_move,uint8_t i)
{
    Geo_center_coordinate_Sys_Init(&Pi0_Pi3_body_move_csys[i]);//初始化坐标系

    Pi0_Pi3_body_move_csys[i].x = Pi0_Pi3_csys[i].x - x_move;
    Pi0_Pi3_body_move_csys[i].y = Pi0_Pi3_csys[i].y - y_move;
    Pi0_Pi3_body_move_csys[i].z = Pi0_Pi3_csys[i].z - z_move;

}
//********************************************//

//限制制关节角度————主要针对股关节和基关节
float Radian_Limit(float theta,float theta_min_limit,float theta_max_limit)
{

    if(theta > theta_max_limit)
    {
        theta = theta_max_limit;
    }
    if(theta < theta_min_limit)
    {
        theta = theta_min_limit;
    }
    return theta;
}
