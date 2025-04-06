#include "stdint.h"
#ifndef __CTRL_LEG_H__
#define __CTRL_LEG_H__

//void Initial_StandingPos(void);//基于正运动的初始复位姿态
enum LegEnumDire
{
	RearLeft,MiddleLeft,FrontLeft,FrontRight,MiddleRight,RearRight
};
/*肢体控制*/
void Single_Leg(uint8_t legid,float theta0,float theta1,float theta2,uint16_t time);

void Leg_ACE_Move(float theta0_foreleg,float theta1_foreleg,float theta2_foreleg,
					 float theta0_middleleg,float theta1_middleleg,float theta2_middleleg,
					 float theta0_hindleg ,float theta1_hindleg,float theta2_hindleg,uint16_t time);
void Leg_BDF_Move(float theta0_foreleg,float theta1_foreleg,float theta2_foreleg,
					 float theta0_middleleg,float theta1_middleleg,float theta2_middleleg,
					 float theta0_hindleg ,float theta1_hindleg,float theta2_hindleg,uint16_t time);
					 					 
#endif
