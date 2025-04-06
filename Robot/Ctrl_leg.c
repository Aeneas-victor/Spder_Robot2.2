
#include "Serial.h"
#include "Mathematical.h"
#include "GaitController.h"
#include "stm32f4xx.h"                  // Device header
#include "Ctrl_leg.h"
#include "RobotServoController.h"
//注：从机械物理结构上1,4,7号舵机绕机身坐标系z轴顺时针方向(角度输入要取负)
// 2,5,8,11,14,17中间舵机绕本地坐标系z轴顺时针方向(用逆运动学解算出的角度输入到驱动层要取负)

/*
//		  12			3
//			\		   /	
//		  11 \D		A /2
//		   10 \		 /1
//		       ------
//			  |      |
//	15  14  13|		 |4   5   6
//	----------|		 |---------
//		E	  |		 |     B
//			  |	0  0 |
//			   ------
//			16/		 \7
//		  17 /F		C \8
//			/		   \
//		  18			9
*/


/*
//		  12			3
//			\		   /	
//rearright  \11	  /2 rearleft
//		   10 \		 /1
//		       ------
//			  |      |
//	15  14  13|		 |4   5   6
//	----------|		 |---------
//middleright |		 | middleleft
//			  |	0  0 |
//			   ------
//			  /16	 \7
//frontright /17	  \8  frontleft
//			/		   \
//		  18			9
*/


#define pi 3.14159f // 圆周率

/*
//初始站立姿态或复位姿态，以舵机中轴线为0度
//中心和中间舵机的角度范围为(-120,120)
//外围舵机的角度范围为(-30,210)(注：在安装上给外围舵机加了90度的正偏)
*/
/****************************************************
 * @brief 控制单个腿的函数
 *
 *
 *
 * @param[num] 区别腿的编号
 * @param[theta0]髋关节角度
 * @param[theta1]大腿关节角度
 * @param[theta2]小腿关节角度
 * @param[time]运动时间
 *
 * @return void
 ****************************************************/
void Single_Leg(uint8_t legid,float theta0,float theta1,float theta2,uint16_t time)
{
	if(legid<3){
		theta2=theta2-pi;
	}else{
		theta2=theta2-pi/3;
		theta1=-theta1;
	}
	uint8_t _id1=legid*3+1;
	uint8_t _id2=_id1+1;
	uint8_t _id3=_id2+1;
	Robot_Send_Byte(0x55);		//帧头
	Robot_Send_Byte(0x55);		//帧头
	Robot_Send_Byte(Servo_DataLength(3));		//数据长度：控制舵机树*3+5
	Robot_Send_Byte(0x03);		//舵机移动指令

	Robot_Send_Byte(Servo_Control_Num(3));		//Prm1:要控制舵机个数——18
	Robot_Send_Byte(Servo_SetTime(time));		//Prm2:时间低八位
	Robot_Send_Byte(Servo_SetTime(time) >> 8);  //Prm3:时间高八位
	
	Robot_Send_Byte(Get_ID(_id1));
	Robot_Send_Byte(Servo_SetAngle(theta0,_id1-1));
	Robot_Send_Byte(Servo_SetAngle(theta0,_id1-1) >> 8);	
	
	Robot_Send_Byte(Get_ID(_id2));
	Robot_Send_Byte(Servo_SetAngle(theta1,_id2-1));
	Robot_Send_Byte(Servo_SetAngle(theta1,_id2-1) >> 8);
	
	Robot_Send_Byte(Get_ID(_id3));
	Robot_Send_Byte(Servo_SetAngle(theta2,_id3-1));
	Robot_Send_Byte(Servo_SetAngle(theta2,_id3-1)>>8);
}


/*控制ACE组*/
void Leg_ACE_Move(float theta0_foreleg,float theta1_foreleg,float theta2_foreleg,
					 float theta0_middleleg,float theta1_middleleg,float theta2_middleleg,
					 float theta0_hindleg ,float theta1_hindleg,float theta2_hindleg,uint16_t time)
{
	Single_Leg(4,theta0_middleleg,theta1_middleleg,theta2_middleleg,time);
	Single_Leg(0,- theta0_hindleg,theta1_hindleg,theta2_hindleg,time);
	Single_Leg(2,- theta0_foreleg,theta1_foreleg,theta2_foreleg,time);
}

/*控制BDF组*/
void Leg_BDF_Move(float theta0_foreleg,float theta1_foreleg,float theta2_foreleg,
					 float theta0_middleleg,float theta1_middleleg,float theta2_middleleg,
					 float theta0_hindleg ,float theta1_hindleg,float theta2_hindleg,uint16_t time)
{
	Single_Leg(1,-theta0_middleleg,theta1_middleleg,theta2_middleleg,time);
	Single_Leg(3,theta0_hindleg,theta1_hindleg,theta2_hindleg,time);
	Single_Leg(5,theta0_foreleg,theta1_foreleg,theta2_foreleg,time);	
}

