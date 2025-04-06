#include "stdio.h"
#include "stdint.h"
#ifndef __GAITCONTROLLER_H__
#define __GAITCONTROLLER_H__
#include "Mathematical.h"
#define Hip_edge 22.8694f        //新足端执行器抬升高度
#define Hip_Redge 34.3041f       //足端执行器抬升高度
//#define SL_half_edge 34.4161f//中心舵机1,7,10,16在x轴方向的步长(新步长)———髋关节转动15度确立的步长
#define SL_half_edge 45.7388//中心舵机1,7,10,16在x轴方向的步长(新步长)———髋关节转动20度确立的步长
//#define SL_half_edge 50.f
#define SL_half 66.4868f//中心舵机1,7,10,16在x轴方向的步长(老步长)———髋关节转动30度确立的步长
#define Gait_Time 200	        //每次腿部动作执行时间
#define Gait_RTime 200
#define Gait_TTime 200

#include <string.h>  // 添加在文件头部，用于memcpy

#define DEG2RAD (pi/180.0f)
#define clamp(v, min, max) ((v) < (min) ? (min) : ((v) > (max) ? (max) : (v)))
#define CHASSIS_CENTER_X 0.0f
#define CHASSIS_CENTER_Y 0.0f
#define CHASSIS_CENTER_Z 0.0f

#define ANGLE2 40.0f/180.0f*pi
#define ANGLE3 120.0f/180.0f*pi
/*  
 * 旋转参数（单位：弧度）  
 *   
 * 参数  | 物理意义       | 坐标系方向                  | 典型应用场景                 
 * ------|----------------|-----------------------------|-----------------------------   
 * RX    | 横滚角 (Roll)  | 绕X轴旋转（机身左右倾斜）  | 调整机身侧倾、保持平衡      
 * RY    | 俯仰角 (Pitch) | 绕Y轴旋转（机身前后俯仰）  | 上下坡姿态、抬头/低头      
 * RZ    | 偏航角 (Yaw)   | 绕Z轴旋转（机身水平转向）  | 转向控制、修正航向角       
 */  

/*  
 * 平移参数（单位：毫米）  
 *   
 * 参数  | 物理意义       | 坐标系方向                  | 典型应用场景                 
 * ------|----------------|-----------------------------|-----------------------------   
 * TX    | X轴平移       | 前后移动（前进/后退）      | 直线行走、位置微调         
 * TY    | Y轴平移       | 左右移动（横移）           | 侧移步态、避障             
 * TZ    | Z轴平移       | 垂直升降（升高/降低）      | 调整机身高度、跨越障碍     
 */  
typedef struct {
    float RX;  // 旋转偏量
    float RY;
    float RZ;
    float TX;  // 平移偏量
    float TY;
    float TZ;
} MotionParams;

enum LegGroup { ACE_GROUP, BDF_GROUP };
enum GaitType { 
    FORWARD,    // 前进
    BACKWARD,   // 后退 
    SIDESTEP_L, // 左侧移
    SIDESTEP_R, // 右侧移
    TURN_LEFT,  // 左转
    TURN_RIGHT  // 右转
};


/* 方法1：使用宏定义 */
#define deg2rad(deg) ((deg) * pi / 180.0f)
void Gait_Init(void);
/* 姿态调整核心函数 */
void AdjustPosture(const MotionParams* params);
/* 改进的调试接口（带安全限制）*/
void Demo_AdjustPosture(float tx, float ty, float tz, 
                       float rx_deg, float ry_deg, float rz_deg);


CoordinateAxis GetBasePosition(const MotionParams* params);
void CalculateAndCacheAngles(CoordinateAxis base, float dx, float dy, float dz,uint8_t leg_id, float* output_array);
void MoveLegsGroup(float* foreleg, float* middleleg, float* hindleg,enum LegGroup group, int duration);
void ExecuteGaitPhase(enum LegGroup group, enum GaitType type,const MotionParams* params, uint8_t phase);
MotionParams DefaultMotionParams(void);
void X_Advance_Gait(const MotionParams* params);
void X_Backward_Gait(const MotionParams* params);
void X_Sidestep_L_Gait(const MotionParams* params);
void X_Sidestep_R_Gait(const MotionParams* params);
void X_Turn_Left_Gait(const MotionParams* params);
void X_Turn_Right_Gait(const MotionParams* params);

void Demo_DefaultAdvance(void);
void Demo_DefaultBack(void);
void Demo_DefaultTurn_Left(void);
void Demo_DefaultTurn_Right(void);
void Demo_DefaultSidestep_L(void);
void Demo_DefaultSidestep_R(void);

#endif


