#include "Mathematical.h"
#include "RobotServoController.h"
#include "Ctrl_leg.h"
#include "GaitController.h"
#include "OLED.h"
#include "stm32f4xx.h"                  //Device header
#include "Delay.h"
#include "Serial.h"
#include "math.h"
#include "PrintfSerial.h"
int16_t Robot_Low_Offset[19]=
{
	 22,  9,-44,
	 25, 39,  5,
	-13, 47,-59,
	  0,-24, 55,
	 -1, 19, 58,
	-17,-25, 43,0
};

//CoordinateAxis   坐标轴
//EntireShank      整体腿
// 确保这些变量已实际存在（在某个.c文件中定义）

extern CoordinateAxis Pi3_csys[6];//定义6条腿Pi3的坐标系,i为腿部编号(0-fd+5:A-F)
extern EntireShank Tissue_leg[6];//定义6条腿部的对应自身的3个关节角度,i为腿部编号(0-5:A-F)
extern CoordinateAxis Pi0_Pi3_balance_csys[6];//定义机身平衡的向量Pi0Pi3
extern CoordinateAxis CPi3_csys[6];//定义以机身几何中心为绝对坐标系参考下的Pi3,i为腿部编号(0-5:A-F)
extern CoordinateAxis Pi0_Pi3_csys[6];             //定义以机身几何中心为绝对坐标系参考下的Pi0Pi3向量
extern CoordinateAxis CPi3_balance_csys[6];        //定义机身平衡时Pi3相对几何中心的位置
extern CoordinateAxis Pi0_Pi3_body_move_csys[6];   //定义机身移动的向量Pi0Pi3

double Move_Theta0,Move_Theta1,Move_Theta2;//重新缓存输出角度
double Move_Position3_X,Move_Position3_Y,Move_Position3_Z;//重新缓存P3坐标
float Move_Theta_Foreleg[3],Move_Theta_Middleleg[3],Move_Theta_Hindleg[3];//重新缓存步态规划下的解算角度

/**
  * @brief  Gait-Init   运动学初始化函数
  * @param  void
  * @retval  void
  */
//           |- Len -|
//          P2---*---P1--------
//          /    |    \     |
//         /     |     \   Width/2
//        /      |      \   |
//       P3-------------P0 ---
//       |\      |      /|
//       | \     |     / |
//       |  \    |    /  |
//       |   P4--*--P5   |
//       |       |       |
//       |--center_Len---|


//       X axis
//        ^
//        |
//        |
//        ----> Y axis

void Gait_Init(void) // 初始站立状态的位姿和坐标初始化
{
    // 预定义参数
    float angles[6] = { -(pi / 4), -(pi / 4), -(pi / 4), -(pi / 4), -(pi / 4), -(pi / 4) };
    float lengths[6] = { 2 * pi / 3, 2 * pi / 3, 2 * pi / 3, 2 * pi / 3, 2 * pi / 3, 2 * pi / 3 };
    // 进行正向运动学计算
    for (int i = 0; i < 6; i++) {
        ForwardKinematics(0, angles[i], lengths[i], i);
    }
    // 进行逆向运动学计算
    for (int i = 0; i < 6; i++) {
        InverseKinematics(Pi3_csys[i].x, Pi3_csys[i].y, Pi3_csys[i].z, i);
    }
    // 锁定机身几何中心坐标系
    float geo_positions[6][3] = {
        { -Chassis_Width / 2, -Chassis_Len / 2, -3 * pi / 4 },
        { 0, -Chassis_center_Len / 2, pi },
        { Chassis_Width / 2, -Chassis_Len / 2, 3 * pi / 4 },
        { -Chassis_Width / 2, Chassis_Len / 2, -pi / 4 },
        { 0, Chassis_center_Len / 2, 0 },
        { Chassis_Width / 2, Chassis_Len / 2, pi / 4 }
    };
    for (int i = 0; i < 6; i++) {
        Geo_center_Position03_Init(geo_positions[i][0], geo_positions[i][1], geo_positions[i][2], i);
    }
    // 站立姿态角度信息传入舵机控制板进行驱动
    for (int i = 0; i < 6; i++) {
        Single_Leg(i, Tissue_leg[i].Radian[0], Tissue_leg[i].Radian[1], Tissue_leg[i].Radian[2], 0);
    }
}
#define _DEBUG
/**
  * @brief  AdjustPosture 函数
  * @param  MotionParams 结构体,其值可以为@MotionParams
  * @retval  none
  */
void AdjustPosture(const MotionParams* params)
{
    MotionParams default_params = {0};
    if(!params) params = &default_params;

    CoordinateAxis base = GetBasePosition(params);
    
    const uint8_t leg_groups[2][3] = {
        {2, 4, 0},  // ACE组
        {5, 1, 3}   // BDF组
    };

    for(int group_idx = 0; group_idx < 2; group_idx++) 
    {
        float group_angles[3][3] = {0};
        
        for(int leg_idx = 0; leg_idx < 3; leg_idx++)
        {
            const uint8_t leg_id = leg_groups[group_idx][leg_idx];
            
            //======= 坐标变换修正开始 =======//
            /* 原始安装位置 */
            float leg_x = Pi3_csys[leg_id].x;
            float leg_y = Pi3_csys[leg_id].y;
            float leg_z = Pi3_csys[leg_id].z;

            /* 旋转顺序：Z->Y->X */
            // 绕Z轴旋转（Yaw）
			float actual_rz = params->RZ;
            // 左侧腿组需要反转旋转方向
            if(leg_id>2) {
                actual_rz = -actual_rz;  // 反转旋转角度
            }

            /* 绕Z轴旋转（带方向修正） */
            float x_z = leg_x * cosf(actual_rz) - leg_y * sinf(actual_rz);
            float y_z = leg_x * sinf(actual_rz) + leg_y * cosf(actual_rz);
            float z_z = leg_z;
            
            // 绕Y轴旋转（Pitch）
            const float ry_gain = 1.0f + fabsf(x_z)/100.0f; // [修改] 添加动态增益
			float y_y = y_z; 
            float x_y = x_z * cosf(params->RY) + z_z * sinf(params->RY) * ry_gain; // [修改]
            float z_y = -x_z * sinf(params->RY) * ry_gain + z_z * cosf(params->RY); // [修改]
            
            // 绕X轴旋转（Roll）
            float x_x = x_y;
            float y_x = y_y * cosf(params->RX) + z_y * sinf(params->RX);  // [修改] 符号修正
            float z_x = -y_y * sinf(params->RX) + z_y * cosf(params->RX); // [修改] 符号修正

            /* 应用平移变换 */
            float final_x = x_x + params->TX;
            float final_y = y_x + params->TY;
            float final_z = z_x + params->TZ;

            /* 计算相对原始位置的偏移量 */
            float dx = final_x - leg_x;
            float dy = final_y - leg_y;
            float dz = final_z - leg_z;

            /* 组别补偿（保留原有逻辑） */
            if(leg_id >= 3) {  // 左侧腿补偿（3-5）
                dx *= 0.9f;
                dy *= 0.7f;
				dz *= 1.1f;  // [新增] Z补偿
            }

// [修改] 添加RY专项补偿 ------------------------------------
            if(group_idx == 0){ // ACE组（右半区）
                if(leg_idx == 0){ // 前腿(2)
                    dz -= 0.3f * params->RY * fabsf(params->RY); // [新增]
                }else if(leg_idx == 2){ // 后腿(0)
                    dz += 0.4f * params->RY * fabsf(params->RY); // [新增]
                }
            }else{ // BDF组（左半区）
                if(leg_idx == 0){ // 前腿(5)
                    dz -= 0.25f * params->RY * fabsf(params->RY); // [新增]
                }else if(leg_idx == 2){ // 后腿(3)
                    dz += 0.35f * params->RY * fabsf(params->RY); // [新增]
                }
            }

            /* 动态参数叠加 */
            switch(leg_idx) {
                case 0: // 前腿
                    dx += params->TX * 0.8f;
                    dy += params->TY * 0.6f;  // [修改] 原0.5->0.6
                    dz += params->RY * 2.0f;  // [新增] RY-Z耦合
                    break;
                case 1: // 中腿
                    dx += params->TX * 0.2f;
                    dy += params->TY * 0.8f;  // [修改] 原1.0->0.8
                    dz += params->RY * 1.5f;   // [新增]
                    break;
                case 2: // 后腿
                    dx += params->TX * 0.8f;
                    dy += params->TY * 0.6f;  // [修改] 原0.5->0.6
                    dz -= params->RY * 2.5f;  // [新增] 反向补偿
                    break;
            }

            // [修改] 添加运动约束 --------------------------------------
            const float max_pitch_comp = 15.0f * (pi/180.0f); // [新增]
            if(fabsf(params->RY) > max_pitch_comp){
                dz *= max_pitch_comp / fabsf(params->RY); // [新增]
            }
            // dz += params->TZ; // 根据需求决定是否叠加
            //======= 坐标变换修正结束 =======//
            
            CalculateAndCacheAngles(
                base, 
                dx, 
                dy, 
                dz,
                leg_id,
                group_angles[leg_idx]
            );
        }
        
        MoveLegsGroup(
            group_angles[0],
            group_angles[1],
            group_angles[2],
            (group_idx == 0) ? ACE_GROUP : BDF_GROUP,
            0
        );
    }
}

/**
  * @brief  优化后的AdjustPosture调用接口，可设置Max-Min范围
  *    - 机体中心为原点 
  *    - X轴：指向机器人正前方
  *    - Y轴：指向机器人左侧
  *    - Z轴：垂直向上（遵循右手定则）
	 @param tx  横向平移量（单位：mm）%%
	 *            >0 向后平移 | <0 向前平移

	 * @param ty  侧向平移量（单位：mm）
	 *            >0 向左平移 | <0 向右平移
          
	 * @param tz  垂直升降量（单位：mm）
	 *            >0 抬升机体 | <0 下降机体
            
	 * @param rx  横滚角（绕X轴旋转弧度）
	 *            >0 右侧向下倾斜 | <0 左侧向下倾斜
	 *            示例：rx=0.1f 右侧降低约5.7度
	             
	 * @param ry  俯仰角（绕Y轴旋转弧度）%%
	 *            >0 前身下压姿态 | <0 后身下压姿态
           
	 * @param rz  偏航角（绕Z轴旋转弧度）%%
	 *            >0 顺时针旋转 | <0 逆时针旋转        
	 *           
  * [特殊组合效果]
  *    - (tx,ry)配合可实现波浪形步态	 
  *    - (ty,rx)配合可实现横向平移平衡
  *    - (rz,tz)配合可实现旋转升降动作
  * @retval  用例
  */
void Demo_AdjustPosture(float tx, float ty, float tz, float rx, float ry, float rz)
{
#ifdef _DEBUG
	printf("\ntx:[%f]-ty:[%f]-tz:[%f]\n rx:[%f]-ry:[%f]-rz[%f]\n",tx,ty,tz,rz,ry,rz);
#endif
    MotionParams params = {
        .TX = clamp(tx, -20.0f, 20.0f),
        .TY = clamp(ty, -10.0f, 10.0f),
        .TZ = clamp(tz, -30.0f, 30.0f),
        .RX = clamp(rx, -20.0f, 20.0f) * DEG2RAD,
        .RY = clamp(ry, -30.0f, 30.0f) * DEG2RAD,
        .RZ = clamp(rz, -35.0f, 35.0f) * DEG2RAD
    };
    AdjustPosture(&params);
}
void KinematicStop_Gait()
{
	Single_Leg(0,Tissue_leg[0].Radian[0],Tissue_leg[0].Radian[1],Tissue_leg[0].Radian[2],0);//0号——A_Leg
	Single_Leg(1,Tissue_leg[1].Radian[0],Tissue_leg[1].Radian[1],Tissue_leg[1].Radian[2],0);//1号——B_Leg
	Single_Leg(2,Tissue_leg[2].Radian[0],Tissue_leg[2].Radian[1],Tissue_leg[2].Radian[2],0);//2号——C_Leg
	Single_Leg(3,Tissue_leg[3].Radian[0],Tissue_leg[3].Radian[1],Tissue_leg[3].Radian[2],0);//3号——D_Leg
	Single_Leg(4,Tissue_leg[4].Radian[0],Tissue_leg[4].Radian[1],Tissue_leg[4].Radian[2],0);//4号——E_Leg
	Single_Leg(5,Tissue_leg[5].Radian[0],Tissue_leg[5].Radian[1],Tissue_leg[5].Radian[2],0);//5号——F_Leg
}

/**
  * @brief  GetBasePosition   获取基准坐标系，
  * @param  params 运动参数调整 
  * @retval  void
  */
CoordinateAxis GetBasePosition(const MotionParams* params) {
    CoordinateAxis base = Pi3_csys[0];
    base.x += params->TX;
    base.y += params->TY;
    base.z += params->TZ;
    return base;
}

/**
  * @brief  CalculateAndCacheAngles  通用逆运动学计算和角度缓存
  * @param  base：基准坐标
  * @param  dx：基于基准x轴的移动偏量
  * @param  dy：基于基准y轴的移动偏量
  * @param  dz：基于基准z轴的移动偏量
  * @param  legid：
  * @param  output_array：
  * @retval  用例
  */

void CalculateAndCacheAngles(CoordinateAxis base, float dx, float dy, float dz,
                            uint8_t leg_id, float* output_array) {
    //计算目标坐标
    CoordinateAxis target = {
        .x = base.x + dx,
        .y = base.y + dy,
        .z = base.z + dz
    };
    //逆运动学计算目标目标解算出的舵机打角
    InverseKinematics(target.x, target.y, target.z, leg_id);
    // 缓存计算结果
    output_array[0] = Tissue_leg[leg_id].Radian[0];
    output_array[1] = Tissue_leg[leg_id].Radian[1];
    output_array[2] = Tissue_leg[leg_id].Radian[2];
}

/**
  * @brief   MoveLegsGroup 腿部控制函数
  * @param  *foreleg  前腿腿部舵机角度数组指针
  * @param  *middleleg 中腿腿部舵机角度数组指针
  * @param  *hidleg 后退腿部舵机角度数组指针
  * @param  gtoup LegGroup枚举 值可以为ACE-GROUP，BDF-GROUP，判断是需要移动的哪一侧腿部
  * @param  duration time
  * @retval  none
  */

void MoveLegsGroup(float* foreleg, float* middleleg, float* hindleg,
                  enum LegGroup group, int duration) {
    if(group == ACE_GROUP) {
        Leg_ACE_Move(foreleg[0], foreleg[1], foreleg[2],
                    middleleg[0], middleleg[1], middleleg[2],
                    hindleg[0], hindleg[1], hindleg[2], duration);
    } else {
        Leg_BDF_Move(foreleg[0], foreleg[1], foreleg[2],
                    middleleg[0], middleleg[1], middleleg[2],
                    hindleg[0], hindleg[1], hindleg[2], duration);
    }
}

/**
  * @brief  ExecuteGaitPhase 步态封装
  * @param  group LegGroup 枚举  值可以为ACE-GROUP，BDF-GROUP，判断是需要移动的哪一侧腿部
  * @param  type GaitType枚举  值可以为 @ref GaitType 的值
  * @param  params MotionParams结构体，控制其三维方面的X轴，Y轴，Z轴偏量及各个偏角
  * @param  phase 第几个阶段，通过不同的阶段，调整dx,dy,dz,例如 一侧腿分为三个解算 0-抬起，1-落地，2-移动
  * @retval  none
  */
void ExecuteGaitPhase(enum LegGroup group, enum GaitType type,
                     const MotionParams* params, uint8_t phase) {
    static CoordinateAxis base_position;
    base_position = GetBasePosition(params);

    float dx = 0.0f, dy = 0.0f, dz = 0.0f;
	const float step_scale = 0.8f; // 步幅缩放系数

    switch(type) {
        case FORWARD:
            dz = (phase == 0) ? Hip_edge : 0.0f;
            dx = SL_half_edge * ((phase == 1) ? 1.0f : -1.0f);
            break;
		
        case BACKWARD:
            dz = (phase == 0) ? Hip_edge : 0.0f;
            dx = -SL_half_edge * ((phase == 1) ? 1.0f : -1.0f);
            dy = -0.4f;
            break;
		
		case SIDESTEP_L: 
			dz = Hip_edge * ((phase == 0) ? 1.0f : 0.0f);
			dy = -SL_half_edge * 0.7f * ((phase == 1) ? 1.0f : -0.5f); // 方向修正
			break;

		case SIDESTEP_R:

			dz = Hip_edge * ((phase == 0) ? 0.4f : 0.0f);
			dy = SL_half_edge * 0.7f * ((phase == 1) ? 1.0f : -0.5f); // 降低回撤幅度
			break;
		
        case TURN_LEFT: {
            const float turn_ratio = (group == ACE_GROUP) ? -1.2f : 1.2f;
            dz = Hip_edge * 0.8f;
            dx = SL_half_edge * turn_ratio * ((phase == 1) ? 1.0f : -1.0f);
            dy = (group == ACE_GROUP) ? 0.4f : -0.4f; // 增大横向偏移
            break;
        }
        
        case TURN_RIGHT: {
            const float turn_ratio = (group == ACE_GROUP) ? 1.2f : -1.2f;
            dz = Hip_edge * 0.8f;
            dx = SL_half_edge * turn_ratio * ((phase == 1) ? 1.0f : -1.0f);
            dy = (group == ACE_GROUP) ? -0.4f : 0.4f;
            break;
        }
    }
    uint8_t foreleg_id, middleleg_id, hindleg_id;
    if(group == ACE_GROUP) {
        foreleg_id = 2; middleleg_id = 4; hindleg_id = 0;
    } else {
        foreleg_id = 5; middleleg_id = 1; hindleg_id = 3;
    }
// 调试输出当前参数
    #ifdef _DEBUG
   
    #endif
// 调整Y轴偏移方向（后退特有）
    float y_offset = (type == BACKWARD) ? -dy : dy;
	if(type==FORWARD||type==BACKWARD)
	{
		CalculateAndCacheAngles(base_position, dx, y_offset*0.25f, dz, foreleg_id, Move_Theta_Foreleg);
		CalculateAndCacheAngles(base_position, dx, 0.0f, dz, middleleg_id, Move_Theta_Middleleg);
		CalculateAndCacheAngles(base_position, dx, -y_offset*0.25f, dz, hindleg_id, Move_Theta_Hindleg);
	}else if(type==SIDESTEP_L||type==SIDESTEP_R)
	{
		CalculateAndCacheAngles(base_position, dx, dy, dz, foreleg_id, Move_Theta_Foreleg);
		CalculateAndCacheAngles(base_position, 0.0f, dy, dz, middleleg_id, Move_Theta_Middleleg);
		CalculateAndCacheAngles(base_position, dx,dy, dz, hindleg_id, Move_Theta_Hindleg);
	}
	else{
		CalculateAndCacheAngles(base_position, dx, y_offset*0.25f, dz, foreleg_id, Move_Theta_Foreleg);
		CalculateAndCacheAngles(base_position, dx, 0.0f, dz, middleleg_id, Move_Theta_Middleleg);
		CalculateAndCacheAngles(base_position, dx, -y_offset*0.25f, dz, hindleg_id, Move_Theta_Hindleg);
	}

    MoveLegsGroup(Move_Theta_Foreleg, Move_Theta_Middleleg, Move_Theta_Hindleg,
                 group, Gait_Time);
    
    if(phase != 0) Delay_Ms(Gait_Time);
}

/**
* @brief  DefaultMotionParams 默认偏量值初始化函数
  * @param  void
  * @retval  none
  */
MotionParams DefaultMotionParams(void) {
    return (MotionParams){
        .RX = 0.0f, .RY = 0.0f, .RZ = 0.0f,
        .TX = 0.0f, .TY = 0.0f, .TZ = 0.0f
    };
}

void X_Advance_Gait(const MotionParams* params){
    MotionParams lifted = *params;
    lifted.TZ += Hip_edge;  // 抬腿高度参数化

    // BDF组抬腿
    ExecuteGaitPhase(BDF_GROUP, FORWARD, &lifted, 0);
    // ACE组移动
    ExecuteGaitPhase(ACE_GROUP, FORWARD, params, 2);
    // BDF组落地
    ExecuteGaitPhase(BDF_GROUP, FORWARD, params, 1);

    // ACE组抬腿
    ExecuteGaitPhase(ACE_GROUP, FORWARD, &lifted, 0);
    // BDF组移动
    ExecuteGaitPhase(BDF_GROUP, FORWARD, params, 2);
    // ACE组落地
    ExecuteGaitPhase(ACE_GROUP, FORWARD, params, 1);
}

void X_Backward_Gait(const MotionParams* params) {
    MotionParams lifted = *params;
    lifted.TZ += Hip_edge;  // 抬腿高度

    // BDF组抬腿
    ExecuteGaitPhase(BDF_GROUP, BACKWARD, &lifted, 0);
    // ACE组向后移动身体
    ExecuteGaitPhase(ACE_GROUP, BACKWARD, params, 2);
    // BDF组落地
    ExecuteGaitPhase(BDF_GROUP, BACKWARD, params, 1);

    // ACE组抬腿
    ExecuteGaitPhase(ACE_GROUP, BACKWARD, &lifted, 0);
    // BDF组向后移动身体
    ExecuteGaitPhase(BDF_GROUP, BACKWARD, params, 2);
    // ACE组落地
    ExecuteGaitPhase(ACE_GROUP, BACKWARD, params, 1);
}

/* 左侧移步态 */
void X_Sidestep_L_Gait(const MotionParams* params) {
    MotionParams lifted = *params;
    lifted.TZ += Hip_edge;

    ExecuteGaitPhase(BDF_GROUP, SIDESTEP_L, &lifted, 0);
    ExecuteGaitPhase(ACE_GROUP, SIDESTEP_L, params, 2);
    ExecuteGaitPhase(BDF_GROUP, SIDESTEP_L, params, 1);

    ExecuteGaitPhase(ACE_GROUP, SIDESTEP_L, &lifted, 0);
    ExecuteGaitPhase(BDF_GROUP, SIDESTEP_L, params, 2);
    ExecuteGaitPhase(ACE_GROUP, SIDESTEP_L, params, 1);
}

/* 右侧移步态 */
void X_Sidestep_R_Gait(const MotionParams* params) {
    MotionParams lifted = *params;
    lifted.TZ += Hip_edge;

    ExecuteGaitPhase(BDF_GROUP, SIDESTEP_R, &lifted, 0);
    ExecuteGaitPhase(ACE_GROUP, SIDESTEP_R, params, 2);
    ExecuteGaitPhase(BDF_GROUP, SIDESTEP_R, params, 1);

    ExecuteGaitPhase(ACE_GROUP, SIDESTEP_R, &lifted, 0);
    ExecuteGaitPhase(BDF_GROUP, SIDESTEP_R, params, 2);
    ExecuteGaitPhase(ACE_GROUP, SIDESTEP_R, params, 1);
}

/* 左转步态 */
void X_Turn_Left_Gait(const MotionParams* params) {
    MotionParams lifted = *params;
    lifted.TZ += Hip_edge;

    ExecuteGaitPhase(BDF_GROUP, TURN_LEFT, &lifted, 0);
    ExecuteGaitPhase(ACE_GROUP, TURN_LEFT, params, 2);
    ExecuteGaitPhase(BDF_GROUP, TURN_LEFT, params, 1);

    ExecuteGaitPhase(ACE_GROUP, TURN_LEFT, &lifted, 0);
    ExecuteGaitPhase(BDF_GROUP, TURN_LEFT, params, 2);
    ExecuteGaitPhase(ACE_GROUP, TURN_LEFT, params, 1);
}

/* 右转步态 */
void X_Turn_Right_Gait(const MotionParams* params) {
    MotionParams lifted = *params;
    lifted.TZ += Hip_edge;

    ExecuteGaitPhase(BDF_GROUP, TURN_RIGHT, &lifted, 0);
    ExecuteGaitPhase(ACE_GROUP, TURN_RIGHT, params, 2);
    ExecuteGaitPhase(BDF_GROUP, TURN_RIGHT, params, 1);

    ExecuteGaitPhase(ACE_GROUP, TURN_RIGHT, &lifted, 0);
    ExecuteGaitPhase(BDF_GROUP, TURN_RIGHT, params, 2);
    ExecuteGaitPhase(ACE_GROUP, TURN_RIGHT, params, 1);
}

void Demo_DefaultAdvance() {
    MotionParams params = DefaultMotionParams(); 
    X_Advance_Gait(&params); 
}
void Demo_DefaultBack() {
    MotionParams params = DefaultMotionParams(); 
    X_Backward_Gait(&params); 
}
void Demo_DefaultTurn_Left() {
    MotionParams params = DefaultMotionParams(); 
    X_Turn_Left_Gait(&params); 
}
void Demo_DefaultTurn_Right() {
    MotionParams params = DefaultMotionParams(); 
    X_Turn_Right_Gait(&params); 
}

void Demo_DefaultSidestep_L() {
    MotionParams params = DefaultMotionParams(); 
    X_Sidestep_L_Gait(&params); 
}

void Demo_DefaultSidestep_R() {
    MotionParams params = DefaultMotionParams(); 
    X_Sidestep_R_Gait(&params); 
}
