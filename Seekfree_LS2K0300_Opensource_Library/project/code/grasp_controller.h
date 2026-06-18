/*******************************************************************************
* 机械臂抓取控制文件
*******************************************************************************/
#ifndef __GRASP_CONTROLLER_H__
#define __GRASP_CONTROLLER_H__


// 机械臂关节索引   
#define JOINT_COUNT                 5       // 舵机总数(兴许用不上这个宏定义)

#define JOINT_BASE                  0       // 底座
#define JOINT_ARM_1                 1       // 一大臂
#define JOINT_ARM_2                 2       // 二大臂
#define JOINT_GRIPPER               3       // 夹爪
#define JOINT_GRIPPER_WRIST         4       // 夹爪旋转

// 机械臂关节默认角度（复位位置）
#define ANGLE_ZERO_BASE             90.0f   // 底座
#define ANGLE_ZERO_ARM_1            90.0f   // 一大臂
#define ANGLE_ZERO_ARM_2            20.0f   // 二大臂
#define ANGLE_ZERO_GRIPPER          90.0f   // 夹爪
#define ANGLE_ZERO_WRIST            0.0f    // 夹爪旋转

#define JOINT_STEP_COUNT            40      // 固定走 STEP_COUNT 步
                                            // 所有关节同时到位

#define ANGLE_EPSILON               0.5f    // 角度到位容差(度)，误差 < 0.1° 视为到位

// 物理长度定义(单位:cm)
#define LEN_ARM_1                   7.47f   // 一大臂旋转中心到二大臂旋转中心距离
#define LEN_ARM_2_TO_GRIPPER        21.5f   // 二大臂旋转中心到夹爪夹取中心距离
#define LEN_HIGH_OFFSET             9.11f   // 一大臂旋转中心与夹取物体平台高差

// 底座在世界坐标系中的位置(单位:cm)
// 用于可能需要的校准
#define BASE_X                      0.0f    // TODO: 实测底座在世界坐标系的 X 坐标
#define BASE_Y                      0.0f    // TODO: 实测底座在世界坐标系的 Y 坐标

/*
 * 舵机旋转方向配置 (1 或 -1)
 *
 * DIR_BASE  =  1  表示底座角度增大时机械臂朝 +X 方向旋转
 * DIR_ARM_1 =  1  表示一大臂角度增大时末端朝前(Y+大致方向)运动
 * DIR_ARM_2 =  1  表示二大臂角度增大时末端朝前运动
 *
 * 方向不确定时保持默认值，联调时根据实测翻转
 */
#define DIR_BASE                    1       // 底座旋转方向
#define DIR_ARM_1                   1       // 一大臂弯曲方向
#define DIR_ARM_2                   1       // 二大臂弯曲方向

// 最终用于抓取解析的坐标值(单位:cm)
extern float x_result , y_result;

/*
 * 抓取逆运动学解析
 *
 * 根据 (x_result, y_result) 计算 base / arm_1 / arm_2 的目标角度
 * 结果写入 target_angle[] 数组
 *
 * 返回值: 1=可达, 0=超出机械臂工作空间
 */
int grasp_compute_angles(void);

/*
 * 舵机逐步运动状态机
 *
 * 主循环中每轮调用一次:
 *   if (servo_move_sync(1)) { ... }  // 返回 1 表示运动完成
 *   servo_move_sync(0);              // 紧急中止运动
 *
 * 内部利用 Servo_Set_Angle() 的返回值做冷却期保护，
 * 所有关节同步插值，同时起步、同时到位
 *
 * 返回值: 1=运动完成, 0=运动中/空闲/中止/冷却期
 */
int servo_move_sync(int enable);

#endif