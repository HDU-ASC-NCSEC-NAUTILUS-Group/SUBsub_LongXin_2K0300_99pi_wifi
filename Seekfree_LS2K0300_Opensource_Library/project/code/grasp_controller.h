/*******************************************************************************
* 机械臂抓取控制文件
*******************************************************************************/
#ifndef __GRASP_CONTROLLER_H__
#define __GRASP_CONTROLLER_H__


// 机械臂关节总数
#define JOINT_COUNT                     5

/*
 * 逻辑关节索引（固定值，用于 target_angle[] 等数组下标）
 *
 * 始终按此顺序: BASE=0, ARM_1=1, ARM_2=2, GRIPPER=3, WRIST=4
 */
#define NAME_JOINT_BASE                 0       // 底座 = 数组下标逻辑关节 0
#define NAME_JOINT_ARM_1                1       // 一大臂 = 数组下标逻辑关节 1
#define NAME_JOINT_ARM_2                2       // 二大臂 = 数组下标逻辑关节 2
#define NAME_JOINT_GRIPPER              3       // 夹爪 = 数组下标逻辑关节 3
#define NAME_JOINT_GRIPPER_WRIST        4       // 夹爪旋转 = 数组下标逻辑关节 4

/*
 * PCA9685 物理通道映射
 *
 * 更改接线时只需修改此处数值，所有 Servo_Set_Angle 调用自动跟随
 *   例: 底座插在 PCA9685 通道 4 → #define DEFINE_JOINT_BASE 4
 *   设为 -1 则该关节被禁用，servo_move_sync / servo_reset_init 均跳过
 */
#define DEFINE_JOINT_BASE               0       // 底座 → PCA9685 通道号
#define DEFINE_JOINT_ARM_1              1       // 一大臂 → PCA9685 通道号
#define DEFINE_JOINT_ARM_2              2       // 二大臂 → PCA9685 通道号
#define DEFINE_JOINT_GRIPPER            3       // 夹爪 → PCA9685 通道号
#define DEFINE_JOINT_GRIPPER_WRIST      -1      // 夹爪旋转 → 禁用(-1)

// 机械臂关节默认角度（复位位置）
#define ANGLE_ZERO_BASE                 90.0f   // 底座
#define ANGLE_ZERO_ARM_1                90.0f   // 一大臂
#define ANGLE_ZERO_ARM_2                135.0f   // 二大臂
#define ANGLE_ZERO_GRIPPER              145.0f   // 夹爪
#define ANGLE_ZERO_WRIST                0.0f    // 夹爪旋转

#define JOINT_STEP_COUNT                40      // 固定走 STEP_COUNT 步
                                                // 所有关节同时到位

#define ANGLE_EPSILON                   0.5f    // 角度到位容差(度)，误差 < 0.1° 视为到位

// 物理长度定义(单位:cm)
#define LEN_ARM_1                       7.47f   // 一大臂旋转中心到二大臂旋转中心距离
#define LEN_ARM_2_TO_GRIPPER            21.5f   // 二大臂旋转中心到夹爪夹取中心距离
#define LEN_HIGH_OFFSET                 10.0f   // 一大臂旋转中心与夹取物体平台高差

// 底座在世界坐标系中的位置(单位:cm)
// 用于可能需要的校准
#define BASE_X                          0.0f    // TODO: 实测底座在世界坐标系的 X 坐标
#define BASE_Y                          0.0f    // TODO: 实测底座在世界坐标系的 Y 坐标

/*
 * 舵机旋转方向配置 (+1 或 -1)
 *
 * 描述的是 DIR 取 +1 时的物理效果:
 *   DIR_BASE  +1 → 角度增大时，末端扫向 +X 方向(逆时针)
 *   DIR_ARM_1 +1 → 角度增大时，一大臂向前(向下)翻转
 *   DIR_ARM_2 +1 → 角度增大时，二大臂向前(向下)翻转
 *
 * 实测方向相反则改为 -1
 */
#define DIR_BASE                       -1      // 底座旋转方向
#define DIR_ARM_1                      -1      // 一大臂弯曲方向
#define DIR_ARM_2                       1      // 二大臂弯曲方向

// 最终用于抓取解析的坐标值(单位:cm)
extern float x_result , y_result;
// 所有舵机目标角度和当前角度(实际上无法直接获取当前角度)的数组
extern float target_angle[JOINT_COUNT];
extern float current_angle[JOINT_COUNT];

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

/*
 * 舵机角度初始化（非阻塞）
 *
 * 按顺序将各关节依次拉回复位角度:
 *   夹爪 → 一大臂 → 二大臂 → 底座
 * 每步成功发送后自动等待 300ms 再进行下一步
 *
 * 调用方式:
 *   servo_reset_init(1);   // 启动: 同步 current_angle/target_angle 为复位值，重置状态机
 *   servo_reset_init(0);   // 中止: 清空状态机（通常不需要调用）
 *   servo_reset_init(2);   // 推进: 主循环每轮调用，推进状态机
 *
 *   if (servo_reset_init(2)) { ... }  // 返回 1 表示全部复位完成
 *
 * 返回值: 1=全部复位完成, 0=进行中
 */
int servo_reset_init(int cmd);

#endif