#include "zf_common_headfile.h"
#include <math.h>       // sqrt, atan2, acos, M_PI

// 最终用于抓取解析的坐标值
float x_result = 0.0f, y_result = 0.0f;

// 初始化为复位角度
float target_angle[JOINT_COUNT]
//  = {
//     [NAME_JOINT_BASE]          = ANGLE_ZERO_BASE,
//     [NAME_JOINT_ARM_1]         = ANGLE_ZERO_ARM_1,
//     [NAME_JOINT_ARM_2]         = ANGLE_ZERO_ARM_2,
//     [NAME_JOINT_GRIPPER]       = ANGLE_ZERO_GRIPPER,
//     [NAME_JOINT_GRIPPER_WRIST] = ANGLE_ZERO_WRIST,
// }
;
float current_angle[JOINT_COUNT];  // 当前角度(实际上是无法直接读取到实际角度的，只是方便状态控制)

/**********************************************************/
/*[S] 抓取解析 [S]-----------------------------------------*/
/**********************************************************/

/*
 * 逆运动学: 物体坐标 → 关节角度
 *
 * 模型说明:
 *   底座旋转使机械臂平面朝向物体
 *   机械臂平面内: 二连杆 (一大臂 L1 + 二大臂 L2) 从上往下够取物体
 *   物体在平台上，平台低于底座旋转中心 LEN_HIGH_OFFSET
 *
 * 角度约定:
 *   底座 90° = 机械臂朝向 +Y
 *   一大臂 90° = 竖直向上 (Z+)
 *   二大臂 90° = 竖直向上 (Z+)
 *
 * DIR_BASE / DIR_ARM_1 / DIR_ARM_2 控制旋转方向极性
 */
int grasp_compute_angles(void)
{
    float dx, dy;           // 物体相对于底座的坐标差
    float h_dist;           // 底座到物体的水平距离
    float z_reach;          // 底座到物体的垂直落差
    float r_sq;             // 水平距离与垂直落差的平方和
    float cos_a2;           // 余弦定理计算二大臂夹角的 cos(α₂)
    float a1, a2;           // α₁=一大臂与竖直方向夹角, α₂=二大臂与一大臂延长线夹角
    float beta;             // 目标向量(底座→物体)与竖直方向的夹角
    float gamma;            // 一大臂与目标向量的夹角(由L1和L2几何关系求得)
    float base_tar;         // 底座目标角度(0~180)
    float arm1_tar;         // 一大臂目标角度(0~180)
    float arm2_tar;         // 二大臂目标角度(0~180)

    // ------ 1. 底座方位计算 ------
    dx = x_result - BASE_X;
    dy = y_result - BASE_Y;

    base_tar = 90.0f + (float)DIR_BASE * (float)(atan2((double)dx, (double)dy) * (180.0 / M_PI));

    if (base_tar < 0.0f)   base_tar = 0.0f;
    if (base_tar > 180.0f) base_tar = 180.0f;

    // ------ 2. 机械臂平面内的二连杆 IK ------
    h_dist  = sqrtf(dx * dx + dy * dy);
    z_reach = LEN_HIGH_OFFSET;

    r_sq = h_dist * h_dist + z_reach * z_reach;
    cos_a2 = (r_sq - LEN_ARM_1 * LEN_ARM_1 - LEN_ARM_2_TO_GRIPPER * LEN_ARM_2_TO_GRIPPER)
           / (2.0f * LEN_ARM_1 * LEN_ARM_2_TO_GRIPPER);

    // 超出工作空间: 太远 (r > L1+L2) 或 太近 (r < |L1-L2|)
    if (cos_a2 < -1.0f || cos_a2 > 1.0f) {
        return 0;
    }

    a2   = acosf(cos_a2);
    beta = atan2f(h_dist, z_reach);
    gamma = atan2f(LEN_ARM_2_TO_GRIPPER * sinf(a2),
                   LEN_ARM_1 + LEN_ARM_2_TO_GRIPPER * cosf(a2));

    a1 = beta - gamma;

    // 转换为舵机角度 (90° = 竖直向上)
    arm1_tar = 90.0f - (float)DIR_ARM_1 * a1 * (180.0f / (float)M_PI);
    arm2_tar = 90.0f - (float)DIR_ARM_2 * a2 * (180.0f / (float)M_PI);

    // 防止溢出
    if (arm1_tar < 5.0f)   arm1_tar = 5.0f;
    if (arm1_tar > 175.0f) arm1_tar = 175.0f;
    if (arm2_tar < 5.0f)   arm2_tar = 5.0f;
    if (arm2_tar > 175.0f) arm2_tar = 175.0f;

    // ------ 3. 写入全局目标角度 ------
    target_angle[NAME_JOINT_BASE]  = base_tar;
    target_angle[NAME_JOINT_ARM_1] = arm1_tar;
    target_angle[NAME_JOINT_ARM_2] = arm2_tar;

    return 1;
}

/**********************************************************/
/*-----------------------------------------[E] 抓取解析 [E]*/
/**********************************************************/




/**********************************************************/
/*[S] 舵机逐步运动 [S]--------------------------------------*/
/**********************************************************/

#define INVALID_ANGLE   -999.0f             // 约定的无效数据

// ============ 运动状态机 ============
enum {
    MOTION_IDLE   = 0,                      // 空闲，等待新目标
    MOTION_MOVING = 1,                      // 运动中，逐帧推进
    MOTION_DONE   = 2,                      // 已完成
};

static int motion_state = MOTION_IDLE;
static float cached_target[JOINT_COUNT] = {
    INVALID_ANGLE,
    INVALID_ANGLE,
    INVALID_ANGLE,
    INVALID_ANGLE,
    INVALID_ANGLE
};                                           // 快照，屏蔽外部改动
static float step_size[JOINT_COUNT];        // 每关节每步增量
static int step_remain;                     // 剩余步数，倒计数

/*
 * 逻辑关节 → PCA9685 物理通道映射
 *
 * 更改接线时只需修改 grasp_controller.h 中 DEFINE_JOINT_* 宏的值，
 * 所有 Servo_Set_Angle 调用自动跟随
 */
static const int joint_channel[JOINT_COUNT] = {
    DEFINE_JOINT_BASE,
    DEFINE_JOINT_ARM_1,
    DEFINE_JOINT_ARM_2,
    DEFINE_JOINT_GRIPPER,
    DEFINE_JOINT_GRIPPER_WRIST
};

/**
 * servo_move_sync - 舵机运动状态机
 * @enable: 1=推进/启动, 0=中止
 *
 * 主循环中每轮调用一次，由 Servo_Set_Angle() 返回值决定实际推进节奏。
 *
 * 返回值: 1=所有关节运动完成, 0=运动中/空闲/中止/冷却期
 *         用于推进上层状态机
 */
int servo_move_sync(int enable)
{
    /* ===== 中止 ===== */
    if (enable == 0) {
        motion_state = MOTION_IDLE;
        step_remain  = 0;
        for (int j = 0; j < JOINT_COUNT; j++) {
            cached_target[j] = INVALID_ANGLE;
        }
        // Stop_Servo_All();  // 可选：立即停舵机
        return 0;
    }

    /* ===== 检测目标变更，触发重启（已禁用）=====
     *
     * 当前版本不自动中止：运动过程中即使 target_angle 被外部改写，
     * 也会先把当前段走完。中途需要中止请显式调用 servo_move_sync(0)。
     *
     * 如需启用此特性，取消下方注释即可。
     */
    // if (motion_state != MOTION_IDLE) {
    //     for (int j = 0; j < JOINT_COUNT; j++) {
    //         float diff = target_angle[j] - cached_target[j];  // 当前目标与快照的偏差
    //         if (diff < -0.1f || diff > 0.1f) {
    //             motion_state = MOTION_IDLE;   // 目标变了，放弃当前运动
    //             break;
    //         }
    //     }
    // }

    /* ===== 空闲态：冻结目标，初始化 ===== */
    if (motion_state == MOTION_IDLE) {
        int has_target = 0;   // 是否有任一关节需要运动
        for (int j = 0; j < JOINT_COUNT; j++) {
            cached_target[j] = target_angle[j];
            float error   = cached_target[j] - current_angle[j];   // 该关节需要走的总角度
            float abs_err = (error < 0.0f) ? -error : error;       // |error|
            if (abs_err >= ANGLE_EPSILON) {
                has_target = 1;
            }
            step_size[j] = error / JOINT_STEP_COUNT;   // 每步增量，可正可负
        }

        if (has_target) {
            step_remain  = JOINT_STEP_COUNT;   // 剩余步数
            motion_state = MOTION_MOVING;
        }
        return 0;
    }

    /* ===== 运动中：尝试推进一步 ===== */
    if (motion_state == MOTION_MOVING) {
        int all_done = 1;    // 本轮所有关节是否都已到位
        int any_sent = 0;    // 本轮是否至少发了一条 I2C 指令

        for (int j = 0; j < JOINT_COUNT; j++) {
            float error   = cached_target[j] - current_angle[j];   // 该关节剩余角度
            float abs_err = (error < 0.0f) ? -error : error;       // |error|

            if (abs_err < ANGLE_EPSILON) {
                continue;     // 已到位，跳过
            }

            all_done = 0;

            float step_target = current_angle[j] + step_size[j];   // 本步目标角度
            if (step_remain <= 1) {
                step_target = cached_target[j];   // 最后一步直接拉到终值
            }

            if (Servo_Set_Angle(joint_channel[j], step_target)) {
                current_angle[j] = step_target;   // 指令已下发，更新跟踪值
                any_sent = 1;
            }
            // 返回 0 表示冷却期内，本轮跳过，下次继续尝试此关节
        }

        // 所有关节都追上步目标 → 步进
        if (all_done) {
            step_remain--;
            if (step_remain <= 0) {
                motion_state = MOTION_DONE;
            }
        }

        if (!any_sent) {
            return 0;   // 冷却期中，空过一轮
        }
    }

    /* ===== 完成态：等待新目标 ===== */
    if (motion_state == MOTION_DONE) {
        return 1;   // 运动完成，通知上层
    }

    return 0;
}

/**********************************************************/
/*--------------------------------------[E] 舵机逐步运动 [E]*/
/**********************************************************/
