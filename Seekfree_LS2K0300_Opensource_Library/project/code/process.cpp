/*******************************************************************************
* 上板-下板数据传输 + 机械臂抓取 进程文件
*
* 双层非阻塞状态机实现：
*
*   外层 transport() 状态机:
*     TRANSPORT_IDLE    → QR_process() 等待识别
*     TRANSPORT_PROCESS → 二维码数据结算组包
*     TRANSPORT_SEND    → UART1 发送至下板
*     TRACK_START       → object_tracking() 红色物块跟踪, 20帧稳定检测
*     GRASP_START       → 委托内层 arm_grasp_tick() 机械臂状态机
*     GRASP_END         → 终止，舵机角度硬件保持
*
*   内层 arm_grasp_tick() 状态机（独立封装）:
*     GRASP_RESET   → servo_reset_init 复位
*     GRASP_COMPUTE → grasp_compute_angles 逆运动学解算
*     GRASP_MOVE    → servo_move_sync 移动到抓取位置
*     GRASP_GRIP    → 夹爪闭合(60°), 设定抬起目标
*     GRASP_LIFT    → servo_move_sync 抬起(夹爪保持, 其余复位)
*     GRASP_DONE    → 完成
*
* 在 main() 的 while(1) 中每轮调用 transport()
*******************************************************************************/
#include "zf_common_headfile.h"
#include "image_process.h"
#include "grasp_controller.h"
#include "process.h"
#include "zf_driver_uart1.h"

#include <string.h>

// ============================================================
// 常量定义
// ============================================================

#define STABLE_PIXEL_THRESHOLD  3       // 像素坐标"相同"判定阈值 (±3px)
#define STABLE_FRAMES_REQUIRED  20      // 需连续20帧坐标稳定

// ============================================================
// 外层状态机内部变量
// ============================================================

static int       g_transport_state   = TRANSPORT_IDLE;
static char      g_tx_packet[128]    = {0};   // 结算后的待发送数据包
static char      g_last_sent[128]    = {0};   // 最后一次成功发送的数据（调试用）
static uint32_t  g_send_retry        = 0;     // 发送重试计数

// ---- 物块跟踪稳定检测变量 ----
static int16_t   g_stable_ref_x      = -1;    // 基准像素X
static int16_t   g_stable_ref_y      = -1;    // 基准像素Y
static int       g_stable_count      = 0;     // 连续稳定帧计数

// ============================================================
// 内层（机械臂）状态机内部变量
// ============================================================

static int g_grasp_state         = GRASP_RESET;
static int g_grasp_just_entered  = 0;           // 刚进入当前状态标志

// ============================================================
// QR 结算处理：QR_process() 已解码出最终结果，此处仅做 UART 组包
// ============================================================

static void qr_settlement(const char *qr_raw, char *packet, int packet_size)
{
    if (!qr_raw || qr_raw[0] == '\0') {
        packet[0] = '\0';
        return;
    }

    int len = (int)strlen(qr_raw);
    if (len > packet_size - 4) len = packet_size - 4;
    memcpy(packet, qr_raw, len);
    packet[len]     = '\r';
    packet[len + 1] = '\n';
    packet[len + 2] = '\0';
}

// ============================================================
// 内层状态机：arm_grasp_tick()
// 独立于传输流程，每次调用推进一个子步骤后立即返回
// 在 GRASP_START 阶段由 transport() 调用
// ============================================================

static void arm_grasp_tick(void)
{
    switch (g_grasp_state) {

    // ----- 复位：首次调用启动 servo_reset_init(1)，后续轮询推进 -----
    case GRASP_RESET:
        if (g_grasp_just_entered) {
            servo_reset_init(1);
            g_grasp_just_entered = 0;
        }
        if (servo_reset_init(2)) {
            printf("ARM:RESET-DONE\n");
            g_grasp_state = GRASP_COMPUTE;
            g_grasp_just_entered = 1;
        }
        break;

    // ----- 逆运动学解算（单次调用） -----
    case GRASP_COMPUTE:
        if (grasp_compute_angles()) {
            printf("ARM:B=%.1f A1=%.1f A2=%.1f\n",
                   (double)target_angle[NAME_JOINT_BASE],
                   (double)target_angle[NAME_JOINT_ARM_1],
                   (double)target_angle[NAME_JOINT_ARM_2]);
            g_grasp_state = GRASP_MOVE;
            g_grasp_just_entered = 1;
        } else {
            // 目标超出工作空间，直接进入 DONE 跳过抓取
            printf("ARM:target out of reach, skip grasp\n");
            g_grasp_state = GRASP_DONE;
            g_grasp_just_entered = 1;
        }
        break;

    // ----- 移动到抓取位置（逐帧推进 servo_move_sync） -----
    case GRASP_MOVE:
        if (servo_move_sync(1)) {
            printf("ARM:MOVE-DONE\n");
            g_grasp_state = GRASP_GRIP;
            g_grasp_just_entered = 1;
        }
        break;

    // ----- 夹爪闭合 + 设定抬起目标 -----
    case GRASP_GRIP:
        Servo_Set_Angle(DEFINE_JOINT_GRIPPER, 60.0f);
        current_angle[NAME_JOINT_GRIPPER] = 60.0f;
        // 抬起目标：夹爪保持60°不张开，其余关节复位
        target_angle[NAME_JOINT_GRIPPER] = 60.0f;
        target_angle[NAME_JOINT_BASE]    = 90.0f;
        target_angle[NAME_JOINT_ARM_1]   = 90.0f;
        target_angle[NAME_JOINT_ARM_2]   = 90.0f;
        printf("ARM:GRIP-CLOSE\n");
        g_grasp_state = GRASP_LIFT;
        g_grasp_just_entered = 1;
        break;

    // ----- 抬起：夹爪保持闭合，其余关节复位（servo_move_sync 检测到目标变更自动重启） -----
    case GRASP_LIFT:
        if (servo_move_sync(1)) {
            printf("ARM:LIFT-DONE\n");
            g_grasp_state = GRASP_DONE;
            g_grasp_just_entered = 1;
        }
        break;

    // ----- 完成：舵机角度由硬件保持，不再改动 -----
    case GRASP_DONE:
    default:
        break;
    }
}

// ============================================================
// 外层状态机：transport()
// main() 的 while(1) 中每轮调用一次
// ============================================================

void transport(void)
{
    switch (g_transport_state) {

    // ======================================================
    // TRANSPORT_IDLE：驱动二维码识别
    // ======================================================
    case TRANSPORT_IDLE: {
        int ret = QR_process();
        if (ret == 1 && g_qr_data_ready) {
            g_transport_state = TRANSPORT_PROCESS;
        }
        break;
    }

    // ======================================================
    // TRANSPORT_PROCESS：结算二维码数据 → UART 组包
    // ======================================================
    case TRANSPORT_PROCESS: {
        g_qr_data_ready = false;
        qr_settlement(g_qr_data, g_tx_packet, sizeof(g_tx_packet));

        g_send_retry = 0;
        g_transport_state = TRANSPORT_SEND;
        break;
    }

    // ======================================================
    // TRANSPORT_SEND：UART1 发送至下板
    // ======================================================
    case TRANSPORT_SEND: {
        int len  = (int)strlen(g_tx_packet);
        int sent = uart1_send((const uint8 *)g_tx_packet, (uint32)len);

        if (sent == len) {
            strncpy(g_last_sent, g_tx_packet, sizeof(g_last_sent) - 1);
            g_last_sent[sizeof(g_last_sent) - 1] = '\0';
            printf("TRANSPORT:QR sent -> %s", g_last_sent);
            // QR 完成，关闭二维码识别，开启物块跟踪
            g_transport_state = TRACK_START;
        } else if (g_send_retry < 3) {
            g_send_retry++;
        } else {
            g_last_sent[0] = '\0';
            g_transport_state = TRACK_START;
        }
        break;
    }

    // ======================================================
    // TRACK_START：红色物块跟踪，等待20帧坐标稳定
    // ======================================================
    case TRACK_START: {
        int ret = object_tracking();

        if (ret == 1) {
            // 成功追踪到红色物块，更新物理坐标
            coordinate_transformation();

            // 检查与基准坐标是否"相同"（±3像素内视为相同）
            int16_t dx = (coordinate_x > g_stable_ref_x)
                       ? (int16_t)(coordinate_x - g_stable_ref_x)
                       : (int16_t)(g_stable_ref_x - coordinate_x);
            int16_t dy = (coordinate_y > g_stable_ref_y)
                       ? (int16_t)(coordinate_y - g_stable_ref_y)
                       : (int16_t)(g_stable_ref_y - coordinate_y);

            if (dx <= STABLE_PIXEL_THRESHOLD && dy <= STABLE_PIXEL_THRESHOLD) {
                g_stable_count++;
                if (g_stable_count >= STABLE_FRAMES_REQUIRED) {
                    // 20帧稳定，记录目标物理坐标，启动机械臂抓取
                    x_result = real_x;
                    y_result = real_y;
                    printf("TRANSPORT:stable %d frames, pos(%.2f,%.2f) -> arm grasp\n",
                           g_stable_count, (double)real_x, (double)real_y);

                    // 初始化内层状态机
                    g_grasp_state        = GRASP_RESET;
                    g_grasp_just_entered = 1;
                    g_transport_state    = GRASP_START;
                }
            } else {
                // 坐标变化，以新坐标重置计数
                g_stable_ref_x = coordinate_x;
                g_stable_ref_y = coordinate_y;
                g_stable_count = 1;
            }
        } else if (ret == -1) {
            // 已处理但未检测到红色物块，重置计数
            g_stable_count = 0;
        }
        // ret==0 (跳帧/无帧): 不改变计数

        break;
    }

    // ======================================================
    // GRASP_START：委托内层机械臂状态机
    // ======================================================
    case GRASP_START:
        arm_grasp_tick();
        if (g_grasp_state == GRASP_DONE) {
            g_transport_state = GRASP_END;
            printf("TRANSPORT:all done, arm hold at gripper=60deg\n");
        }
        break;

    // ======================================================
    // GRASP_END：全部完成，不再调用任何识别函数
    // 舵机角度由硬件保持（夹爪60°，其余90°）
    // ======================================================
    case GRASP_END:
    default:
        break;

    } // end switch(g_transport_state)
}

// ============================================================
// 复位接口
// ============================================================

void transport_reset(void)
{
    g_transport_state   = TRANSPORT_IDLE;
    g_qr_data_ready     = false;
    g_tx_packet[0]      = '\0';
    g_last_sent[0]      = '\0';
    g_send_retry        = 0;
    g_stable_ref_x      = -1;
    g_stable_ref_y      = -1;
    g_stable_count      = 0;
    g_grasp_state       = GRASP_RESET;
    g_grasp_just_entered = 0;
}

// ============================================================
// 调试接口
// ============================================================

int transport_get_state(void)
{
    return g_transport_state;
}

int arm_grasp_get_state(void)
{
    return g_grasp_state;
}

const char* transport_get_last_sent(void)
{
    return g_last_sent;
}
