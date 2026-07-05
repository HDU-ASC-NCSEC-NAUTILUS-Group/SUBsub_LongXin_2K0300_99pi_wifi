/*******************************************************************************
* 上板-下板数据传输 + 机械臂抓取 进程文件
*
*   完整流程（对照 Debug_ARM_Grasp 示例）:
*     1. 二维码识别 → 结算 → UART1 发送至下板
*     2. 机械臂复位（清旧状态 → 初始化 → 逐帧推进，恢复到初始角度）
*     3. 摄像头物块跟踪（10帧稳定检测）
*     4. 机械臂抓取: IK解算 → 移动 → 夹爪闭合 → 抬起
*     5. 终止，舵机角度硬件保持
*
*   复位在跟踪之前——机械臂先收起来，给摄像头让出视野。
*******************************************************************************/
#include "zf_common_headfile.h"
#include "image_process.h"
#include "grasp_controller.h"
#include "process.h"
#include "zf_driver_uart1.h"

#include <string.h>

// ============================================================
// 常量
// ============================================================

#define STABLE_PIXEL_THRESHOLD  3       // ±3 像素
#define STABLE_FRAMES_REQUIRED  10      // 连续稳定帧数

// ============================================================
// 外层状态机变量
// ============================================================

static int       g_transport_state   = TRANSPORT_IDLE;
static char      g_tx_packet[128]    = {0};
static char      g_last_sent[128]    = {0};
static uint32_t  g_send_retry        = 0;

// ---- 复位 ----
static int       g_reset_started     = 0;    // servo_reset_init(1) 已调用过

// ---- 物块跟踪 ----
static int16_t   g_stable_ref_x      = -1;
static int16_t   g_stable_ref_y      = -1;
static int       g_stable_count      = 0;

// ============================================================
// 机械臂子状态（对照示例 arm_work_state 值）
//   1 = 逆运动学解算   2 = 舵机运动   4 = 夹爪闭合   5 = 抬起   0 = 完成
// ============================================================
static int g_arm_state = 1;     // 进入 GRASP_START 时从 1 开始

// ============================================================
// QR 结算：透传原始数据 + \r\n
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
// 外层状态机：transport()
// ============================================================

void transport(void)
{
    switch (g_transport_state) {

    // ======================================================
    // 1. 二维码识别
    // ======================================================
    case TRANSPORT_IDLE: {
        int ret = QR_process();
        if (ret == 1 && g_qr_data_ready) {
            g_transport_state = TRANSPORT_PROCESS;
        }
        break;
    }

    // ======================================================
    // 2. 二维码结算组包
    // ======================================================
    case TRANSPORT_PROCESS: {
        g_qr_data_ready = false;
        gpio_set_level(BEEP, 0x0);
        qr_settlement(g_qr_data, g_tx_packet, sizeof(g_tx_packet));
        g_send_retry = 0;
        g_transport_state = TRANSPORT_SEND;
        break;
    }

    // ======================================================
    // 3. UART1 发送至下板
    // ======================================================
    case TRANSPORT_SEND: {
        int len  = (int)strlen(g_tx_packet);
        int sent = uart1_send((const uint8 *)g_tx_packet, (uint32)len);

        if (sent == len) {
            strncpy(g_last_sent, g_tx_packet, sizeof(g_last_sent) - 1);
            g_last_sent[sizeof(g_last_sent) - 1] = '\0';
            printf("TRANSPORT:QR sent -> %s", g_last_sent);
            // QR 完成，先复位机械臂
            g_reset_started = 0;
            g_transport_state = ARM_RESET_START;
        } else if (g_send_retry < 3) {
            g_send_retry++;
        } else {
            g_last_sent[0] = '\0';
            g_reset_started = 0;
            g_transport_state = ARM_RESET_START;
        }
        break;
    }

    // ======================================================
    // 4. 机械臂复位（对照示例: 启动 state3 复位，在跟踪之前）
    //    先清旧状态，再初始化，然后逐帧推进
    // ======================================================
    case ARM_RESET_START: {
        if (!g_reset_started) {
            // 对照示例 line 618-620: 清旧状态
            servo_move_sync(0);
            servo_reset_init(0);
            Stop_Servo_All();
            // 对照示例 line 593: 启动复位
            servo_reset_init(1);
            g_reset_started = 1;
        }
        // 对照示例 line 627: 逐帧推进
        if (servo_reset_init(2)) {
            printf("ARM:RESET-DONE\n");
            g_transport_state = TRACK_START;
        }
        break;
    }

    // ======================================================
    // 5. 物块跟踪（10帧稳定检测）
    //    对照示例 line 638-667 (对象可见后才开始累积)
    // ======================================================
    case TRACK_START: {
        int ret = object_tracking();

        if (ret == 1) {
            coordinate_transformation();

            int16_t dx = (coordinate_x > g_stable_ref_x)
                       ? (int16_t)(coordinate_x - g_stable_ref_x)
                       : (int16_t)(g_stable_ref_x - coordinate_x);
            int16_t dy = (coordinate_y > g_stable_ref_y)
                       ? (int16_t)(coordinate_y - g_stable_ref_y)
                       : (int16_t)(g_stable_ref_y - coordinate_y);

            if (dx <= STABLE_PIXEL_THRESHOLD && dy <= STABLE_PIXEL_THRESHOLD) {
                g_stable_count++;
                if (g_stable_count >= STABLE_FRAMES_REQUIRED) {
                    x_result = real_x;
                    y_result = real_y;
                    printf("TRANSPORT:stable %d frames, pos(%.2f,%.2f)\n",
                           g_stable_count, (double)real_x, (double)real_y);

                    g_arm_state       = 1;     // 从 COMPUTE 开始
                    g_transport_state = GRASP_START;
                }
            } else {
                g_stable_ref_x = coordinate_x;
                g_stable_ref_y = coordinate_y;
                g_stable_count = 1;
            }
        } else if (ret == -1) {
            g_stable_count = 0;
        }
        break;
    }

    // ======================================================
    // 6. 机械臂抓取
    //    逐行对照 Debug_ARM_Grasp while(1) 循环体 (line 635-701)
    // ======================================================
    case GRASP_START: {
        // ---- 对照示例 line 635-643: 视觉识别（每轮都调） ----
        int Track_Success = 0;
        if (object_tracking() == 1) {
            coordinate_transformation();
            Track_Success = 1;
        }

        // ---- 对照示例 line 645-666: 解算（g_arm_state==1 时做一次） ----
        if (g_arm_state == 1) {
            if (grasp_compute_angles()) {
                printf("ARM:B=%.1f A1=%.1f A2=%.1f\n",
                       (double)target_angle[NAME_JOINT_BASE],
                       (double)target_angle[NAME_JOINT_ARM_1],
                       (double)target_angle[NAME_JOINT_ARM_2]);
                g_arm_state = 2;
            } else {
                printf("ARM:target out of reach\n");
                g_transport_state = GRASP_END;
            }
        }

        // ---- 对照示例 line 669-677: 舵机运动中 ----
        if (g_arm_state == 2) {
            if (servo_move_sync(1)) {
                g_arm_state = 4;
                printf("ARM:MOVE-DONE!\n");
            }
        }

        // ---- 对照示例 line 679-690: 夹爪闭合（一步到位） ----
        if (g_arm_state == 4) {
            Servo_Set_Angle(DEFINE_JOINT_GRIPPER, 60.0f);
            current_angle[NAME_JOINT_GRIPPER] = 60.0f;
            // 设定抬起目标角度（夹爪保持在60°不张开）
            target_angle[NAME_JOINT_GRIPPER] = 60.0f;
            target_angle[NAME_JOINT_BASE]    = 90.0f;
            target_angle[NAME_JOINT_ARM_1]   = 90.0f;
            target_angle[NAME_JOINT_ARM_2]   = 90.0f;
            g_arm_state = 5;
            printf("ARM:GRIP-CLOSE!\n");
        }

        // ---- 对照示例 line 693-701: 机械臂抬起 ----
        if (g_arm_state == 5) {
            if (servo_move_sync(1)) {
                g_transport_state = GRASP_END;
                printf("ARM:LIFT-DONE!\n");
            }
        }
        break;
    }

    // ======================================================
    // 7. 终止
    // ======================================================
    case GRASP_END:
    default:
        break;

    } // end switch
}

// ============================================================
// 复位接口
// ============================================================

void transport_reset(void)
{
    g_transport_state = TRANSPORT_IDLE;
    g_qr_data_ready   = false;
    g_tx_packet[0]    = '\0';
    g_last_sent[0]    = '\0';
    g_send_retry      = 0;
    g_reset_started   = 0;
    g_stable_ref_x    = -1;
    g_stable_ref_y    = -1;
    g_stable_count    = 0;
    g_arm_state        = 1;
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
    return g_arm_state;
}

const char* transport_get_last_sent(void)
{
    return g_last_sent;
}
