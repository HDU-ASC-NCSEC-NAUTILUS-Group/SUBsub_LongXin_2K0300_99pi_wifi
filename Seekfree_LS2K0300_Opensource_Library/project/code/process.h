/*******************************************************************************
* 上板-下板数据传输 + 机械臂抓取 进程文件
*
* 双层非阻塞状态机：
*   外层 transport(): QR识别/传输 → 机械臂复位 → 物块跟踪(10帧稳定) → 抓取 → 终止
*   内层 arm_grasp_tick(): IK解算 → 移动到抓取位 → 夹取 → 抬起 → 完成
*
* 对照 Debug_ARM_Grasp 示例，复位在跟踪之前，夹爪独立控制
* 在 main() 的 while(1) 中调用 transport()
*******************************************************************************/
#ifndef __PROCESS_H__
#define __PROCESS_H__

#include <stdint.h>

// ============================================================
// 外层状态机：传输流程阶段
// ============================================================
enum TransportState {
    TRANSPORT_IDLE      = 0,    // 等待二维码识别 (QR_process)
    TRANSPORT_PROCESS,          // 二维码数据结算组包
    TRANSPORT_SEND,             // UART1 发送至下板
    ARM_RESET_START,            // 机械臂复位（清旧状态 → 初始化 → 逐帧推进）
    TRACK_START,                // 红色物块跟踪，等待10帧坐标稳定
    GRASP_START,                // 机械臂抓取（委托内层状态机）
    GRASP_END,                  // 抓取完成，保持舵机角度，终止
};

// ============================================================
// 内层状态机：机械臂抓取（不含复位，复位在外层 ARM_RESET_START 完成）
// ============================================================
// 机械臂子状态（对照示例 arm_work_state 值）
// 1=解算  2=移动  4=夹爪闭合  5=抬起  0=完成
enum GRASPState {
    GRASP_COMPUTE   = 1,        // 逆运动学解算（示例 state1）
    GRASP_MOVE      = 2,        // 舵机运动（示例 state2）
    GRASP_GRIP      = 4,        // 夹爪闭合（示例 state4）
    GRASP_LIFT      = 5,        // 机械臂抬起（示例 state5）
    GRASP_DONE      = 0,        // 完成（示例 state0）
};

// ---- 外层状态机接口 ----

void transport(void);
void transport_reset(void);
int  transport_get_state(void);
const char* transport_get_last_sent(void);

// ---- 内层状态机接口 ----

int  arm_grasp_get_state(void);

#endif
