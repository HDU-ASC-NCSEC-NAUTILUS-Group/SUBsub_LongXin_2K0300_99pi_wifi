/*******************************************************************************
* 上板-下板数据传输 + 机械臂抓取 进程文件
*
* 双层非阻塞状态机：
*   外层 transport(): QR识别/传输 → 物块跟踪(20帧稳定) → 机械臂抓取 → 终止
*   内层 arm_grasp_tick(): 舵机复位 → IK解算 → 移动到抓取位 → 夹取 → 抬起 → 完成
*
* 在 main() 的 while(1) 中调用 transport()
*******************************************************************************/
#ifndef __PROCESS_H__
#define __PROCESS_H__

#include <stdint.h>

// ============================================================
// 外层状态机：传输流程阶段
// ============================================================
enum TransportState {
    TRANSPORT_IDLE   = 0,   // 等待二维码识别 (QR_process)
    TRANSPORT_PROCESS,      // 二维码数据结算组包
    TRANSPORT_SEND,         // UART1 发送至下板
    TRACK_START,            // 红色物块跟踪，等待20帧坐标稳定
    GRASP_START,            // 机械臂抓取（委托内层状态机）
    GRASP_END,              // 抓取完成，保持舵机角度，终止
};

// ============================================================
// 内层状态机：机械臂抓取（独立状态机，与传输流程分离）
// ============================================================
enum GRASPState {
    GRASP_RESET   = 0,      // 舵机复位至初始角度
    GRASP_COMPUTE,          // 逆运动学解算目标角度
    GRASP_MOVE,             // 移动到抓取位置
    GRASP_GRIP,             // 夹爪闭合（60°），设定抬起目标
    GRASP_LIFT,             // 机械臂抬起（夹爪保持闭合，其余复位）
    GRASP_DONE,             // 抓取完成
};

// ---- 外层状态机接口 ----

void transport(void);
void transport_reset(void);
int  transport_get_state(void);
const char* transport_get_last_sent(void);

// ---- 内层状态机接口 ----

int  arm_grasp_get_state(void);

#endif
