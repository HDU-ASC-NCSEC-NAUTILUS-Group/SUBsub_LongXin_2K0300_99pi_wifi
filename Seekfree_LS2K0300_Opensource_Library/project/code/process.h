/*******************************************************************************
* 上板-下板数据传输进程文件
* 非阻塞状态机：二维码识别 -> 数据结算 -> UART1发送至下板
*******************************************************************************/
#ifndef __PROCESS_H__
#define __PROCESS_H__

#include <stdint.h>

// 传输状态枚举
enum TransportState {
    TRANSPORT_IDLE = 0,     // 等待二维码识别
    TRANSPORT_PROCESS,      // 数据结算处理
    TRANSPORT_SEND,         // UART1发送至下板
    TRANSPORT_DONE,         // 已完成一轮传输，停止扫描
};

// 非阻塞传输函数：在定时中断(推荐200ms)中周期性调用
// 每次调用执行一个状态步骤后立即返回，不阻塞中断
// 完成一轮(识别->结算->发送)后进入 DONE 态，停止扫描
void transport(void);

// 复位状态机至 IDLE，用于需要再次扫描二维码时调用
void transport_reset(void);

// 获取当前传输状态（供调试/监控）
int transport_get_state(void);

// 获取最后一次发送的结算数据（供调试）
const char* transport_get_last_sent(void);

#endif
