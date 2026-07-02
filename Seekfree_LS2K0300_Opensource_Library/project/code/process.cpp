/*******************************************************************************
* 上板-下板数据传输进程文件
* 非阻塞状态机实现：
*   IDLE    -> QR_process() 等待识别
*   PROCESS -> 解析二维码数据、结算处理
*   SEND    -> UART1 发送至下板
* 在定时中断中周期性调用 transport()，每次调用执行一步后立即返回
*******************************************************************************/
#include "zf_common_headfile.h"
#include "image_process.h"
#include "process.h"
#include "zf_driver_uart1.h"

#include <string.h>
#include <stdlib.h>

// ============================================================
// 状态机内部变量
// ============================================================

static int       g_transport_state   = TRANSPORT_IDLE;
static char      g_tx_packet[128]    = {0};   // 结算后的待发送数据包
static char      g_last_sent[128]    = {0};   // 最后一次成功发送的数据（调试用）
static uint32_t  g_send_retry        = 0;     // 发送重试计数

// ============================================================
// 结算处理：将原始二维码字符串解析为下板可执行的指令格式
// 返回 0=成功, -1=解析失败（数据仍可通过原始格式发送）
// ============================================================

// 简易 key=value 解析辅助
static int parse_kv(const char *raw, const char *key, char *value, int value_size)
{
    const char *p = strstr(raw, key);
    if (!p) return -1;

    p += strlen(key);
    if (*p == '=' || *p == ':') p++;

    int i = 0;
    while (*p && *p != ',' && *p != ';' && *p != ' ' && *p != '\n' && *p != '\r'
           && i < value_size - 1)
    {
        value[i++] = *p++;
    }
    value[i] = '\0';
    return (i > 0) ? 0 : -1;
}

// 结算主逻辑：从原始QR数据提取有效信息，组装下板指令包
static int qr_settlement(const char *qr_raw, char *packet, int packet_size)
{
    if (!qr_raw || qr_raw[0] == '\0') {
        return -1;
    }

    // 尝试解析坐标格式 "X:xxx,Y:xxx" 或 "X=xxx;Y=xxx"
    char x_str[32] = {0};
    char y_str[32] = {0};
    int has_x = (parse_kv(qr_raw, "X", x_str, sizeof(x_str)) == 0);
    int has_y = (parse_kv(qr_raw, "Y", y_str, sizeof(y_str)) == 0);

    if (has_x && has_y) {
        // 二维码含坐标信息 -> 组装导航目标指令
        snprintf(packet, packet_size, "TARGET:%.6s,%.6s\r\n", x_str, y_str);
        return 0;
    }

    // 尝试解析任务ID格式 "TASK:xxx" 或 "ID:xxx"
    char task_id[64] = {0};
    if (parse_kv(qr_raw, "TASK", task_id, sizeof(task_id)) == 0 ||
        parse_kv(qr_raw, "ID",   task_id, sizeof(task_id)) == 0)
    {
        snprintf(packet, packet_size, "TASK:%s\r\n", task_id);
        return 0;
    }

    // 兜底：将原始QR数据整体透传，标记为RAW类型
    int copy_len = (int)strlen(qr_raw);
    if (copy_len > packet_size - 8) copy_len = packet_size - 8;
    memcpy(packet, "RAW:", 4);
    memcpy(packet + 4, qr_raw, copy_len);
    packet[4 + copy_len]     = '\r';
    packet[4 + copy_len + 1] = '\n';
    packet[4 + copy_len + 2] = '\0';
    return 0;
}

// ============================================================
// transport() — 非阻塞传输状态机
// 调用方：定时中断回调 (推荐 pit_callback_200ms)
// ============================================================

void transport(void)
{
    switch (g_transport_state) {

    // --------------------------------------------------
    // IDLE：驱动二维码识别，检测到数据后转入处理阶段
    // --------------------------------------------------
    case TRANSPORT_IDLE: {
        int ret = QR_process();
        if (ret == 1 && g_qr_data_ready) {
            g_transport_state = TRANSPORT_PROCESS;
        }
        // ret==0 (跳帧/无帧) 或 ret==-1 (未识别) 均保持 IDLE
        break;
    }

    // --------------------------------------------------
    // PROCESS：结算处理，消费 g_qr_data 并生成下板指令包
    // --------------------------------------------------
    case TRANSPORT_PROCESS: {
        g_qr_data_ready = false;

        if (qr_settlement(g_qr_data, g_tx_packet, sizeof(g_tx_packet)) != 0) {
            // 解析失败，用原始数据兜底
            int len = (int)strlen(g_qr_data);
            if (len > (int)sizeof(g_tx_packet) - 4) len = sizeof(g_tx_packet) - 4;
            memcpy(g_tx_packet, "RAW:", 4);
            memcpy(g_tx_packet + 4, g_qr_data, len);
            g_tx_packet[4 + len]     = '\r';
            g_tx_packet[4 + len + 1] = '\n';
            g_tx_packet[4 + len + 2] = '\0';
        }

        g_send_retry = 0;
        g_transport_state = TRANSPORT_SEND;
        break;
    }

    // --------------------------------------------------
    // SEND：通过 UART1 将结算数据发送至下板
    // --------------------------------------------------
    case TRANSPORT_SEND: {
        int len = (int)strlen(g_tx_packet);
        int sent = uart1_send((const uint8 *)g_tx_packet, (uint32)len);

        if (sent == len) {
            // 发送成功，记录并进入 DONE 态，不再继续扫描
            strncpy(g_last_sent, g_tx_packet, sizeof(g_last_sent) - 1);
            g_last_sent[sizeof(g_last_sent) - 1] = '\0';
            g_transport_state = TRANSPORT_DONE;
        } else if (g_send_retry < 3) {
            // 发送未完成，下次 tick 重试（最多3次）
            g_send_retry++;
        } else {
            // 重试耗尽，丢弃本帧，进入 DONE 态
            g_last_sent[0] = '\0';
            g_transport_state = TRANSPORT_DONE;
        }
        break;
    }

    // --------------------------------------------------
    // DONE：已完成一轮传输，不再调用 QR_process()，等待外部复位
    // --------------------------------------------------
    case TRANSPORT_DONE:
    default:
        break;

    } // end switch
}

// ============================================================
// 调试接口
// ============================================================

void transport_reset(void)
{
    g_transport_state = TRANSPORT_IDLE;
    g_qr_data_ready   = false;
    g_tx_packet[0]    = '\0';
    g_send_retry      = 0;
}

int transport_get_state(void)
{
    return g_transport_state;
}

const char* transport_get_last_sent(void)
{
    return g_last_sent;
}
