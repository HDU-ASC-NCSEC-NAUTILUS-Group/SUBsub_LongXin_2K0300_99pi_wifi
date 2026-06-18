#ifndef _zf_device_pca9685_h
#define _zf_device_pca9685_h

#include "zf_common_typedef.h"
#include <time.h>

#define PCA9685_ADDR_DEFAULT        0x40

#define PCA9685_MODE1               0x00
#define PCA9685_MODE2               0x01
#define PCA9685_LED0_ON_L           0x06
#define PCA9685_ALL_LED_ON_L        0xFA
#define PCA9685_PRE_SCALE           0xFE

#define PCA9685_MODE1_SLEEP         0x10
#define PCA9685_MODE1_AI            0x20
#define PCA9685_MODE1_RESTART       0x80
#define PCA9685_MODE2_OUTDRV        0x04

#define PCA9685_OSC_CLOCK           25000000
#define PCA9685_PWM_RESOLUTION      4096

#define SERVO_FREQ                  50
#define SERVO_ANGLE_MIN             0
#define SERVO_ANGLE_MAX             180

/*
 * 舵机脉冲范围校准（不同批次 MG90S 实测偏差可达 100-200μs）
 *
 * 校准方法:
 *   将 MIN 改大后调用 Servo_Set_Angle(0, 0.0f)，观察是否到 0°
 *   将 MAX 改小后调用 Servo_Set_Angle(0, 180.0f)，观察是否到 180°
 *   反复微调直到两个端点都恰好到位，建议每次增减 50μs
 *
 * 常见参考: 500~2500, 600~2400, 700~2300
 * 改中间值即可，角度比例映射自动生效，90°恒为 (MIN+MAX)/2
 */
#define SERVO_PULSE_MIN_US          400
#define SERVO_PULSE_MAX_US          2600

/*
 * 同一通道连续写入保护（微秒）
 *
 * 高频调用时只有第一次生效，冷却期内后续调用被忽略且返回 0
 * 所有通道共用一条 I2C 总线，两条不同通道命令之间有独立的总线冷却期
 */
#define SERVO_CHANNEL_COOLDOWN_US   5000   // 同通道冷却 5ms
#define SERVO_BUS_COOLDOWN_US       2000   // I2C 总线冷却 2ms

// ==================== 非阻塞状态机 ====================

/*
 * 状态机状态枚举
 *
 * 调用 Servo_Init_Async 后，需在主循环中周期性调用 Servo_Tick
 * 状态机自动推进 init → set_freq → ready 的全部非阻塞流程
 * SM_READY 后 Servo_Set_Angle 方可生效
 */

enum ServoSMState {
    SM_UNINIT = 0,       // 未初始化
    SM_INIT_RESET,       // 写 MODE1=0x00（复位）
    SM_INIT_MODE2,       // 写 MODE2=OUTDRV
    SM_INIT_MODE1,       // 写 MODE1=AI|SLEEP
    SM_INIT_DELAY,       // 等待 5ms（芯片唤醒稳定）
    SM_FREQ_READ,        // 读 MODE1 备份 oldmode
    SM_FREQ_SLEEP,       // 写 MODE1=oldmode|SLEEP（进入休眠以设置 prescale）
    SM_FREQ_PRESCALE,    // 写 PRE_SCALE
    SM_FREQ_WAKE,        // 写 MODE1=oldmode（退出休眠）
    SM_FREQ_DELAY,       // 等待 500μs（振荡器稳定）
    SM_FREQ_RESTART,     // 写 MODE1=oldmode|RESTART
    SM_READY,            // 就绪，可正常控制舵机
    SM_ERROR,            // 初始化失败
};

int  pca9685_init(const char *i2c_dev, uint8 addr);
void pca9685_close(int fd);
void pca9685_reset(int fd);
void pca9685_set_pwm_freq(int fd, uint16 freq);
void pca9685_set_pwm(int fd, uint8 channel, uint16 on, uint16 off);
void pca9685_set_all_pwm(int fd, uint16 on, uint16 off);
void pca9685_set_servo_pulse(int fd, uint8 channel, uint16 pulse_us);

/* ==================== 初始化（二选一）==================== */

/*
 * [方式一] 阻塞初始化，适合启动阶段一次性完成
 *
 *   例:
 *     if (Servo_Init("/dev/i2c-4", 0x40) < 0) {
 *         printf("failed:PCA9685多路舵机驱动初始化失败\n");
 *     }
 *     Stop_Servo_All();
 */
int Servo_Init(const char *i2c_dev, uint8 addr);

/*
 * [方式二] 非阻塞异步初始化，适合不能阻塞主循环的场景
 *
 *   例:
 *     Servo_Init_Async("/dev/i2c-4", 0x40, SERVO_FREQ);
 *     while (1) {
 *         Servo_Tick();
 *         if (Servo_GetState() == SM_READY) {
 *             // 初始化就绪，可开始控制舵机
 *         }
 *         // 其他业务逻辑继续运行不被阻塞 ...
 *     }
 */
void Servo_Init_Async(const char *i2c_dev, uint8 addr, uint16 freq);
void Servo_Tick(void);
enum ServoSMState Servo_GetState(void);

/* ==================== 舵机角度控制 ==================== */

/*
 * 设置舵机角度（两种初始化方式通用）
 *
 * 返回值: 1=指令已通过 I2C 下发, 0=冷却期内被忽略/未初始化/参数错误
 *
 *   例:
 *     // 标准调用：检查返回值，确认 I2C 是否真正下发
 *     if (Servo_Set_Angle(0, 90.0f)) {  }
 *
 *     // 简写：不在意是否被冷却期跳过
 *     Servo_Set_Angle(1, 45.5f);
 *
 *     // 紧急停止特定通道 / 全部通道
 *     Stop_Servo(3);
 *     Stop_Servo_All();
 */
int  Servo_Set_Angle(uint8 channel, float angle);
void Stop_Servo(uint8 channel);
void Stop_Servo_All(void);

#endif
