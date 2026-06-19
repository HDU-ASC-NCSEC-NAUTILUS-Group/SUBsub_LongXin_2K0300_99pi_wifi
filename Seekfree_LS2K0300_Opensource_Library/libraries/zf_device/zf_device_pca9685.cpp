#include "zf_device_pca9685.h"
#include "zf_driver_i2c.h"
#include "zf_driver_delay.h"
#include <math.h>       // fminf, fmaxf

static int g_fd = -1;

// ==================== 写入节流保护 ====================

/* 每个通道最近一次成功写入的时刻（微秒），0 表示从未写入 */
static uint64_t g_chan_last_us[16];

/* I2C 总线最近一次写入时刻（微秒），跨通道共享 */
static uint64_t g_bus_last_us;

// ==================== 非阻塞状态机上下文 ====================

static enum ServoSMState g_sm_state = SM_UNINIT;
static uint64_t           g_sm_deadline_us;  // 延时到期时刻（微秒）
static const char      *g_sm_i2c_dev;
static uint8            g_sm_addr;
static uint16           g_sm_freq;
static uint8            g_sm_oldmode;       // 保存 MODE1 原始值

/* 获取当前单调时间（微秒） */
static inline uint64_t sm_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/* 设置延时 deadline = now + delay_us */
static inline void sm_set_deadline(uint32 delay_us)
{
    g_sm_deadline_us = sm_now_us() + delay_us;
}

/* 检查是否已到 deadline */
static inline bool sm_deadline_expired(void)
{
    return sm_now_us() >= g_sm_deadline_us;
}

int pca9685_init(const char *i2c_dev, uint8 addr)
{
    g_fd = i2c_open(i2c_dev);
    if (g_fd < 0) {
        return -1;
    }

    if (i2c_set_slave(g_fd, addr) < 0) {
        i2c_close(g_fd);
        g_fd = -1;
        return -1;
    }

    pca9685_reset(g_fd);

    i2c_write_byte(g_fd, PCA9685_MODE2, PCA9685_MODE2_OUTDRV);
    i2c_write_byte(g_fd, PCA9685_MODE1, PCA9685_MODE1_AI | 0x01);
    system_delay_ms(5);

    return g_fd;
}

void pca9685_close(int fd)
{
    if (fd >= 0) {
        pca9685_reset(fd);
        i2c_close(fd);
    }
    if (fd == g_fd) {
        g_fd = -1;
    }
}

void pca9685_reset(int fd)
{
    i2c_write_byte(fd, PCA9685_MODE1, 0x00);
}

void pca9685_set_pwm_freq(int fd, uint16 freq)
{
    if (freq == 0) return;

    uint8 oldmode;
    if (i2c_read_byte(fd, PCA9685_MODE1, &oldmode) < 0) return;

    uint8 newmode = (oldmode & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
    i2c_write_byte(fd, PCA9685_MODE1, newmode);

    float prescaleval = (float)PCA9685_OSC_CLOCK / ((float)PCA9685_PWM_RESOLUTION * (float)freq);
    int prescale = (int)(prescaleval + 0.5) - 1;
    if (prescale < 3) prescale = 3;
    if (prescale > 255) prescale = 255;

    i2c_write_byte(fd, PCA9685_PRE_SCALE, (uint8)prescale);
    i2c_write_byte(fd, PCA9685_MODE1, oldmode);
    system_delay_us(500);
    i2c_write_byte(fd, PCA9685_MODE1, oldmode | PCA9685_MODE1_RESTART);
}

void pca9685_set_pwm(int fd, uint8 channel, uint16 on, uint16 off)
{
    if (channel > 15) return;

    uint8 data[4];
    data[0] = on & 0xFF;
    data[1] = on >> 8;
    data[2] = off & 0xFF;
    data[3] = off >> 8;

    i2c_write_bytes(fd, PCA9685_LED0_ON_L + 4 * channel, data, 4);
}

void pca9685_set_all_pwm(int fd, uint16 on, uint16 off)
{
    uint8 data[4];
    data[0] = on & 0xFF;
    data[1] = on >> 8;
    data[2] = off & 0xFF;
    data[3] = off >> 8;

    i2c_write_bytes(fd, PCA9685_ALL_LED_ON_L, data, 4);
}

void pca9685_set_servo_pulse(int fd, uint8 channel, uint16 pulse_us)
{
    float period_us = 1000000.0f / (float)SERVO_FREQ;
    float us_per_count = period_us / (float)PCA9685_PWM_RESOLUTION;
    uint16 pulse = (uint16)((float)pulse_us / us_per_count);
    pca9685_set_pwm(fd, channel, 0, pulse);
}

int Servo_Init(const char *i2c_dev, uint8 addr)
{
    int fd = pca9685_init(i2c_dev, addr);
    if (fd < 0) return -1;

    pca9685_set_pwm_freq(fd, SERVO_FREQ);
    return fd;
}

int Servo_Set_Angle(uint8 channel, float angle)
{
    uint64_t now;

    if (g_fd < 0) return 0;
    if (channel > 15) return 0;

    now = sm_now_us();

    // 同通道冷却保护：高频调用只有第一次生效
    if (g_chan_last_us[channel] != 0
        && (now - g_chan_last_us[channel]) < (uint64_t)SERVO_CHANNEL_COOLDOWN_US) {
        return 0;
    }

    // I2C 总线冷却保护：跨通道也须间隔
    if (g_bus_last_us != 0
        && (now - g_bus_last_us) < (uint64_t)SERVO_BUS_COOLDOWN_US) {
        return 0;
    }

    // 钳位到有效范围
    angle = fmaxf((float)SERVO_ANGLE_MIN, fminf((float)SERVO_ANGLE_MAX, angle));

    uint32 pulse_us = (uint32)((float)SERVO_PULSE_MIN_US
        + ((float)SERVO_PULSE_MAX_US - (float)SERVO_PULSE_MIN_US) * angle
          / (float)SERVO_ANGLE_MAX);

    float period_us   = 1000000.0f / (float)SERVO_FREQ;
    float us_per_count = period_us / (float)PCA9685_PWM_RESOLUTION;
    uint16 pulse = (uint16)((float)pulse_us / us_per_count + 0.5f);

    pca9685_set_pwm(g_fd, channel, 0, pulse);

    // 写入成功，更新双级时间戳
    g_chan_last_us[channel] = sm_now_us();
    g_bus_last_us           = g_chan_last_us[channel];

    return 1;
}

void Stop_Servo(uint8 channel)
{
    pca9685_set_pwm(g_fd, channel, 0, 0);
}

void Stop_Servo_All(void)
{
    pca9685_set_all_pwm(g_fd, 0, 0);
}

// ==================== 非阻塞状态机实现 ====================

void Servo_Init_Async(const char *i2c_dev, uint8 addr, uint16 freq)
{
    g_sm_i2c_dev = i2c_dev;
    g_sm_addr    = addr;
    g_sm_freq    = freq;
    g_sm_state   = SM_INIT_RESET;
}

enum ServoSMState Servo_GetState(void)
{
    return g_sm_state;
}

/*
 * 状态机驱动函数，需在主循环中周期性调用
 *
 * 状态流转:
 *   SM_INIT_RESET  → 打开 I2C，写复位
 *   SM_INIT_MODE2  → 写 MODE2
 *   SM_INIT_MODE1  → 写 MODE1(AI|SLEEP)，设 5ms 延时
 *   SM_INIT_DELAY  → 等待 5ms 到期
 *   SM_FREQ_READ   → 读 MODE1 存 oldmode
 *   SM_FREQ_SLEEP  → 写 MODE1(oldmode|SLEEP)
 *   SM_FREQ_PRESCALE → 计算并写 prescale
 *   SM_FREQ_WAKE   → 写 MODE1(oldmode)，设 500μs 延时
 *   SM_FREQ_DELAY  → 等待 500μs 到期
 *   SM_FREQ_RESTART → 写 MODE1(oldmode|RESTART)
 *   SM_READY       → 就绪
 *
 * 每步失败均跳转 SM_ERROR
 */
void Servo_Tick(void)
{
    float prescaleval;
    int prescale;

    switch (g_sm_state) {

    // --- 复位阶段 ---
    case SM_INIT_RESET:   // 打开 I2C，设置从机地址，写 MODE1=0x00
        g_fd = i2c_open(g_sm_i2c_dev);
        if (g_fd < 0) { g_sm_state = SM_ERROR; return; }
        if (i2c_set_slave(g_fd, g_sm_addr) < 0) {
            i2c_close(g_fd); g_fd = -1;
            g_sm_state = SM_ERROR; return;
        }
        pca9685_reset(g_fd);
        g_sm_state = SM_INIT_MODE2;
        break;

    case SM_INIT_MODE2:   // 写 MODE2=OUTDRV（推挽输出）
        if (i2c_write_byte(g_fd, PCA9685_MODE2, PCA9685_MODE2_OUTDRV) < 0) {
            g_sm_state = SM_ERROR; return;
        }
        g_sm_state = SM_INIT_MODE1;
        break;

    case SM_INIT_MODE1:   // 写 MODE1=AI|SLEEP（自动递增+休眠），然后等待 5ms 振荡器稳定
        if (i2c_write_byte(g_fd, PCA9685_MODE1, PCA9685_MODE1_AI | 0x01) < 0) {
            g_sm_state = SM_ERROR; return;
        }
        sm_set_deadline(5000);
        g_sm_state = SM_INIT_DELAY;
        break;

    case SM_INIT_DELAY:   // 等待 5ms 到期
        if (!sm_deadline_expired()) return;
        g_sm_state = SM_FREQ_READ;
        break;

    // --- 频率设置阶段 ---
    case SM_FREQ_READ:    // 读取当前 MODE1，保存到 g_sm_oldmode
        if (i2c_read_byte(g_fd, PCA9685_MODE1, &g_sm_oldmode) < 0) {
            g_sm_state = SM_ERROR; return;
        }
        g_sm_state = SM_FREQ_SLEEP;
        break;

    case SM_FREQ_SLEEP: { // 写 MODE1=SLEEP（必须先休眠才能改 prescale）
        uint8 newmode = (g_sm_oldmode & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
        if (i2c_write_byte(g_fd, PCA9685_MODE1, newmode) < 0) {
            g_sm_state = SM_ERROR; return;
        }
        g_sm_state = SM_FREQ_PRESCALE;
        break;
    }

    case SM_FREQ_PRESCALE: // 计算 prescale 并写入 PRE_SCALE 寄存器
        prescaleval = (float)PCA9685_OSC_CLOCK
                    / ((float)PCA9685_PWM_RESOLUTION * (float)g_sm_freq);
        prescale = (int)(prescaleval + 0.5f) - 1;
        if (prescale < 3)  prescale = 3;
        if (prescale > 255) prescale = 255;
        if (i2c_write_byte(g_fd, PCA9685_PRE_SCALE, (uint8)prescale) < 0) {
            g_sm_state = SM_ERROR; return;
        }
        g_sm_state = SM_FREQ_WAKE;
        break;

    case SM_FREQ_WAKE:    // 写 MODE1=oldmode（退出休眠），然后等待 500μs 振荡器重新起振
        if (i2c_write_byte(g_fd, PCA9685_MODE1, g_sm_oldmode) < 0) {
            g_sm_state = SM_ERROR; return;
        }
        sm_set_deadline(500);
        g_sm_state = SM_FREQ_DELAY;
        break;

    case SM_FREQ_DELAY:   // 等待 500μs 到期
        if (!sm_deadline_expired()) return;
        g_sm_state = SM_FREQ_RESTART;
        break;

    case SM_FREQ_RESTART: // 写 MODE1=oldmode|RESTART（恢复 PWM 输出）
        if (i2c_write_byte(g_fd, PCA9685_MODE1, g_sm_oldmode | PCA9685_MODE1_RESTART) < 0) {
            g_sm_state = SM_ERROR; return;
        }
        g_sm_state = SM_READY;
        break;

    // --- 终态 ---
    case SM_READY:        // 初始化完成，舵机可控
    case SM_ERROR:        // 初始化失败
    case SM_UNINIT:       // 尚未启动
    default:
        break;
    }
}


