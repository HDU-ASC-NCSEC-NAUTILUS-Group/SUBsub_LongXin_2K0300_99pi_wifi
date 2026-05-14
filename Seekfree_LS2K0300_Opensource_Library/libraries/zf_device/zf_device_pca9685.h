#ifndef _zf_device_pca9685_h
#define _zf_device_pca9685_h

#include "zf_common_typedef.h"

#define PCA9685_ADDR_DEFAULT     0x40

#define PCA9685_MODE1            0x00
#define PCA9685_MODE2            0x01
#define PCA9685_LED0_ON_L        0x06
#define PCA9685_ALL_LED_ON_L     0xFA
#define PCA9685_ALL_LED_OFF_L    0xFC
#define PCA9685_PRE_SCALE        0xFE

#define PCA9685_MODE1_SLEEP      0x10
#define PCA9685_MODE1_AI         0x20
#define PCA9685_MODE1_RESTART    0x80
#define PCA9685_MODE2_OUTDRV     0x04

#define PCA9685_OSC_CLOCK        25000000
#define PCA9685_PWM_RESOLUTION   4096

#define SERVO_FREQ               50
#define SERVO_ANGLE_MIN          0
#define SERVO_ANGLE_MAX          180

/*
 * 舵机脉冲范围校准（不同批次 MG90S 实测偏差可达 100-200μs）
 *
 * 校准方法:
 *   将 MIN 改大后调用 Set_Servo(0, 0)，观察是否到 0°
 *   将 MAX 改小后调用 Set_Servo(0, 180)，观察是否到 180°
 *   反复微调直到两个端点都恰好到位，建议每次增减 50μs
 *
 * 常见参考: 500~2500, 600~2400, 700~2300
 * 改中间值即可，角度比例映射自动生效，90°恒为 (MIN+MAX)/2
 */
#define SERVO_PULSE_MIN_US       300
#define SERVO_PULSE_MAX_US       2700

int  pca9685_init(const char *i2c_dev, uint8 addr);
void pca9685_close(int fd);
void pca9685_reset(int fd);
void pca9685_set_pwm_freq(int fd, uint16 freq);
void pca9685_set_pwm(int fd, uint8 channel, uint16 on, uint16 off);
void pca9685_set_all_pwm(int fd, uint16 on, uint16 off);
void pca9685_set_servo_pulse(int fd, uint8 channel, uint16 pulse_us);

/*
 * 简化舵机接口（无 fd 参数，内部自动管理）
 * Servo_Init 打开 I2C 并初始化 PCA9685，后续调用直接写 channel+angle 即可
 *   例: Servo_Init("/dev/i2c-4", 0x40);
 *       Set_Servo(0, 90);
 *       Stop_Servo(3);
 *       Stop_Servo_All();
 * 多模块级联时用底层 pca9685_* 函数（带 fd 参数）
 */
int  Servo_Init(const char *i2c_dev, uint8 addr);
void Set_Servo(uint8 channel, uint16 angle);
void Stop_Servo(uint8 channel);
void Stop_Servo_All(void);

#endif
