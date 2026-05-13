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
#define SERVO_PULSE_MIN_US       500
#define SERVO_PULSE_MAX_US       2500

int  pca9685_init(const char *i2c_dev, uint8 addr);
void pca9685_close(int fd);
void pca9685_reset(int fd);
void pca9685_set_pwm_freq(int fd, uint16 freq);
void pca9685_set_pwm(int fd, uint8 channel, uint16 on, uint16 off);
void pca9685_set_all_pwm(int fd, uint16 on, uint16 off);
void pca9685_set_servo_pulse(int fd, uint8 channel, uint16 pulse_us);

/*
 * 简化舵机接口
 * fd 是 open("/dev/i2c-*") 返回的文件描述符，不会自动变化
 * 每次调用显式传入 fd 是为了支持多模块级联：
 *   int fd1 = Servo_Init("/dev/i2c-4", 0x40);  // 模块1
 *   int fd2 = Servo_Init("/dev/i2c-4", 0x41);  // 模块2 (不同地址)
 *   Set_Servo(fd1, 0, 90);   Set_Servo(fd2, 0, 45);
 * 只接一个模块时，保存好 fd 一直复用即可
 */
int  Servo_Init(const char *i2c_dev, uint8 addr);
void Set_Servo(int fd, uint8 channel, uint16 angle);
void Stop_Servo(int fd, uint8 channel);
void Stop_Servo_All(int fd);

#endif
