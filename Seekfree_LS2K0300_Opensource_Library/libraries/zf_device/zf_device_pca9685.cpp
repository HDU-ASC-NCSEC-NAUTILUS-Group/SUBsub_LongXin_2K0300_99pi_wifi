#include "zf_device_pca9685.h"
#include "zf_driver_i2c.h"
#include "zf_driver_delay.h"

static int g_fd = -1;

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

void Set_Servo(uint8 channel, uint16 angle)
{
    uint32 pulse_us = SERVO_PULSE_MIN_US
        + (uint32)(SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) * angle / SERVO_ANGLE_MAX;

    float period_us = 1000000.0f / (float)SERVO_FREQ;
    float us_per_count = period_us / (float)PCA9685_PWM_RESOLUTION;
    uint16 pulse = (uint16)((float)pulse_us / us_per_count);

    pca9685_set_pwm(g_fd, channel, 0, pulse);
}

void Stop_Servo(uint8 channel)
{
    pca9685_set_pwm(g_fd, channel, 0, 0);
}

void Stop_Servo_All(void)
{
    pca9685_set_all_pwm(g_fd, 0, 0);
}
