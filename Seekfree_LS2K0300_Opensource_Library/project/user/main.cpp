
#include "zf_common_headfile.h"

// PCA9685 demo (定义在 ../code/pca9685_demo.cpp)
extern void pca9685_demo(void);

int main(int, char**)
{
    // 运行 PCA9685 PWM 驱动测试 (参照 Arduino Adafruit_PWMServoDriver)
    pca9685_demo();

    while(1)
    {
        system_delay_ms(100);
    }
}
