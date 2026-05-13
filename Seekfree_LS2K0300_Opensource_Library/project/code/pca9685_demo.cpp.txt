/*********************************************************************************************************************
* PCA9685 舵机控制 Demo
* 简化接口: Servo_Init / Set_Servo
* 硬件: PCA9685 SDA->P23 SCL->P22, MG90S 舵机 0-180°
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "zf_device_pca9685.h"

static int servo_fd = -1;

void pca9685_demo(void)
{
    printf("PCA9685 初始化...\n");

    servo_fd = Servo_Init("/dev/i2c-4", PCA9685_ADDR_DEFAULT);
    if (servo_fd < 0) {
        printf("初始化失败! 检查 PCA9685 电源和接线\n");
        return;
    }
    printf("PCA9685 初始化成功\n");

    printf("舵机控制命令正在发出\n");
    // Set_Servo(servo_fd, 0, 180);
    Stop_Servo(servo_fd, 0);
    // Set_Servo(servo_fd, 1, 60);
    Stop_Servo(servo_fd, 1);
    // Set_Servo(servo_fd, 2, 45);
    Set_Servo(servo_fd, 3, 100);

    printf("完成\n");
}
