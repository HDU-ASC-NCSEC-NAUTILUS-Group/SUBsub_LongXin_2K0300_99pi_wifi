/*********************************************************************************************************************
* I2C 通用接口示例
* 使用前请确认 /dev/i2c-* 设备节点存在
* 编译：在 project/user 目录运行 ./build.sh 即可
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "zf_driver_i2c.h"
#include <linux/i2c-dev.h>

/*
 * I2C 总线扫描
 * 遍历 0x03-0x77 地址范围，打印所有应答的设备地址
 */
static void i2c_scan(const char *i2c_dev)
{
    int fd = i2c_open(i2c_dev);
    if (fd < 0) return;

    printf("I2C bus scan on %s:\n", i2c_dev);
    printf("   Found devices at: ");

    int found = 0;
    for (int addr = 0x03; addr <= 0x77; addr++) {
        // 尝试设置地址并发送一个空写检测设备是否存在
        if (ioctl(fd, I2C_SLAVE, addr) < 0) continue;

        char dummy = 0;
        if (write(fd, &dummy, 0) >= 0) {
            printf("0x%02X ", addr);
            found++;
        }
    }

    if (found == 0) {
        printf("(none)");
    }
    printf("\n   Total: %d device(s)\n", found);

    i2c_close(fd);
}

/*
 * 示例：读取设备 WHO_AM_I 寄存器
 * 许多 I2C 传感器的寄存器 0x00 或 0x0F 是 ID 寄存器
 */
void i2c_read_device_id(const char *i2c_dev, uint8 slave_addr, uint8 id_reg)
{
    int fd = i2c_open(i2c_dev);
    if (fd < 0) return;

    if (i2c_set_slave(fd, slave_addr) < 0) {
        i2c_close(fd);
        return;
    }

    uint8 id;
    if (i2c_read_byte(fd, id_reg, &id) == 0) {
        printf("Device 0x%02X ID register 0x%02X = 0x%02X\n", slave_addr, id_reg, id);
    } else {
        printf("Device 0x%02X: failed to read ID register\n", slave_addr);
    }

    i2c_close(fd);
}

void i2c_demo(void)
{
    printf("=== LS2K0300 I2C Demo ===\n\n");

    // --------------------------------------------------------------------
    // 步骤1: 确定 I2C 设备节点
    // --------------------------------------------------------------------
    // i2c-gpio 设备树节点会在系统中注册为 /dev/i2c-N
    // 通常 i2c0/i2c1 是硬件I2C，i2c-gpio 会分配到下一个可用编号
    // 请根据实际 dmesg 确认，此处以 /dev/i2c-2 为例
    // --------------------------------------------------------------------
    const char *i2c_dev = "/dev/i2c-2";

    // --------------------------------------------------------------------
    // 步骤2: 扫描 I2C 总线
    // --------------------------------------------------------------------
    i2c_scan(i2c_dev);

    // --------------------------------------------------------------------
    // 步骤3: 读写设备示例
    // --------------------------------------------------------------------
    // 假设连接了一个 SSD1306 OLED (地址 0x3C)
    // 取消下面的注释并修改地址后使用
    // --------------------------------------------------------------------
#if 0
    int fd = i2c_open(i2c_dev);
    if (fd >= 0) {
        i2c_set_slave(fd, 0x3C);  // 设置从设备地址

        // 写单字节: 向寄存器 0x00 写入 0xAE (display off)
        i2c_write_byte(fd, 0x00, 0xAE);

        // 写多字节: 向寄存器 0x00 连续写入初始化序列
        uint8 init_seq[] = {0xAE, 0xD5, 0x80, 0xA8, 0x3F};
        i2c_write_bytes(fd, 0x00, init_seq, sizeof(init_seq));

        // 读单字节: 读取状态寄存器
        uint8 status;
        i2c_read_byte(fd, 0x00, &status);

        // 读多字节: 从寄存器 0x00 读取 4 字节
        uint8 buf[4];
        i2c_read_bytes(fd, 0x00, buf, 4);

        i2c_close(fd);
    }
#endif

    // --------------------------------------------------------------------
    // 步骤4: 读取设备 ID 示例
    // --------------------------------------------------------------------
    // 如果连接了设备，取消注释调用：
    // i2c_read_device_id(i2c_dev, 0x68, 0x75);  // MPU6050 WHO_AM_I
    // i2c_read_device_id(i2c_dev, 0x3C, 0x00);  // SSD1306 status

    printf("\n=== Demo finished ===\n");
}
