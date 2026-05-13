#include "zf_driver_i2c.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

int i2c_open(const char *path)
{
    int fd = open(path, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "i2c_open: failed to open %s: %s\n", path, strerror(errno));
    }
    return fd;
}

void i2c_close(int fd)
{
    if (fd >= 0) {
        close(fd);
    }
}

int i2c_set_slave(int fd, uint8 addr)
{
    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        fprintf(stderr, "i2c_set_slave: failed to set addr 0x%02x: %s\n", addr, strerror(errno));
        return -1;
    }
    return 0;
}

int i2c_write_byte(int fd, uint8 reg, uint8 data)
{
    uint8 buf[2] = { reg, data };
    if (write(fd, buf, 2) != 2) {
        fprintf(stderr, "i2c_write_byte: write reg 0x%02x failed: %s\n", reg, strerror(errno));
        return -1;
    }
    return 0;
}

int i2c_read_byte(int fd, uint8 reg, uint8 *data)
{
    if (data == NULL) return -1;

    if (write(fd, &reg, 1) != 1) {
        fprintf(stderr, "i2c_read_byte: write reg 0x%02x failed: %s\n", reg, strerror(errno));
        return -1;
    }
    if (read(fd, data, 1) != 1) {
        fprintf(stderr, "i2c_read_byte: read reg 0x%02x failed: %s\n", reg, strerror(errno));
        return -1;
    }
    return 0;
}

int i2c_write_bytes(int fd, uint8 reg, const uint8 *data, uint32 len)
{
    if (data == NULL || len == 0) return -1;

    uint8 *buf = (uint8 *)malloc(len + 1);
    if (buf == NULL) {
        fprintf(stderr, "i2c_write_bytes: malloc failed\n");
        return -1;
    }

    buf[0] = reg;
    memcpy(buf + 1, data, len);

    int ret = 0;
    if (write(fd, buf, len + 1) != (ssize_t)(len + 1)) {
        fprintf(stderr, "i2c_write_bytes: write %u bytes to reg 0x%02x failed: %s\n",
                len, reg, strerror(errno));
        ret = -1;
    }

    free(buf);
    return ret;
}

int i2c_read_bytes(int fd, uint8 reg, uint8 *data, uint32 len)
{
    if (data == NULL || len == 0) return -1;

    if (write(fd, &reg, 1) != 1) {
        fprintf(stderr, "i2c_read_bytes: write reg 0x%02x failed: %s\n", reg, strerror(errno));
        return -1;
    }
    if (read(fd, data, len) != (ssize_t)len) {
        fprintf(stderr, "i2c_read_bytes: read %u bytes failed: %s\n", len, strerror(errno));
        return -1;
    }
    return 0;
}
