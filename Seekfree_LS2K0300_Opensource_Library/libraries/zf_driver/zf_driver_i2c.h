#ifndef _zf_driver_i2c_h
#define _zf_driver_i2c_h

#include "zf_common_typedef.h"

int     i2c_open            (const char *path);
void    i2c_close           (int fd);
int     i2c_set_slave       (int fd, uint8 addr);
int     i2c_write_byte      (int fd, uint8 reg, uint8 data);
int     i2c_read_byte       (int fd, uint8 reg, uint8 *data);
int     i2c_write_bytes     (int fd, uint8 reg, const uint8 *data, uint32 len);
int     i2c_read_bytes      (int fd, uint8 reg, uint8 *data, uint32 len);

#endif
