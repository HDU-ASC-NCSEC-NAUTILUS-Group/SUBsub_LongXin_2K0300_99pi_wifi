#ifndef _zf_driver_uart1_h
#define _zf_driver_uart1_h

#include "zf_common_typedef.h"

int  uart1_init(const char *device, uint32 baudrate);
void uart1_deinit(void);
int  uart1_send(const uint8 *data, uint32 len);
int  uart1_recv(uint8 *buf, uint32 maxlen);
int  uart1_available(void);
int  uart1_printf(const char *fmt, ...);

#endif
