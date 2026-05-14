#include "zf_driver_uart1.h"
#include <stdio.h>
#include <stdarg.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <poll.h>

static int  g_uart1_fd = -1;
static char g_uart1_txbuf[256];

int uart1_init(const char *device, uint32 baudrate)
{
    g_uart1_fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (g_uart1_fd < 0) {
        fprintf(stderr, "uart1_init: open %s failed: %s\n", device, strerror(errno));
        return -1;
    }

    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    tcgetattr(g_uart1_fd, &tio);

    speed_t speed = B115200;
    if (baudrate >= 921600)      speed = B921600;
    else if (baudrate >= 460800) speed = B460800;
    else if (baudrate >= 230400) speed = B230400;
    else if (baudrate >= 115200) speed = B115200;
    else if (baudrate >= 57600)  speed = B57600;
    else if (baudrate >= 38400)  speed = B38400;
    else if (baudrate >= 19200)  speed = B19200;
    else                         speed = B9600;

    cfsetospeed(&tio, speed);
    cfsetispeed(&tio, speed);

    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |=  CS8;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag |=  CREAD | CLOCAL;

    tio.c_lflag = 0;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_oflag = 0;

    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    tcsetattr(g_uart1_fd, TCSANOW, &tio);
    tcflush(g_uart1_fd, TCIOFLUSH);

    return 0;
}

void uart1_deinit(void)
{
    if (g_uart1_fd >= 0) {
        close(g_uart1_fd);
        g_uart1_fd = -1;
    }
}

int uart1_send(const uint8 *data, uint32 len)
{
    if (g_uart1_fd < 0 || data == NULL || len == 0) return -1;

    int total = 0;
    while (total < (int)len) {
        int n = write(g_uart1_fd, data + total, len - total);
        if (n < 0) {
            if (errno == EAGAIN) continue;
            return -1;
        }
        total += n;
    }
    return total;
}

int uart1_recv(uint8 *buf, uint32 maxlen)
{
    if (g_uart1_fd < 0 || buf == NULL || maxlen == 0) return -1;

    int n = read(g_uart1_fd, buf, maxlen);
    if (n < 0) {
        if (errno == EAGAIN) return 0;
        return -1;
    }
    return n;
}

int uart1_available(void)
{
    if (g_uart1_fd < 0) return 0;

    struct pollfd pfd;
    pfd.fd     = g_uart1_fd;
    pfd.events = POLLIN;
    return (poll(&pfd, 1, 0) > 0) && (pfd.revents & POLLIN);
}

int uart1_printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(g_uart1_txbuf, sizeof(g_uart1_txbuf), fmt, args);
    va_end(args);

    if (len <= 0) return -1;
    return uart1_send((const uint8*)g_uart1_txbuf, (uint32)len);
}
