#ifndef _UART_H_
#define _UART_H_

#include "xprintf/xprintf.h"

#define SER_BUF_SIZE 10

void send_uart (const char *buf, int len);
int recv_uart (char *buf, int len);
void init_uart (int baud);

#endif
