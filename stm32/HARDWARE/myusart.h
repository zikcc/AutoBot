#ifndef __MYUSART_H
#define __MYUSART_H

#include "main.h"

void usart_test(void);
void UART1_CallBack(void);
void Uart1RxMonitor(uint8_t ms); //接收监控
void uart1driver(void);
void uart1cmddriver(void);

#endif
