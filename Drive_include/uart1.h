#ifndef __UART1_H
#define __UART1_H

#include "stm32f4xx.h"

void UART1_Init(uint32_t baud);
void uart1_send_byte(uint8_t dat);
void uart1_send_str(const uint8_t *s);

#endif
