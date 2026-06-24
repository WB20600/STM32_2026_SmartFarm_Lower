#ifndef __UART6_LINK_H
#define __UART6_LINK_H

#include "stm32f4xx.h"

void Uart6_Link_Init(uint32_t baud);
void Uart6_SendByte(uint8_t b);
void Uart6_SendBuf(const uint8_t *p, uint16_t n);

#endif
