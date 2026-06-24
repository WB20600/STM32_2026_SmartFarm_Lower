#ifndef __BEEP_H
#define __BEEP_H

#include "stm32f4xx.h"
#include "Sys.h"

void BEEP_Init(void);
void beep_on_times(int times, int delay_ms);

#define BEEP_Off()    GPIO_ResetBits(GPIOC, GPIO_Pin_13)
#define BEEP_On()     GPIO_SetBits(GPIOC, GPIO_Pin_13)
#define BEEP_Toggle() GPIO_WriteBit(GPIOC, GPIO_Pin_13, (BitAction)(1 - GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13)))

#endif
