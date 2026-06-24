#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include "stm32f4xx.h"

void Scheduler_Init(void);

uint8_t Scheduler_TakeProtocolFlag(void);
uint8_t Scheduler_TakeOdoFlag(void);
uint8_t Scheduler_TakeHbFlag(void);
uint8_t Scheduler_TakeBatFlag(void);
uint8_t Scheduler_TakeActuatorFlag(void);

#endif
