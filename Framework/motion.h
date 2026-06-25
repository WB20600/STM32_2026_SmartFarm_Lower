#ifndef __MOTION_H
#define __MOTION_H

#include "stm32f4xx.h"

extern volatile uint8_t g_motion_en;

void Motion_Init(void);
void Motion_OnSpd(const uint8_t *d);
void Motion_OnStop(const uint8_t *d);
void Motion_OnMode(const uint8_t *d);
void Motion_OnPid(const uint8_t *d);
void Motion_SpeedLoop(void);
void Motion_GetOdo(int16_t *abcd);

#endif
