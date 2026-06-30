#ifndef __MOTION_H
#define __MOTION_H

#include "stm32f4xx.h"

#define MOTION_SPEED_LOOP_PERIOD_MS  5U
#define MOTION_SPEED_LOOP_DT_S       0.005f

extern volatile uint8_t g_motion_en;
extern volatile uint32_t g_pid_start_cycle;
extern volatile uint32_t g_pid_end_cycle;
extern volatile uint32_t g_pid_elapsed_cycle;
extern volatile uint32_t g_pid_max_cycle;
extern volatile uint32_t g_pid_measure_count;

void Motion_Init(void);
void Motion_OnSpd(const uint8_t *d);
void Motion_OnStop(const uint8_t *d);
void Motion_OnMode(const uint8_t *d);
void Motion_OnPid(const uint8_t *d);
void Motion_SpeedLoop(void);
void Motion_GetOdo(int16_t *abcd);

#endif
