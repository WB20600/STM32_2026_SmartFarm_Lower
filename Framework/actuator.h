#ifndef __ACTUATOR_H
#define __ACTUATOR_H

#include "stm32f4xx.h"

typedef struct
{
    int16_t arm_x;
    int16_t arm_y;
    int16_t arm_z;
    int16_t arm_r;
    uint8_t grip_on;
    uint8_t fire_count;
    int16_t lift_target;
    uint8_t storage_id;
} ActuatorState;

extern volatile uint8_t g_actuator_en;

void Actuator_Init(void);
void Actuator_OnArm(const uint8_t *d);
void Actuator_OnGrip(const uint8_t *d);
void Actuator_OnFire(const uint8_t *d);
void Actuator_OnLift(const uint8_t *d);
void Actuator_OnStorage(const uint8_t *d);
void Actuator_Tick(void);
void Actuator_GetState(ActuatorState *state);

#endif
