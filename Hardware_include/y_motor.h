#ifndef __Y_MOTOR_H
#define __Y_MOTOR_H

#include "stm32f4xx.h"
#include "Sys.h"
#include <stdlib.h>

typedef struct
{
    float SumError;
    float Kp;
    float Ki;
    float Kd;
    float LastError;
    float LLastError;
    float iIncpid;
    float errormax;
    float errormin;
    float Imax;
} PID;

extern PID A_SpeedPID;
extern PID B_SpeedPID;
extern PID C_SpeedPID;
extern PID D_SpeedPID;

extern int Z_step;
extern int Y_step;

float IncPIDCalc(PID *sptr, float Setpoint, float Turepoint);
void PID_int(void);

void MOTOR_A_SetSpeed(int16_t speed);
void MOTOR_B_SetSpeed(int16_t speed);
void MOTOR_C_SetSpeed(int16_t speed);
void MOTOR_D_SetSpeed(int16_t speed);

void MOTOR_A_SetSpeed_Close(float speed, float SpeedA);
void MOTOR_B_SetSpeed_Close(float speed, float SpeedB);
void MOTOR_C_SetSpeed_Close(float speed, float SpeedC);
void MOTOR_D_SetSpeed_Close(float speed, float SpeedD);

void Stepper_Motor_Run_1(int32_t num);
void Stepper_Motor_Run_2(int32_t num);
void Stepper_ON(void);
void Stepper_OFF(void);

#endif
