#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "stm32f4xx.h"

#define FRAME_LEN  12
#define FRAME_H1   0xAA
#define FRAME_H2   0x55
#define FRAME_DATA 8

#define CMD_SPD   0x01
#define CMD_STOP  0x02
#define CMD_MODE  0x03
#define CMD_PING  0x04
#define CMD_PID   0x05

#define CMD_ARM   0x10
#define CMD_GRP   0x11
#define CMD_FIRE  0x12
#define CMD_LFT   0x13
#define CMD_STO   0x14

#define CMD_HB    0x81
#define CMD_ODO   0x82
#define CMD_BAT   0x83
#define CMD_SEN   0x84

extern volatile uint8_t  g_frame_ready;
extern volatile uint8_t  g_frame[FRAME_LEN];
extern volatile uint32_t g_proto_err;

void Protocol_Poll(void);
void Frame_Send(uint8_t cmd, const uint8_t *d8);

int16_t be16(const uint8_t *p);
void wr_be16(uint8_t *p, int16_t v);

void Telemetry_SendHB(void);
void Telemetry_SendOdo(void);
void Telemetry_SendBat(void);
void Sys_OnPing(const uint8_t *d);

#endif
