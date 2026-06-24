#include "protocol.h"
#include "uart6_link.h"
#include "motion.h"
#include "actuator.h"
#include "Sys.h"

volatile uint8_t  g_frame_ready = 0;
volatile uint8_t  g_frame[FRAME_LEN];
volatile uint32_t g_proto_err = 0;

extern uint16_t VIN_GetVol_X100(void);

#define FW_VER 2

typedef void (*CmdHandler)(const uint8_t *data);
typedef struct { uint8_t cmd; CmdHandler fn; volatile uint8_t *en; } CmdEntry;

static const CmdEntry CMD_TABLE[] = {
    { CMD_SPD,  Motion_OnSpd,      &g_motion_en   },
    { CMD_STOP, Motion_OnStop,     0              },
    { CMD_MODE, Motion_OnMode,     &g_motion_en   },
    { CMD_PING, Sys_OnPing,        0              },
    { CMD_ARM,  Actuator_OnArm,    &g_actuator_en },
    { CMD_GRP,  Actuator_OnGrip,   &g_actuator_en },
    { CMD_FIRE, Actuator_OnFire,   &g_actuator_en },
    { CMD_LFT,  Actuator_OnLift,   &g_actuator_en },
    { CMD_STO,  Actuator_OnStorage,&g_actuator_en },
};

int16_t be16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] << 8 | p[1]);
}

void wr_be16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)((uint16_t)v >> 8);
    p[1] = (uint8_t)v;
}

void Frame_Send(uint8_t cmd, const uint8_t *d8)
{
    uint8_t f[FRAME_LEN];
    uint8_t chk = 0;
    int i;

    f[0] = FRAME_H1;
    f[1] = FRAME_H2;
    f[2] = cmd;
    for (i = 0; i < FRAME_DATA; i++) f[3 + i] = d8 ? d8[i] : 0;
    for (i = 0; i < FRAME_LEN - 1; i++) chk ^= f[i];
    f[FRAME_LEN - 1] = chk;
    Uart6_SendBuf(f, FRAME_LEN);
}

void Protocol_Poll(void)
{
    uint8_t f[FRAME_LEN];
    uint8_t chk = 0, cmd;
    const uint8_t *data;
    unsigned i;

    if (!g_frame_ready) return;
    for (i = 0; i < FRAME_LEN; i++) f[i] = g_frame[i];
    g_frame_ready = 0;

    for (i = 0; i < FRAME_LEN - 1; i++) chk ^= f[i];
    if (chk != f[FRAME_LEN - 1]) {
        g_proto_err++;
        return;
    }

    cmd = f[2];
    data = &f[3];
    for (i = 0; i < sizeof(CMD_TABLE) / sizeof(CMD_TABLE[0]); i++) {
        if (CMD_TABLE[i].cmd == cmd) {
            if (CMD_TABLE[i].en == 0 || *CMD_TABLE[i].en) {
                CMD_TABLE[i].fn(data);
            }
            return;
        }
    }
}

void Telemetry_SendHB(void)
{
    uint8_t d[FRAME_DATA] = {0};
    uint16_t tick = (uint16_t)millis();
    uint16_t err = (uint16_t)g_proto_err;

    d[0] = FW_VER;
    d[1] = g_motion_en ? 1 : 0;
    d[2] = g_actuator_en ? 1 : 0;
    wr_be16(&d[3], (int16_t)tick);
    wr_be16(&d[5], (int16_t)err);
    Frame_Send(CMD_HB, d);
}

void Telemetry_SendOdo(void)
{
    uint8_t d[FRAME_DATA] = {0};
    int16_t e[4];
    Motion_GetOdo(e);
    wr_be16(&d[0], e[0]);
    wr_be16(&d[2], e[1]);
    wr_be16(&d[4], e[2]);
    wr_be16(&d[6], e[3]);
    Frame_Send(CMD_ODO, d);
}

void Telemetry_SendBat(void)
{
    uint8_t d[FRAME_DATA] = {0};
    uint16_t mv = (uint16_t)(VIN_GetVol_X100() * 10);
    wr_be16(&d[0], (int16_t)mv);
    Frame_Send(CMD_BAT, d);
}

void Sys_OnPing(const uint8_t *d)
{
    (void)d;
    Telemetry_SendHB();
}
