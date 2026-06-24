#include "motion.h"
#include "protocol.h"

extern void MOTOR_A_SetSpeed(int16_t s);
extern void MOTOR_B_SetSpeed(int16_t s);
extern void MOTOR_C_SetSpeed(int16_t s);
extern void MOTOR_D_SetSpeed(int16_t s);
extern void MOTOR_A_SetSpeed_Close(float aim, float enc);
extern void MOTOR_B_SetSpeed_Close(float aim, float enc);
extern void MOTOR_C_SetSpeed_Close(float aim, float enc);
extern void MOTOR_D_SetSpeed_Close(float aim, float enc);

volatile uint8_t g_motion_en = 1;

static volatile int16_t s_target_active[4] = {0, 0, 0, 0};
static volatile int16_t s_target_pending[4] = {0, 0, 0, 0};
static volatile uint8_t s_target_dirty = 0;

static volatile uint8_t s_closed_active = 1;
static volatile uint8_t s_closed_pending = 1;
static volatile uint8_t s_mode_dirty = 0;

static volatile int16_t s_enc[4] = {0, 0, 0, 0};

static int16_t encGet(TIM_TypeDef *T)
{
    int16_t c = (int16_t)TIM_GetCounter(T);
    TIM_SetCounter(T, 0);
    return c;
}

static void commit_target(int16_t a, int16_t b, int16_t c, int16_t d)
{
    __disable_irq();
    s_target_pending[0] = a;
    s_target_pending[1] = b;
    s_target_pending[2] = c;
    s_target_pending[3] = d;
    s_target_dirty = 1;
    __enable_irq();
}

void Motion_Init(void)
{
    commit_target(0, 0, 0, 0);
    __disable_irq();
    s_closed_pending = 1;
    s_mode_dirty = 1;
    __enable_irq();
}

void Motion_OnSpd(const uint8_t *d)
{
    commit_target(be16(&d[0]), be16(&d[2]), be16(&d[4]), be16(&d[6]));
}

void Motion_OnStop(const uint8_t *d)
{
    (void)d;
    commit_target(0, 0, 0, 0);
}

void Motion_OnMode(const uint8_t *d)
{
    __disable_irq();
    s_closed_pending = d[0] ? 1 : 0;
    s_mode_dirty = 1;
    __enable_irq();
}

void Motion_SpeedLoop(void)
{
    int16_t target[4];
    uint8_t closed;
    int16_t ea = encGet(TIM2), eb = encGet(TIM3), ec = encGet(TIM4), ed = encGet(TIM5);

    s_enc[0] = ea;
    s_enc[1] = eb;
    s_enc[2] = ec;
    s_enc[3] = ed;

    if (s_target_dirty) {
        s_target_active[0] = s_target_pending[0];
        s_target_active[1] = s_target_pending[1];
        s_target_active[2] = s_target_pending[2];
        s_target_active[3] = s_target_pending[3];
        s_target_dirty = 0;
    }
    if (s_mode_dirty) {
        s_closed_active = s_closed_pending;
        s_mode_dirty = 0;
    }

    target[0] = s_target_active[0];
    target[1] = s_target_active[1];
    target[2] = s_target_active[2];
    target[3] = s_target_active[3];
    closed = s_closed_active;

    /* Vehicle convention: positive target means forward.
     * Bench test showed raw A/C positive and raw B/D negative are forward.
     */
    if (closed) {
        MOTOR_A_SetSpeed_Close( (float)target[0], (float)ea);
        MOTOR_B_SetSpeed_Close(-(float)target[1], (float)(-eb));
        MOTOR_C_SetSpeed_Close( (float)target[2], (float)ec);
        MOTOR_D_SetSpeed_Close(-(float)target[3], (float)(-ed));
    } else {
        MOTOR_A_SetSpeed( target[0]);
        MOTOR_B_SetSpeed(-target[1]);
        MOTOR_C_SetSpeed( target[2]);
        MOTOR_D_SetSpeed(-target[3]);
    }
}

void Motion_GetOdo(int16_t *abcd)
{
    __disable_irq();
    abcd[0] = s_enc[0];
    abcd[1] = s_enc[1];
    abcd[2] = s_enc[2];
    abcd[3] = s_enc[3];
    __enable_irq();
}
