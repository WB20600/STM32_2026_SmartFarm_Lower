#include "actuator.h"
#include "protocol.h"

volatile uint8_t g_actuator_en = 1;

static volatile ActuatorState s_state;
static volatile uint8_t s_dirty = 0;

void Actuator_Init(void)
{
    __disable_irq();
    s_state.arm_x = 0;
    s_state.arm_y = 0;
    s_state.arm_z = 0;
    s_state.arm_r = 0;
    s_state.grip_on = 0;
    s_state.fire_count = 0;
    s_state.lift_target = 0;
    s_state.storage_id = 0;
    s_dirty = 0;
    __enable_irq();
}

void Actuator_OnArm(const uint8_t *d)
{
    __disable_irq();
    s_state.arm_x = be16(&d[0]);
    s_state.arm_y = be16(&d[2]);
    s_state.arm_z = be16(&d[4]);
    s_state.arm_r = be16(&d[6]);
    s_dirty = 1;
    __enable_irq();
}

void Actuator_OnGrip(const uint8_t *d)
{
    __disable_irq();
    s_state.grip_on = d[0] ? 1 : 0;
    s_dirty = 1;
    __enable_irq();
}

void Actuator_OnFire(const uint8_t *d)
{
    __disable_irq();
    s_state.fire_count = d[0];
    s_dirty = 1;
    __enable_irq();
}

void Actuator_OnLift(const uint8_t *d)
{
    __disable_irq();
    s_state.lift_target = be16(&d[0]);
    s_dirty = 1;
    __enable_irq();
}

void Actuator_OnStorage(const uint8_t *d)
{
    __disable_irq();
    s_state.storage_id = d[0];
    s_dirty = 1;
    __enable_irq();
}

void Actuator_Tick(void)
{
    if (!s_dirty) return;

    /* Hardware mapping is intentionally left here, not in the UART ISR.
     * Connect servo, stepper, launcher, or bus-servo drivers after the
     * 2026 mechanisms are finalized.
     */
    s_dirty = 0;
}

void Actuator_GetState(ActuatorState *state)
{
    __disable_irq();
    state->arm_x = s_state.arm_x;
    state->arm_y = s_state.arm_y;
    state->arm_z = s_state.arm_z;
    state->arm_r = s_state.arm_r;
    state->grip_on = s_state.grip_on;
    state->fire_count = s_state.fire_count;
    state->lift_target = s_state.lift_target;
    state->storage_id = s_state.storage_id;
    __enable_irq();
}
