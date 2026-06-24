#include "scheduler.h"
#include "motion.h"

static volatile uint8_t s_protocol_flag = 0;
static volatile uint8_t s_odo_flag = 0;
static volatile uint8_t s_hb_flag = 0;
static volatile uint8_t s_bat_flag = 0;
static volatile uint8_t s_actuator_flag = 0;

static uint8_t take_flag(volatile uint8_t *flag)
{
    uint8_t v;
    __disable_irq();
    v = *flag;
    *flag = 0;
    __enable_irq();
    return v;
}

void Scheduler_Init(void)
{
    TIM_TimeBaseInitTypeDef t;
    NVIC_InitTypeDef        n;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    t.TIM_Prescaler     = 84 - 1;
    t.TIM_Period        = 1000 - 1;
    t.TIM_ClockDivision = TIM_CKD_DIV1;
    t.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &t);

    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    n.NVIC_IRQChannel                   = TIM6_DAC_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 0;
    n.NVIC_IRQChannelSubPriority        = 0;
    n.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&n);

    TIM_Cmd(TIM6, ENABLE);
}

void TIM6_DAC_IRQHandler(void)
{
    static uint16_t t = 0;
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) == RESET) return;
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

    t++;
    s_protocol_flag = 1;

    if (t % 5 == 0) {
        Motion_SpeedLoop();
    }
    if (t % 20 == 0) {
        s_odo_flag = 1;
    }
    if (t % 50 == 0) {
        s_actuator_flag = 1;
    }
    if (t % 500 == 0) {
        s_hb_flag = 1;
    }
    if (t >= 1000) {
        s_bat_flag = 1;
        t = 0;
    }
}

uint8_t Scheduler_TakeProtocolFlag(void) { return take_flag(&s_protocol_flag); }
uint8_t Scheduler_TakeOdoFlag(void)      { return take_flag(&s_odo_flag); }
uint8_t Scheduler_TakeHbFlag(void)       { return take_flag(&s_hb_flag); }
uint8_t Scheduler_TakeBatFlag(void)      { return take_flag(&s_bat_flag); }
uint8_t Scheduler_TakeActuatorFlag(void) { return take_flag(&s_actuator_flag); }
