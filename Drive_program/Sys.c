#include "Sys.h"

static volatile u32 s_systick_ms = 0;

void SysTickConfig(void)
{
    if (SysTick_Config(SystemCoreClock / 1000)) {
        while (1) {
        }
    }

    NVIC_SetPriority(SysTick_IRQn, 0);
}

void Delay_us(u32 t)
{
    u32 i;
    u32 j;

    for (j = 0; j < t; j++) {
        for (i = 0; i < 41; i++) {
            __NOP();
        }
    }
}

void Delay_ms(u32 t)
{
    u32 i;
    u32 j;

    for (j = 0; j < t; j++) {
        for (i = 0; i < 41000; i++) {
            __NOP();
        }
    }
}

u32 millis(void)
{
    return s_systick_ms;
}

void SysTick_Handler(void)
{
    s_systick_ms++;
}
