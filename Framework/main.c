#include "stm32f4xx.h"
#include "uart6_link.h"
#include "scheduler.h"
#include "protocol.h"
#include "motion.h"
#include "actuator.h"

extern void SysTickConfig(void);
extern void Delay_ms(uint32_t t);
extern void GPIO_LED_int(void);
extern void BEEP_Init(void);
extern void beep_on_times(int times, int ms);
extern void UART1_Init(uint32_t baud);
extern void MOTOR_AB_Init(void);
extern void MOTOR_CD_Init(void);
extern void ENCODER_A_Init(void);
extern void ENCODER_B_Init(void);
extern void ENCODER_C_Init(void);
extern void ENCODER_D_Init(void);
extern void PID_int(void);
extern void VIN_Init(void);

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTickConfig();
	 
	  /* Enable Cortex-M4 DWT cycle counter. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    GPIO_LED_int();
    BEEP_Init();
    UART1_Init(115200);

    MOTOR_AB_Init();
    MOTOR_CD_Init();
    ENCODER_A_Init();
    ENCODER_B_Init();
    ENCODER_C_Init();
    ENCODER_D_Init();
    PID_int();
    VIN_Init();

    Motion_Init();
    Actuator_Init();
    Uart6_Link_Init(115200);
    Scheduler_Init();

    beep_on_times(2, 100);
		
		     /* PID zero-speed test: all wheels target 0. */
    {
        uint8_t closed_loop[8] = {1};
        uint8_t zero_speed[8] = {0};

        wr_be16(&zero_speed[0], 10);  // A = 0
        wr_be16(&zero_speed[2], 10);  // B = 0
        wr_be16(&zero_speed[4], 10);  // C = 0
        wr_be16(&zero_speed[6], 10);  // D = 0

        Motion_OnMode(closed_loop);
        Motion_OnSpd(zero_speed);
    }

    while (1) {
        if (Scheduler_TakeProtocolFlag()) {
            Protocol_Poll();
        }
        if (Scheduler_TakeOdoFlag()) {
            Telemetry_SendOdo();
        }
//        if (Scheduler_TakeBatFlag()) {
//            Telemetry_SendBat();
//        }
        if (Scheduler_TakeActuatorFlag()) {
            Actuator_Tick();
        }
        if (Scheduler_TakeHbFlag()) {
            GPIO_ToggleBits(GPIOB, GPIO_Pin_6);
            Telemetry_SendHB();
        }
    }
}
