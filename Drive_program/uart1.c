#include "uart1.h"

#include <stdio.h>

void UART1_Init(uint32_t baud)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    gpio.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate            = baud;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart);
    USART_Cmd(USART1, ENABLE);
}

void uart1_send_byte(uint8_t dat)
{
    USART_SendData(USART1, dat);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
    }
}

void uart1_send_str(const uint8_t *s)
{
    while (*s) {
        uart1_send_byte(*s++);
    }
}

int fputc(int ch, FILE *f)
{
    (void)f;
    USART_SendData(USART1, (uint8_t)ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {
    }
    return ch;
}

int fgetc(FILE *f)
{
    (void)f;
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET) {
    }
    return (int)USART_ReceiveData(USART1);
}
