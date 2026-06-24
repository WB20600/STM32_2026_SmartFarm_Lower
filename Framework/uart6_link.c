#include "uart6_link.h"
#include "protocol.h"

static uint8_t rxbuf[FRAME_LEN];
static uint8_t rxidx = 0;

void Uart6_Link_Init(uint32_t baud)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef  nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);

    usart.USART_BaudRate            = baud;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART6, &usart);

    USART_Cmd(USART6, ENABLE);
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel                   = USART6_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

void Uart6_SendByte(uint8_t b)
{
    USART_SendData(USART6, b);
    while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET) {
    }
}

void Uart6_SendBuf(const uint8_t *p, uint16_t n)
{
    while (n--) {
        Uart6_SendByte(*p++);
    }
}

void USART6_IRQHandler(void)
{
    uint8_t c;
    if (USART_GetITStatus(USART6, USART_IT_RXNE) == RESET) return;
    c = (uint8_t)USART_ReceiveData(USART6);

    if (rxidx == 0) {
        if (c == FRAME_H1) rxbuf[rxidx++] = c;
    } else if (rxidx == 1) {
        if (c == FRAME_H2) rxbuf[rxidx++] = c; else rxidx = 0;
    } else {
        rxbuf[rxidx++] = c;
        if (rxidx >= FRAME_LEN) {
            if (!g_frame_ready) {
                uint8_t i;
                for (i = 0; i < FRAME_LEN; i++) g_frame[i] = rxbuf[i];
                g_frame_ready = 1;
            }
            rxidx = 0;
        }
    }
}
