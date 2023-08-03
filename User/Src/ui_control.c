#include "ui_control.h"
#include "usart.h"

#define UI_RX_BUF_LEN 64
uint8_t g_uiRxBuf[UI_RX_BUF_LEN];

void uiControlInit(void)
{
    HAL_UART_Receive_DMA(&huart3, g_uiRxBuf, UI_RX_BUF_LEN);
}

void uiUartCallBack(void)
{
    HAL_UART_Receive_DMA(&huart3, g_uiRxBuf, UI_RX_BUF_LEN);
}
