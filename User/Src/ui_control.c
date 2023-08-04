#include  <string.h>
#include <stdio.h>
#include "ui_control.h"
#include "sample.h"
#include "usart.h"

#define UI_RX_BUF_LEN 64
uint8_t g_uiRxBuf[UI_RX_BUF_LEN];
uint8_t strBuf[100] = {0};
PhaseControlFrame g_phaseControlFrame = {0};
TestControlFrame g_testControlFrame = {0};

void uiControlInit(void)
{
    HAL_UART_Receive_DMA(&huart3, g_uiRxBuf, UI_RX_BUF_LEN);
}

#define WAVEA_FREQ_FORMAT "page0.na.val=%d\xff\xff\xff"
#define WAVEB_FREQ_FORMAT "page0.nb.val=%d\xff\xff\xff"
#define WAVEA_FORMAT "page0.signa.val=%d\xff\xff\xff"
#define WAVEB_FORMAT "page0.signb.val=%d\xff\xff\xff"
void uiSendWaveInf(uint32_t wave1Freq, uint32_t wave2Freq, uint8_t wave1Type, uint8_t wave2Type)
{
    uint16_t wave1KHz = wave1Freq / 1000;
    uint16_t wave2KHz = wave2Freq / 1000;
    sprintf(strBuf, WAVEA_FREQ_FORMAT, wave1KHz);
    HAL_UART_Transmit(&huart3, strBuf, strlen(strBuf), 1000);
    sprintf(strBuf, WAVEB_FREQ_FORMAT, wave2KHz);
    HAL_UART_Transmit(&huart3, strBuf, strlen(strBuf), 1000);
    sprintf(strBuf, WAVEA_FORMAT, wave1Type);
    HAL_UART_Transmit(&huart3, strBuf, strlen(strBuf), 1000);
    sprintf(strBuf, WAVEB_FORMAT, wave2Type);
    HAL_UART_Transmit(&huart3, strBuf, strlen(strBuf), 1000);
}

void uiUartCallBack(void)
{
    uint8_t isPackage = 0;
    uint16_t len = UI_RX_BUF_LEN - huart3.hdmarx->Instance->NDTR;
    switch (g_uiRxBuf[0])
    {
        case PHASE_CONTROL_HEAD:
            if (len == PHASE_CONTROL_LENGTH)
            {
                memcpy((uint8_t *)&g_phaseControlFrame, g_uiRxBuf, PHASE_CONTROL_LENGTH);
                for (uint8_t i = 0; i < 3; i++)
                {
                    if (g_phaseControlFrame.tail[i] != 0x00)
                    {
                        isPackage = 0;
                        break;
                    }
                }
                if (isPackage == 1) {
                    setFirstPhase(g_phaseControlFrame.phase * DEG_TO_RAD);
                    if (g_phaseControlFrame.enable == 0x01)
                    {
                        changeWorkMode(PHASE_MODE);
                    } else {
                        changeWorkMode(NORMAL_MODE);
                    }
                }
            }
            break;
        case TEST_CONTROL_HEAD:
            if (len == TEST_CONTROL_LENGTH)
            {
                memcpy((uint8_t *)&g_testControlFrame, g_uiRxBuf, TEST_CONTROL_LENGTH);
                for (uint8_t i = 0; i < 3; i++)
                {
                    if (g_testControlFrame.tail[i] != 0x00)
                    {
                        isPackage = 0;
                        break;
                    }
                }
                if (isPackage == 1) {
                    if (g_testControlFrame.enable == 0x01)
                    {
                        changeWorkMode(Test_MODE);
                    } else {
                        changeWorkMode(NORMAL_MODE);
                    }
                }
            }
            break;
        default:
            break;
    }
    HAL_UART_Receive_DMA(&huart3, g_uiRxBuf, UI_RX_BUF_LEN);
}
