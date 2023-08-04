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

#define WAVEA_OFFSET_FORMAT "page2.n0.val=%d\xff\xff\xff"
#define WAVEB_OFFSET_FORMAT "page3.n1.val=%d\xff\xff\xff"
void uiSendOffset(uint8_t offset1, uint8_t offset2)
{
    sprintf(strBuf, WAVEA_OFFSET_FORMAT, offset1);
    HAL_UART_Transmit(&huart3, strBuf, strlen(strBuf), 1000);
    sprintf(strBuf, WAVEB_OFFSET_FORMAT, offset2);
    HAL_UART_Transmit(&huart3, strBuf, strlen(strBuf), 1000);
}

void uiUartCallBack(void)
{
    uint8_t isTail = 1;
    uint16_t len = UI_RX_BUF_LEN - huart3.hdmarx->Instance->NDTR;
    // HAL_UART_AbortReceive(&huart3);
    HAL_UART_DMAStop(&huart3);
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
                        isTail = 0;
                        break;
                    }
                }
                if (isTail == 1) {
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
                        isTail = 0;
                        break;
                    }
                }
                if (isTail == 1) {
                    if (g_testControlFrame.enable == 0x02)
                    {
                        changeWorkMode(Test_MODE);
                    } else if (g_testControlFrame.enable == 0x00) {
                        changeWorkMode(NORMAL_MODE);
                    } else if (g_testControlFrame.enable == 0x01) {
                        setFreqOffsetRatio(g_testControlFrame.wave-1, g_testControlFrame.offset);
                    }
                    
                }
            }
            break;
        default:
            break;
    }
    __HAL_DMA_SET_COUNTER(huart3.hdmarx, 0);
    HAL_UART_Receive_DMA(&huart3, g_uiRxBuf, UI_RX_BUF_LEN);
}
