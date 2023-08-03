/*
 * @file AD9833.h
 * @brief Function for the AD9833 chip
 *
 * This contains functions for working with
 * AD9833 signal generator.
 *
 * !!!!IMPORTANT!!!!
 * Setup Hardware SPI to POLATRITY HIGH, PHASE 1 EDGE
 *
 * Offical Documents:
 * https://www.analog.com/media/en/technical-documentation/application-notes/AN-1070.pdf
 * https://www.analog.com/media/en/technical-documentation/data-sheets/AD9833.pdf
 *
 * @author Andrii Ivanchenko <ivanchenko59@gmail.com>
 */

#include "ad9833.h"

	uint8_t _waveform = WAVEFORM_SINE;
    uint8_t _sleep_mode = NO_POWERDOWN;
    uint8_t _freq_source = 0;
    uint8_t _phase_source = 0;
    uint8_t _reset_state = 0;

void AD9833_Select(void)
{
	HAL_GPIO_WritePin(AD9833_FSYNC_GPIO_Port, AD9833_FSYNC_Pin, GPIO_PIN_RESET);
}

void AD9833_Unselect(void)
{
	HAL_GPIO_WritePin(AD9833_FSYNC_GPIO_Port, AD9833_FSYNC_Pin, GPIO_PIN_SET);
}

void AD9833_WriteRegister(uint16_t data)
{
	AD9833_Select();
	uint8_t LByte = data & 0xff;
	uint8_t HByte = (data >> 8) & 0xff;
	HAL_SPI_Transmit(&AD9833_SPI_PORT, &HByte, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&AD9833_SPI_PORT, &LByte, 1, HAL_MAX_DELAY);
	AD9833_Unselect();
}

void AD9833_WriteCfgReg(void)
{
	uint16_t cfg = 0;
	cfg |= _waveform;
	cfg |= _sleep_mode;
	cfg |= (_freq_source ? F_SELECT_CFG : 0);	//it's unimportant because don't use FREQ1
	cfg |= (_phase_source ? P_SELECT_CFG : 0);	//it's unimportant because don't use PHASE1
	cfg |= (_reset_state ? RESET_CFG : 0);
	cfg |= B28_CFG;
	AD9833_WriteRegister(cfg);
}

void AD9833_SetWaveform(WaveDef Wave)
{
	if (Wave == wave_sine) 			_waveform = WAVEFORM_SINE;
	else if (Wave == wave_square) 	_waveform = WAVEFORM_SQUARE;
	else if (Wave == wave_triangle)	_waveform = WAVEFORM_TRIANGLE;
	AD9833_WriteCfgReg();
}

void AD9833_SetFrequency(float freq)
{
	// TODO: calculate max frequency based on refFrequency.
	// Use the calculations for sanity checks on numbers.
	// Sanity check on frequency: Square - refFrequency / 2
	// Sine/Triangle - refFrequency / 4

	if (freq > (FMCLK >> 1))	//bitwise FMCLK / 2
		freq = FMCLK >> 1;
	else if (freq < 0) freq = 0;

	uint32_t freq_reg = (float)freq / FMCLK * (float)((1 << 28)); // Tuning word

	uint16_t LSB = FREQ0_REG | (freq_reg & 0x3FFF);
	uint16_t MSB = FREQ0_REG | (freq_reg >> 14);

	AD9833_WriteCfgReg();	// Update Config Register
	AD9833_WriteRegister(LSB);
	AD9833_WriteRegister(MSB);
}

void AD9833_SetPhase(uint16_t phase_deg)
{
	if(phase_deg < 0) phase_deg = 0;
	else if (phase_deg > 360) phase_deg %= 360;
	uint16_t phase_val  = ((uint16_t)(phase_deg * BITS_PER_DEG)) &  0xFFF;
	AD9833_WriteRegister(PHASE0_REG | phase_val);
}

void AD9833_Init(WaveDef Wave, uint32_t freq, uint16_t phase_deg)
{
	AD9833_OutputEnable(0);
	AD9833_SetWaveform(Wave);
	AD9833_WriteCfgReg();
	AD9833_SetFrequency(freq);
	AD9833_SetPhase(phase_deg);
	AD9833_OutputEnable(1);
}

void AD9833_SleepMode(uint8_t mode)
{
	_sleep_mode = mode;
	AD9833_WriteCfgReg();
}

void AD9833_OutputEnable(uint8_t output_state)
{
	_reset_state = !output_state;
	AD9833_WriteCfgReg();
}
