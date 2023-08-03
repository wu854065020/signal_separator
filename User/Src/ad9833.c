#include "ad9833.h"

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


void AD9833_Select(AD9833_Handler* device)
{
	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
}

void AD9833_Unselect(AD9833_Handler* device)
{
	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);
}

void AD9833_WriteRegister(AD9833_Handler* device, uint16_t data)
{
	AD9833_Select(device);
	uint8_t LByte = data & 0xff;
	uint8_t HByte = (data >> 8) & 0xff;
	HAL_SPI_Transmit(device->SPI_handler, &HByte, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(device->SPI_handler, &LByte, 1, HAL_MAX_DELAY);
	AD9833_Unselect(device);
}

void AD9833_WriteCfgReg(AD9833_Handler* device)
{
	uint16_t cfg = 0;
	cfg |= device->_waveform;
	cfg |= device->_sleep_mode;
	cfg |= (device->_freq_source ? F_SELECT_CFG : 0);	//it's unimportant because don't use FREQ1
	cfg |= (device->_phase_source ? P_SELECT_CFG : 0);	//it's unimportant because don't use PHASE1
	cfg |= (device->_reset_state ? RESET_CFG : 0);
	cfg |= B28_CFG;
	AD9833_WriteRegister(device, cfg);
}

void AD9833_SetWaveform(AD9833_Handler* device, WaveDef Wave)
{
	if (Wave == wave_sine) 			device->_waveform = WAVEFORM_SINE;
	else if (Wave == wave_square) 	device->_waveform = WAVEFORM_SQUARE;
	else if (Wave == wave_triangle)	device->_waveform = WAVEFORM_TRIANGLE;
	AD9833_WriteCfgReg(device);
}

void AD9833_SetFrequency(AD9833_Handler* device, float freq)
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

	AD9833_WriteCfgReg(device);	// Update Config Register
	AD9833_WriteRegister(device, LSB);
	AD9833_WriteRegister(device, MSB);
}

void AD9833_SetPhase(AD9833_Handler* device, uint16_t phase_deg)
{
	if(phase_deg < 0) phase_deg = 0;
	else if (phase_deg > 360) phase_deg %= 360;
	uint16_t phase_val  = ((uint16_t)(phase_deg * BITS_PER_DEG)) &  0xFFF;
	AD9833_WriteRegister(device, PHASE0_REG | phase_val);
}

void AD9833_Init(AD9833_Handler* device, WaveDef Wave, uint32_t freq, uint16_t phase_deg, SPI_HandleTypeDef *SPI_handler, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
	device->SPI_handler = SPI_handler;
	device->cs_port = cs_port;
	device->cs_pin = cs_pin;
	device->_sleep_mode = NO_POWERDOWN;
	device->_freq_source = 0;
	device->_phase_source = 0;
	device->_reset_state = 0;
	AD9833_OutputEnable(device, 0);
	AD9833_SetWaveform(device, Wave);
	AD9833_WriteCfgReg(device);
	AD9833_SetFrequency(device, freq);
	AD9833_SetPhase(device, phase_deg);
	AD9833_OutputEnable(device, 1);
}

void AD9833_SleepMode(AD9833_Handler* device, uint8_t mode)
{
	device->_sleep_mode = mode;
	AD9833_WriteCfgReg(device);
}

void AD9833_OutputEnable(AD9833_Handler* device, uint8_t output_state)
{
	device->_reset_state = !output_state;
	AD9833_WriteCfgReg(device);
}
