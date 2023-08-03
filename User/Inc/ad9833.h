/*
 * @file AD9833.h
 * @brief Function prototypes for the AD9833 chip
 *
 * This contains the prototypes for the AD9833
 * driver and eventually any macros, constants.
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

#ifndef INC_AD9833_H_
#define INC_AD9833_H_

#include "main.h"

/*** Redefine if necessary ***/
#define AD9833_SPI_PORT 		hspi1
extern SPI_HandleTypeDef 		AD9833_SPI_PORT;
#define AD9833_FSYNC_GPIO_Port 	GPIOC
#define AD9833_FSYNC_Pin 		GPIO_PIN_4

/*** Control Register Bits (DataSheet AD9833 p. 14, Table 6) ***/
#define B28_CFG					(1 << 13)
#define HLB_CFG		  			(1 << 12)
#define F_SELECT_CFG			(1 << 11)
#define P_SELECT_CFG			(1 << 10)
#define RESET_CFG				(1 << 8)
#define SLEEP1_CFG				(1 << 7)
#define SLEEP12_CFG				(1 << 6)
#define OPBITEN_CFG				(1 << 5)
#define DIV2_CFG				(1 << 3)
#define MODE_CFG				(1 << 1)

/*** Bitmask to register access ***/
#define FREQ0_REG				0x4000
#define PHASE0_REG				0xC000
//#define FREQ1_ACCESS  			0x8000
//#define PHASE1_ACCESS 			0xE000

/*** Waveform Types (DataSheet p. 16, Table 15) ***/
#define WAVEFORM_SINE         	0
#define WAVEFORM_TRIANGLE     	MODE_CFG
#define WAVEFORM_SQUARE       	OPBITEN_CFG | DIV2_CFG
#define WAVEFORM_SQUARE_DIV2  	OPBITEN_CFG

/*** Sleep Modes ***/
#define NO_POWERDOWN	  		0
#define DAC_POWERDOWN			SLEEP12_CFG
#define CLOCK_POWERDOWN			SLEEP1_CFG
#define FULL_POWERDOWN			SLEEP12_CFG | SLEEP1_CFG

#define FMCLK	 				25000000
#define BITS_PER_DEG 			11.3777777777778	// 4096 / 360

typedef enum {
	wave_triangle,
	wave_square,
	wave_sine,
} WaveDef;

/*
 * @brief Set Chip Select pin to LOW state
 */
void AD9833_Select(void);

/*
 * @brief Set Chip Select pin to HIGH state
 */
void AD9833_Unselect(void);

/*
 * @brief Send data by SPI protocol
 * @param Data variable in uint16_t format
*/
void AD9833_WriteRegister(uint16_t data);

/*
 * @brief Update Control Register Bits
 */
void AD9833_WriteCfgReg(void);

/*
 * @brief Set signal generation frequency
 * @param Frequency value in uint32_t format
 */
void AD9833_SetFrequency(float freq);

/*
 * @brief Set signal generation waveform
 * @param Waveform in WaveDef Type declared in .h file
 */
void AD9833_SetWaveform(WaveDef Wave);

/*
 * @brief Set signal generation phase
 * @param Phase in degrees in uint16_t format. Value can be large then 360
 */
void AD9833_SetPhase(uint16_t phase_deg);

/*
 * @brief AD9833 Initial Configuration
 * @param Type of Waveform, Frequency, Phase in degrees
 */
void AD9833_Init(WaveDef Wave, uint32_t freq, uint16_t phase_deg);

/*
 * @brief Enable or disable the output of the AD9833
 * @param Output state (ON/OFF)
 */
void AD9833_OutputEnable(uint8_t output_state);

/*
 * @brief Set Sleep Mode Function (Explained in datasheet Table 14)
 * @param Mode of sleep function defined in title
 */
void AD9833_SleepMode(uint8_t mode);

#endif /* INC_AD9833_H_ */
