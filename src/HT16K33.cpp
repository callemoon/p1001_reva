/*
 * HT16K33.cpp
 *
 *  Created on: May 5, 2016
 *      Author: calle
 */
#include <stddef.h>
#include <stdint.h>
#include "stm32f0xx_i2c.h"

#include "i2c.h"

#define I2C_ADDRESS 0xE0

#define HT16K33_BLINK_CMD 0x80
#define HT16K33_BLINK_DISPLAYON 0x01
#define HT16K33_BLINK_OFF 0
#define HT16K33_BLINK_2HZ  1
#define HT16K33_BLINK_1HZ  2
#define HT16K33_BLINK_HALFHZ  3

#define HT16K33_CMD_BRIGHTNESS 0xE0

void ht16k33_i2cwrite(uint8_t i2cAdress, uint8_t command, uint8_t *data, uint8_t len)
{
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);

	I2C_TransferHandling(I2C1, i2cAdress, 1+len, I2C_Reload_Mode, I2C_Generate_Start_Write);

	while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);

	I2C_SendData(I2C1, (uint8_t)command);

	if(len == 0)
	{
		while(I2C_GetFlagStatus(I2C1, I2C_ISR_TCR) == RESET);
	}
	else
	{
		while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);
	}

	for(uint8_t i = 0; i < len; i++)
	{
		while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C1, data[i]);
	}

	I2C_TransferHandling(I2C1, i2cAdress, len, I2C_AutoEnd_Mode, I2C_Generate_Stop);

	while(I2C_GetFlagStatus(I2C1, I2C_ISR_STOPF) == RESET);

	I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);
}

void ht16k33_init(void)
{
	// Turn on ocillator
	ht16k33_i2cwrite(I2C_ADDRESS, 0x21, 0, 0);

	// Blink-mode
	uint8_t blink = HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_2HZ << 1);

	ht16k33_i2cwrite(I2C_ADDRESS, blink, 0, 0);

	// Brightness
	uint8_t brightNess = HT16K33_CMD_BRIGHTNESS | 0xF;

	ht16k33_i2cwrite(I2C_ADDRESS, brightNess, 0, 0);
}

void ht16k33_writepixeldata(void)
{
	// Write pixel-data
	uint8_t data[] = {0x1, 0x0, 0x2, 0x0, 0x4, 0x0, 0x8, 0x0, 0x10, 0x0, 0x20, 0x0, 0x40, 0x0, 0x80, 0x0};

	ht16k33_i2cwrite(I2C_ADDRESS, 0x0, data, 16);
}
