/*
 * m24c64.cpp
 *
 *  Created on: May 5, 2016
 *      Author: calle
 */

#include <stddef.h>
#include <stdint.h>
#include "stm32f0xx_i2c.h"

#include "m24c64.h"

static const uint8_t EEPROM_ID	= 0xA0;

void m24c64_write(uint16_t address, uint8_t data)
{
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);

  I2C_TransferHandling(I2C1, EEPROM_ID, 2, I2C_Reload_Mode, I2C_Generate_Start_Write);

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);

  I2C_SendData(I2C1, (uint8_t)((address & 0xFF00) >> 8));

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);

  I2C_SendData(I2C1, (uint8_t)(address & 0x00FF));

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TCR) == RESET);

  I2C_TransferHandling(I2C1, EEPROM_ID, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);

  I2C_SendData(I2C1, data);

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_STOPF) == RESET);

  I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);
}

uint8_t m24c64_read(uint16_t address)
{
  uint8_t readData;

  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);

  I2C_TransferHandling(I2C1, EEPROM_ID, 2, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);

  I2C_SendData(I2C1, (uint8_t)((address & 0xFF00) >> 8));

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);

  I2C_SendData(I2C1, (uint8_t)(address & 0x00FF));

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TC) == RESET);

  I2C_TransferHandling(I2C1, EEPROM_ID, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_RXNE) == RESET);

  readData = I2C_ReceiveData(I2C1);

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_STOPF) == RESET);

   /* Clear STOPF flag */
  I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);

  return readData;
}