/*
 * i2c.cpp
 *
 *  Created on: May 5, 2016
 *      Author: calle
 */

#include <stddef.h>
#include <stdint.h>
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_i2c.h"

void I2C_Config(void)
{
  I2C_InitTypeDef  I2C_InitStructure;
  GPIO_InitTypeDef gpio;

  RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_1);

  // PB6 I2C1_SCL
  // PB7 I2C1_SDA
  gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_Speed = GPIO_Speed_Level_3;
  gpio.GPIO_OType = GPIO_OType_OD;
  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &gpio);

  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
  I2C_InitStructure.I2C_DigitalFilter = 0x00;
  I2C_InitStructure.I2C_OwnAddress1 = 0x0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_Timing = 0x10420f13;

  /* Apply sEE_I2C configuration after enabling it */
  I2C_Init(I2C1, &I2C_InitStructure);

  /* sEE_I2C Peripheral Enable */
  I2C_Cmd(I2C1, ENABLE);
}



