/**
*****************************************************************************
**
**  File        : main.cpp
**
**  Abstract    : main function.
**
**  Functions   : main
**
**  Environment : Atollic TrueSTUDIO(R)
**
**  Distribution: The file is distributed "as is", without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. This file may only be built (assembled or compiled and linked)
**  using the Atollic TrueSTUDIO(R) product. The use of this file together
**  with other tools than Atollic TrueSTUDIO(R) is not permitted.
**
*****************************************************************************
*/

/* Includes */
#include <stddef.h>
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"

/* Private typedef */

/* Private define  */

/* Private macro */

/* Private variables */

/* Private function prototypes */

/* Private functions */

//#define I2C_TEST
#define I2C_INIT
//#define I2C_WRITE
#define I2C_READ

extern "C"
{
/* Global variables */
uint32_t timer=0;
uint8_t  timerFlag=0;

/**
**===========================================================================
**
**  Abstract: SysTick interrupt handler
**
**===========================================================================
*/
void SysTick_Handler(void)
{
  timer++;
  if  (timer>2000)
  {
    timerFlag = 1;
    timer = 0;
  }
}
}


/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
  uint32_t ii = 0;

  /* TODO - Add your application code here */
  SysTick_Config(48000);  /* 0.1 ms = 100us if clock frequency 12 MHz */

  SystemCoreClockUpdate();
  ii = SystemCoreClock;    /* This is a way to read the System core clock */
  ii = 0;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  GPIO_InitTypeDef gpio;

  // PB5 is user led
  gpio.GPIO_Pin = GPIO_Pin_5;
  gpio.GPIO_Mode = GPIO_Mode_OUT;
  gpio.GPIO_Speed = GPIO_Speed_Level_1;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &gpio);

  // PA10 is distance trig
  gpio.GPIO_Pin = GPIO_Pin_10;
  gpio.GPIO_Mode = GPIO_Mode_OUT;
  gpio.GPIO_Speed = GPIO_Speed_Level_1;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &gpio);

  // PA0-3 is analog in
  gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  gpio.GPIO_Mode = GPIO_Mode_AN;
  gpio.GPIO_Speed = GPIO_Speed_Level_3;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &gpio);

#ifdef I2C_TEST
  // PB6 I2C1_SCL
  // PB7 I2C1_SDA
  gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  gpio.GPIO_Mode = GPIO_Mode_OUT;
  gpio.GPIO_Speed = GPIO_Speed_Level_3;
  gpio.GPIO_OType = GPIO_OType_OD;
  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &gpio);
#endif

#ifdef I2C_INIT
  I2C_InitTypeDef  I2C_InitStructure;

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

  uint16_t WriteAddr = 1;
#endif

#ifdef I2C_WRITE
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);

  I2C_TransferHandling(I2C1, 0xA0, 2, I2C_Reload_Mode, I2C_Generate_Start_Write);

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);

  I2C_SendData(I2C1, (uint8_t)((WriteAddr & 0xFF00) >> 8));

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);

  I2C_SendData(I2C1, (uint8_t)(WriteAddr & 0x00FF));

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TCR) == RESET);

  I2C_TransferHandling(I2C1, 0xA0, 2, I2C_AutoEnd_Mode, I2C_No_StartStop);

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);

  I2C_SendData(I2C1, 51);

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);

  I2C_SendData(I2C1, 52);

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_STOPF) == RESET);

  I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);
#endif

#ifdef I2C_READ
  uint8_t readback;
  uint8_t readback2;

  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);
  // Read back data
  I2C_TransferHandling(I2C1, 0xA0, 2, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);

  I2C_SendData(I2C1, (uint8_t)((WriteAddr & 0xFF00) >> 8));

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);

  I2C_SendData(I2C1, (uint8_t)(WriteAddr & 0x00FF));

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_TC) == RESET);

  I2C_TransferHandling(I2C1, 0xA0, 2, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_RXNE) == RESET);

  readback = I2C_ReceiveData(I2C1);

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_RXNE) == RESET);

  readback2 = I2C_ReceiveData(I2C1);

  while(I2C_GetFlagStatus(I2C1, I2C_ISR_STOPF) == RESET);

   /* Clear STOPF flag */
  I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);
#endif

  bool on = false;

  while (1)
  {
	  if (timerFlag)
	  {
		  timerFlag = 0;
		  ii++;

		  on = !on;

		  if(on)
		  {
			  GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_SET);
			  GPIO_WriteBit(GPIOA, GPIO_Pin_10, Bit_SET);

#ifdef I2C_TEST
			  GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_SET);
			  GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);
#endif
		  }
		  else
		  {
			  GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_RESET);
			  GPIO_WriteBit(GPIOA, GPIO_Pin_10, Bit_RESET);

#ifdef I2C_TEST
			  GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_RESET);
			  GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_RESET);
#endif
		  }
	  }
  }
  return 0;
}
