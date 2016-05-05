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

#include "i2c.h"
#include "m24c64.h"
#include "ht16k33.h"

/* Private typedef */

/* Private define  */

/* Private macro */

/* Private variables */

/* Private function prototypes */

/* Private functions */

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

static void ADC_Config(void)
{
  ADC_InitTypeDef     ADC_InitStructure;
  GPIO_InitTypeDef    GPIO_InitStructure;

  /* GPIOA Periph clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* ADC1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  /* Configure ADC Channel 0,1,2,3 as analog input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* ADCs DeInit */
  ADC_DeInit(ADC1);

  /* Initialize ADC structure */
  ADC_StructInit(&ADC_InitStructure);

  /* Configure the ADC1 in continuous mode with a resolution equal to 12 bits  */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* Convert the ADC1 Channel with 239.5 Cycles as sampling time */
  ADC_ChannelConfig(ADC1, ADC_Channel_3 , ADC_SampleTime_239_5Cycles);

  /* ADC Calibration */
  ADC_GetCalibrationFactor(ADC1);

  /* Enable the ADC peripheral */
  ADC_Cmd(ADC1, ENABLE);

  /* Wait the ADRDY flag */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY));

  /* ADC1 regular Software Start Conv */
  ADC_StartOfConversion(ADC1);
}

static void GPIOPin_Config()
{

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

	// PB4 is user key
	gpio.GPIO_Pin = GPIO_Pin_4;
	gpio.GPIO_Mode = GPIO_Mode_IN;
	gpio.GPIO_Speed = GPIO_Speed_Level_3;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &gpio);

	// PA10 is distance trig
	gpio.GPIO_Pin = GPIO_Pin_10;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_Speed = GPIO_Speed_Level_1;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpio);

#ifdef I2C_PIN_TEST
	// PB6 I2C1_SCL
	// PB7 I2C1_SDA
	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_Speed = GPIO_Speed_Level_3;
	gpio.GPIO_OType = GPIO_OType_OD;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &gpio);
#endif
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


  GPIOPin_Config();
  ADC_Config();

  bool on = false;
  bool ledOn = true;
  bool lastButtonState = false;
  bool buttonState = false;

  i2c_init();

  uint8_t readback;

  //m24c64_write(0, 0x26);
  //for(volatile int i = 0; i < 10000; i++);
  //readback = m24c64_read(0);

//  ht16k33_init();
//  ht16k33_writepixeldata();

  while (1)
  {
	  uint16_t adcValue;

	  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

	  adcValue = ADC_GetConversionValue(ADC1);

	  buttonState = (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == Bit_RESET);

	  if((buttonState != lastButtonState) && buttonState)
	  {
		  ledOn = !ledOn;
	  }

	  lastButtonState = buttonState;

	  if(ledOn)
	  {
		  GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_SET);
	  }
	  else
	  {
		  GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_RESET);
	  }

	  if (timerFlag)
	  {
		  timerFlag = 0;
		  ii++;

		  on = !on;

		  if(on)
		  {
			  GPIO_WriteBit(GPIOA, GPIO_Pin_10, Bit_SET);

#ifdef I2C_PIN_TEST
			  GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_SET);
			  GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);
#endif
		  }
		  else
		  {
			  GPIO_WriteBit(GPIOA, GPIO_Pin_10, Bit_RESET);

#ifdef I2C_PIN_TEST
			  GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_RESET);
			  GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_RESET);
#endif
		  }
	  }
  }
  return 0;
}
