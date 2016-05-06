// Motor driver for TI DRV8701
// Driver uses four PWM channels driven by TIMER1 and TIMER3
// Drive the motors with 24Khz PWM to keep noise down and avoid to much switching loss

#include "drv8701.h"
#include <stdint.h>
#include "stm32f0xx_gpio.h"

const uint32_t PERIOD = 1000;   // 24Mhz/1000 = 24Khz
const uint32_t PRESCALER = 2;   // 48/2 = 24 Mhz

void drv8701_init(void)
{
  GPIO_InitTypeDef gpio;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  // PA8 is PWM out
  gpio.GPIO_Pin = GPIO_Pin_8;
  gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_Speed = GPIO_Speed_Level_1;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &gpio);

  // PA9 is PWM out
  gpio.GPIO_Pin = GPIO_Pin_9;
  gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_Speed = GPIO_Speed_Level_1;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &gpio);

  // PB0 is PWM out
  gpio.GPIO_Pin = GPIO_Pin_0;
  gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_Speed = GPIO_Speed_Level_1;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &gpio);

  // PB1 is PWM out
  gpio.GPIO_Pin = GPIO_Pin_1;
  gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_Speed = GPIO_Speed_Level_1;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &gpio);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_1);

  /* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = PRESCALER - 1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = PERIOD;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);
  TIM_Cmd(TIM3, ENABLE);

  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
  TIM_CtrlPWMOutputs(TIM3, ENABLE);
}

void drv8701_setspeed(uint32_t rightSpeed, uint32_t leftSpeed, MotorDirection direction)
{
//  assert(speed < PERIOD);
  
  switch(direction)
  {
  case DRV8701_STOP:
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
    break;

  case DRV8701_FORWARD:
    TIM1->CCR1 = rightSpeed;
    TIM1->CCR2 = 0;
    TIM3->CCR3 = leftSpeed;
    TIM3->CCR4 = 0;
    break;

  case DRV8701_BACKWARD:
    TIM1->CCR1 = 0;
    TIM1->CCR2 = rightSpeed;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = leftSpeed;
    break;

  case DRV8701_TURN_CCW:
    TIM1->CCR1 = rightSpeed;
    TIM1->CCR2 = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = leftSpeed;
    break;

  case DRV8701_TURN_CW:
    TIM1->CCR1 = 0;
    TIM1->CCR2 = rightSpeed;
    TIM3->CCR3 = leftSpeed;
    TIM3->CCR4 = 0;
    break;

  default:
    break;
  }
}
