// Line following algorithm

#include "linefollower.h"

#include "drv8701.h"
#include "qre1113.h"
#include "stm32f0xx_gpio.h"


#include <stdint.h>

const uint32_t LOOPTIME_MS = 3;   // loop time

const uint32_t SLOWSPEED = 840; // Speed in slow mode
const uint32_t ULTRASLOWSPEED = 750;  // Speed in ultra slow mode
const uint32_t FULLSPEED = 880; // Speed in fast mode

const uint32_t LINESENSORS = 4; // Number of line sensors available

typedef enum
{
  NORMAL,
  CALIBRATE_BLACKLEVEL,
  CALIBRATE_WHITELEVEL,
  PID
}runMode;

typedef enum
{
  STOP,
  LEFT,
  RIGHT,
  SLOWLEFT,
  SLOWRIGHT,
  ROTATE,
  FORWARD
}direction;

// 0 = white
// 1 = black

// direction of robot for all possible combinations of line sensor values
direction directions[] =
{
  ROTATE,   // 0000
  RIGHT,     // 0001
  SLOWRIGHT,     // 0010
  SLOWRIGHT,     // 0011
  SLOWLEFT,   // 0100
  SLOWRIGHT,     // 0101
  FORWARD,   // 0110
  SLOWRIGHT,     // 0111
  LEFT,    // 1000
  FORWARD,   // 1001
  SLOWLEFT,    // 1010
  SLOWRIGHT,     // 1011
  SLOWLEFT,    // 1100
  SLOWLEFT,    // 1101
  SLOWLEFT,    // 1110
  STOP     // 1111
}; 


// right = +
// left = -

int32_t errors[] =
{
    0,   // 0000
    5,     // 0001
    1,     // 0010
    2,     // 0011
    -1,   // 0100
    1,     // 0101
    0,   // 0110
    1,     // 0111
    -5,    // 1000
    0,   // 1001
    -1,    // 1010
    1,     // 1011
    -2,    // 1100
    -1,    // 1101
    -1,    // 1110
    0     // 1111
};

void linefollower_init(void)
{
  drv8701_init();   // motor driver
  qre1113_init();   // line sensor
//
//  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);   // push button
//
//  HAL_Delay(100); // Short delay to avoid false readings
//
//  BSP_LED_Init(LED3);
//  BSP_LED_Init(LED5);
//  BSP_LED_Init(LED10);
//  BSP_LED_Init(LED9);
}

void linefollower_run(void)
{  
  uint32_t lineSensor[LINESENSORS];

//  runMode mode = CALIBRATE_WHITELEVEL;
  runMode mode = NORMAL;

//  uint32_t blackLevel = 0;
//  uint32_t whiteLevel = 0;
  uint32_t threashold = 300;

  int32_t totalError = 0;
  uint32_t rightSpeed;

  uint32_t lineValue;

  while(1)
  {
    uint8_t i;

    lineValue = 0;

    // Read sensor data
    for(i = 0; i < LINESENSORS; i++)
    {
      lineSensor[i] = (qre1113_getValue(i) > threashold);
      lineValue += (lineSensor[i] << i);
    }

    // Indicate if line is detected at a certain sensor
    if(lineSensor[0])
    {
      GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_SET);
    }
    else
    {
      GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_RESET);
    }
//
//    if(lineSensor[1])
//    {
//      BSP_LED_On(LED5);
//    }
//    else
//    {
//      BSP_LED_Off(LED5);
//    }
//
//    if(lineSensor[2])
//    {
//      BSP_LED_On(LED9);
//    }
//    else
//    {
//      BSP_LED_Off(LED9);
//    }
//
//    if(lineSensor[3])
//    {
//      BSP_LED_On(LED10);
//    }
//    else
//    {
//      BSP_LED_Off(LED10);
//    }

    // Check user button
//    if (BSP_PB_GetState(BUTTON_USER))
//    {
//      switch (mode)
//      {
//      case CALIBRATE_WHITELEVEL:
//        whiteLevel = qre1113_getValue(0);
//
//        mode = CALIBRATE_BLACKLEVEL;
//        break;
//
//      case CALIBRATE_BLACKLEVEL:
//        blackLevel = qre1113_getValue(0);
//
//        threashold = (blackLevel + whiteLevel) / 2;
//
//        mode = PID;
//        break;
//
//      default:
//        break;
//      }
//
//      // Debounce
//      HAL_Delay(500);
//    }

//    if(mode == PID)
//    {
//      rightSpeed = SLOWSPEED + (errors[lineValue] * 20) + (totalError * 4);
//      totalError += errors[lineValue];
//
//      if(totalError > 100)
//        totalError = 100;
//
//      if(totalError < -100)
//        totalError = -100;
//
//      drv8701_setspeed(rightSpeed, SLOWSPEED, DRV8701_FORWARD);
//    }

    if(mode == NORMAL)
    {
      switch(directions[lineValue])
      {
      case FORWARD:
        //          printf("forward\n");
        drv8701_setspeed(FULLSPEED, FULLSPEED, DRV8701_FORWARD);
        break;

      case RIGHT:
        //          printf("left\n");
        drv8701_setspeed(SLOWSPEED, SLOWSPEED, DRV8701_TURN_CCW);
        break;

      case LEFT:
        //          printf("right\n");
        drv8701_setspeed(SLOWSPEED, SLOWSPEED, DRV8701_TURN_CW);
        break;

      case SLOWRIGHT:
        //          printf("slowleft\n");
        drv8701_setspeed(FULLSPEED, SLOWSPEED, DRV8701_FORWARD);
        break;

      case SLOWLEFT:
        //          printf("slowright\n");
        drv8701_setspeed(SLOWSPEED, FULLSPEED, DRV8701_FORWARD);
        break;

      case STOP:
        //          printf("stop\n");
        drv8701_setspeed(0, 0, DRV8701_STOP);
        break;

      case ROTATE:
        //          printf("rotate\n");
        drv8701_setspeed(SLOWSPEED, SLOWSPEED, DRV8701_TURN_CW);
        break;
      }
    }

    //HAL_Delay(LOOPTIME_MS);
  }
}

