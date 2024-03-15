#pragma once

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

// define for different project
#define DEBUGX
#define G431RB
// #define F302R8
// #define MCSDK

// #define TEST_BH1750
// #define TEST_TCS3472
// #define TEST_CAN
// #define USE_PS2

#ifdef G431RB

#include "stm32g4xx.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_uart.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_i2c.h"
#include "stm32g4xx_hal_fdcan.h"
#include "stm32g4xx_hal_pwr_ex.h"

#include "BH1750.h"
#include "TCS3472.h"
#include "hal_extension.h"
#include "zlac8015d.h"

#endif

#ifdef F302R8
#include "stm32f3xx.h"
#include "stm32f3xx_hal_gpio.h"
#endif

#ifdef MCSDK
#include "mc_type.h"
#include "mc_interface.h"
#include "motorcontrol.h"
#endif

#ifndef DEBUGX
  #define xlog
#else
  #define xlog printf
#endif

#define toString(x) #x

typedef enum
{
  LED_Off  = 0U,
  LED_On
} LEDState;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern FDCAN_HandleTypeDef hfdcan1;
extern HAL_StatusTypeDef HALStatus;

extern int testCounter;
extern bool testBool;
extern int testInt;
extern int testCase;

// task flags
extern bool task_PS2;

void printInfo(void);
void setUserLED(LEDState);
void toggleUserLED(void);

