#pragma once

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

// define for different project
#define G431RB
// #define F302R8
// #define MCSDK

#ifdef G431RB

#include "stm32g4xx.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_uart.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_i2c.h"

#include "BH1750.h"

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

#define DEBUGX

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
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;

extern int testCounter;
extern bool testBool;
extern int testInt;
extern int testCase;

void delay_us(uint32_t nus);
void printInfo(void);
void setUserLED(LEDState);
void toggleUserLED(void);

#ifdef MCSDK
char* printMotorState(MCI_State_t);
void printMotorError(uint16_t);
#endif
