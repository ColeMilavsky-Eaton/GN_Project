/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "../../src/main/main.h"
#include <stdio.h>

#define TIMCLOCK 64000000
#define PRESCALAR 65535

// Define a unique macro to prevent multiple definitions
#ifndef MAIN_C
#define MAIN_C

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Configure the system clock */
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();

  MX_SPI1_Init();

  // Start the timers
  startTIM14();  // Start Timer 14
  startTIM2();   // Start Timer 2
  //startTIM3();   // Start Timer 3

  // Initialize ADE9039
  ADE9039_Init();

  ConfigurePulseInput();

  /* Infinite loop */
  while (1)
  {
	uint32_t value = retCF3();
	printf("CF3 Value: %lu\n", value);
  }
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *, uint32_t);

#endif

#endif /* MAIN_C */
