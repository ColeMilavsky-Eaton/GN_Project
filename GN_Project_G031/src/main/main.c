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
#include "main.h"

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

  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();

  // Initialize ADE9039
  ADE9039_Init();
  ConfigurePulseInput();


  uint32_t reg_0010 = ADE9039_ReadReg(0x0010);
  uint32_t reg_481 = ADE9039_ReadReg(0x481);
  uint32_t reg_480 = ADE9039_ReadReg(0x480);
  uint32_t reg_0120 = ADE9039_ReadReg(0x0120);
  uint32_t reg_0102 = ADE9039_ReadReg(0x0102);
  uint32_t reg_420 = ADE9039_ReadReg(0x420);
  uint32_t reg_421 = ADE9039_ReadReg(0x421);
  uint32_t reg_422 = ADE9039_ReadReg(0x422);
  uint32_t reg_425 = ADE9039_ReadReg(0x425);
  uint32_t reg_490 = ADE9039_ReadReg(0x490);
  uint32_t reg_496 = ADE9039_ReadReg(0x496);
  uint32_t reg_4B9 = ADE9039_ReadReg(0x4B9);

  printf("Register 0x0010 value: %06X\n", reg_0010);
  printf("Register 0x481 value: %06X\n", reg_481);
  printf("Register 0x480 value: %06X\n", reg_480);
  printf("Register 0x0120 value: %06X\n", reg_0120);
  printf("Register 0x0102 value: %06X\n", reg_0102);
  printf("Register 0x420 value: %06X\n", reg_420);
  printf("Register 0x421 value: %06X\n", reg_421);
  printf("Register 0x422 value: %06X\n", reg_422);
  printf("Register 0x425 value: %06X\n", reg_425);
  printf("Register 0x490 value: %06X\n", reg_490);
  printf("Register 0x496 value: %06X\n", reg_496);
  printf("Register 0x4B9 value: %06X\n", reg_4B9);


  // Infinite loop
  while (1){}
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *, uint32_t);

#endif

#endif /* MAIN_C */
