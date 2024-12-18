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
#include "../main/main.h"

#include <stdio.h>

#define TIMCLOCK 64000000
#define PRESCALAR 65535

// Define a unique macro to prevent multiple definitions
#ifndef MAIN_C
#define MAIN_C

//////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Used for testing, not needed for final project
//////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
void LED_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE(); // Enable the GPIOB clock

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_3; // Pin 3 is the built-in LED on the Nucleo-32
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// Function to blink the LED
void Blink_LED(void) {
    LED_Init(); // Initialize the LED

    while (1) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3); // Toggle the LED
        HAL_Delay(500); // Delay for 500 milliseconds
    }
}
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
	printf("Start...");
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Configure the system clock */
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();

  MX_SPI1_Init();
  ConfigurePulseInput();

  // Initialize ADE9039
  ADE9039_Init();

  // Start the timers
  startTIM14();  // Start Timer 14
  startTIM2();   // Start Timer 2
  startTIM3();   // Start Timer 3

  /* Infinite loop */
  while (1)
  {
	  printf("Looping...\n");
	if (getNewFrequencyAvailable())
	{
	  resetNewFrequencyAvailable();

	  // Check for ground neutral fault
	  if (getFrequency() < 1500000) // Not sure exact value to compare frequency with(frequency should equal 2000000)
	  {
	     // Handle fault condition
		 //Blink_LED(); // Used for testing, will delete in final project
	     //LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6);  // Break the circuit // Commented out for testing
	     while(1) {} // Latch in fault state
	  }
	}
  }
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *, uint32_t);

#endif

#endif /* MAIN_C */
