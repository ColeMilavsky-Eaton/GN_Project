/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "main_internal.h"
#include "stm32g0xx_ll_spi.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private defines -----------------------------------------------------------*/
#define TRIP_LED_Pin LL_GPIO_PIN_6
#define TRIP_LED_GPIO_Port GPIOC
#define TRIP_SIGNAL_Pin LL_GPIO_PIN_7
#define TRIP_SIGNAL_GPIO_Port GPIOB
#define CF3_Pin LL_GPIO_PIN_0
#define CF3_GPIO_Port GPIOA
#define PWM_1KHz_Pin LL_GPIO_PIN_4
#define PWM_1KHz_GPIO_Port GPIOA
#define CF4_Pin LL_GPIO_PIN_6
#define CF4_GPIO_Port GPIOA
#define PWM_1Hz_Pin LL_GPIO_PIN_8
#define PWM_1Hz_GPIO_Port GPIOA

#define TIMCLOCK 64000000
#define PRESCALAR 65535

#define HAL_MODULE_ENABLED

/* Private Variables ---------------------------------------------*/
static int firstCaptured = 0;
static uint32_t difference = 0;
static uint32_t lastCaptureValue = 0;

static uint32_t CF3_1 = 0;
static uint32_t CF3_2 = 0;

SPI_HandleTypeDef hspi1;

/* Exported Functions ---------------------------------------------*/
/**
  * @brief System Clock Configuration
  * @retval None
  */
static void SystemClock_Config(void)
{
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
	while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
	{
	}

	/* HSI configuration and activation */
	LL_RCC_HSI_Enable();
	while(LL_RCC_HSI_IsReady() != 1)
	{
	}

	/* Main PLL configuration and activation */
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
	LL_RCC_PLL_Enable();
	LL_RCC_PLL_EnableDomain_SYS();
	while(LL_RCC_PLL_IsReady() != 1)
	{
	}

	/* Set AHB prescaler*/
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

	/* Sysclk activation on the main PLL */
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
	{
	}

	/* Set APB1 prescaler*/
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

	LL_Init1msTick(24000000);

	/* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
	LL_SetSystemCoreClock(24000000);
}
/* -------------------------------------------------------------- */
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

	/**/
	LL_GPIO_ResetOutputPin(TRIP_LED_GPIO_Port, TRIP_LED_Pin);

	/**/
	LL_GPIO_ResetOutputPin(TRIP_SIGNAL_GPIO_Port, TRIP_SIGNAL_Pin);

	/**/
	GPIO_InitStruct.Pin = TRIP_LED_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(TRIP_LED_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = TRIP_SIGNAL_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(TRIP_SIGNAL_GPIO_Port, &GPIO_InitStruct);

	// Initialize circuit breaker control pin
	GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

	LL_TIM_InitTypeDef TIM_InitStruct = {0};

	// Enable the clock for TIM2
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

	// Configure the timer
	TIM_InitStruct.Prescaler = 0; // No prescaler, timer runs at 24 MHz
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP; // Count up mode
	TIM_InitStruct.Autoreload = 0xFFFFFFFF; // Max value for 32-bit counter
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1; // No clock division
	LL_TIM_Init(TIM2, &TIM_InitStruct);

	// Disable auto-reload preload
	LL_TIM_DisableARRPreload(TIM2);

	// Set the clock source to internal
	LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);

	// Enable the counter
	LL_TIM_EnableCounter(TIM2);
}
/**
  * @brief  Initialize SPI1 for communication with ADE9039
  * @param  None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

	LL_SPI_InitTypeDef SPI_InitStruct = {0};

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Enable clocks for SPI1 and GPIOs
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

	// Configure SPI1 pins
	GPIO_InitStruct.Pin = LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7; //CS, SCK, MISO, MOSI
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configure SPI1
	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	LL_SPI_Init(SPI1, &SPI_InitStruct);
	LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);

	// Enable SPI1
	LL_SPI_Enable(SPI1);
}
/* --------------------------------------------------------------- */
/**
  * @brief  Write to a register in ADE9039
  * @param  reg: Register address
  * @param  data: Data to write
  * @retval None
  */
static void ADE9039_WriteReg(uint16_t reg, uint32_t data)
{
	uint8_t tx_buffer[6];
	tx_buffer[0] = 0x00;  // Write command
	tx_buffer[1] = (reg >> 8) & 0xFF;
	tx_buffer[2] = reg & 0xFF;
	tx_buffer[3] = (data >> 16) & 0xFF;
	tx_buffer[4] = (data >> 8) & 0xFF;
	tx_buffer[5] = data & 0xFF;

	for(int i = 0; i < 6; i++)
	{
	while(!LL_SPI_IsActiveFlag_TXE(SPI1));
	LL_SPI_TransmitData8(SPI1, tx_buffer[i]);
	}
	while(LL_SPI_IsActiveFlag_BSY(SPI1));

}
uint32_t ADE9039_ReadReg(uint16_t reg)
{
    uint8_t tx_buffer[4];
    uint8_t rx_buffer[4];
    uint32_t data = 0;

    tx_buffer[0] = 0x01; // Read command
    tx_buffer[1] = (reg >> 16) & 0xFF; // High byte of register address
    tx_buffer[2] = (reg >> 8) & 0xFF; // Middle byte of register address
    tx_buffer[3] = reg & 0xFF; // Low byte of register address

    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
    for(int i = 0; i < 4; i++)
    {
        while(!LL_SPI_IsActiveFlag_TXE(SPI1));
        LL_SPI_TransmitData8(SPI1, tx_buffer[i]);
    }
    for(int i = 0; i < 4; i++)
    {
		while(!LL_SPI_IsActiveFlag_TXE(SPI1));
		LL_SPI_TransmitData8(SPI1, 0x00); // Send dummy byte

        while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
        rx_buffer[i] = LL_SPI_ReceiveData8(SPI1);
    }
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);

    data = (rx_buffer[0] << 24) | (rx_buffer[1] << 16) | (rx_buffer[2] << 8) | rx_buffer[3];
    return data;
}
/* --------------------------------------------------------------- */
/**
  * @brief  Initialize ADE9039 metering chip
  * @param  None
  * @retval None
  */
static void ADE9039_Init(void)
{
  /* Reset the chip */
  ADE9039_WriteReg(0x0010, 0x0001); // check reset
  LL_mDelay(100);  // Wait for reset

  ADE9039_WriteReg(0x481, 0x1002);		// CONFIG1 (set this way in metrology_hal.c) sets 12 (IRQ0_ON_IRQ1) and 1 (CF3_CFG set high)
  ADE9039_WriteReg(0x480, 0x0001);		// DSP On

  ADE9039_WriteReg(0x0120, 0x0030);  // Enable CF3 pulse output
  ADE9039_WriteReg(0x0102, 0x0007);  // Set CF3 source to active power

  // Additional configurations for pulse output
  ADE9039_WriteReg(0x420, 0x00100000);  // Set WTHR
  ADE9039_WriteReg(0x421, 0x00100000);  // Set VARTHR
  ADE9039_WriteReg(0x422, 0x00100000);  // Set VATHR

  ADE9039_WriteReg(0x425, 0x00100100);  // Set CF3 calibration pulse width (256 microseconds)
  ADE9039_WriteReg(0x490, 0x0001);		// Set CFMODE to enable CF3		// ADE9039_WriteReg(0x490, 0xB180);
  ADE9039_WriteReg(0x496, 0x0010);		// Set CF3 denominator (2)
  ADE9039_WriteReg(0x4B9, 0x0918);		// Igain = 4, Vgain = 2, Rest Gain of all channels = 1
}
/* --------------------------------------------------------------- */
/**
  * @brief  Configure pulse input from ADE9039
  * @param  None
  * @retval None
  */

static void ConfigurePulseInput(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Enable GPIO clock */
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA); // Changed from GPIOC

	/* Configure GPIO pin for pulse input */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0;  /* Assuming PA0 is connected to CF3 */
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Configure EXTI for PA0 */
	LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
	EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
	EXTI_InitStruct.LineCommand = ENABLE;
	EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
	LL_EXTI_Init(&EXTI_InitStruct);

	/* EXTI_0_1 interrupt init */
	NVIC_SetPriority(EXTI0_1_IRQn, 0);
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}

static uint32_t count = 0;
static uint32_t last_timer_value = 0;
/**
  * @brief  This function handles EXTI0_1 interrupt
  * @param  None
  * @retval None
  */
void EXTI0_1_IRQHandler(void)
{
	if (LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_0) != RESET)
	{
		LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_0); // Clear the interrupt flag

		if (firstCaptured == 0)
		{
			//Captures the value of CF3 at the first pulse of the 1KHz signal
			CF3_1 = LL_TIM_GetCounter(TIM2);
			firstCaptured = 1;
		}
		else
		{
			//Captures the value of CF3 at the next pulse of the 1KHz signal
			CF3_2 = LL_TIM_GetCounter(TIM2);
			//Calculates the difference between signal pulses of CF3
			if (CF3_2 > CF3_1)
			{
				difference = CF3_2 - CF3_1;
			}
			else if (CF3_1 > CF3_2)
			{
				difference = CF3_1 - CF3_2;
			}
			firstCaptured = 0;

			uint32_t microSeconds = difference / 24;

			// Debugging output
			printf("CF3_1: %u, CF3_2: %u, Difference: %u, Time: %u\n", CF3_1, CF3_2, difference, microSeconds);
		}

		// This if-statement would inherently trip the breaker because the
		// impedance change of the CF3 channel is greater than 40
		if (difference > 10000000)
		{
			// Handle fault condition
			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6);  // Break the circuit
			while (1)
			{
				printf("TRIGGERED\n");
			} // Latch in fault state

		}
	}
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
static void Error_Handler(void)
{
	__disable_irq();
	while (1)
	{
	}
}
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
static void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	  /* User can add their own implementation to report the file name and line number,
		 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	  /* USER CODE END 6 */
}

#ifdef __cplusplus
}
#endif

#endif /*__MAIN_H*/
