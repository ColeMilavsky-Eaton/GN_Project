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
static float frequency = 0;
static uint32_t lastCaptureValue = 0;
static int newFrequencyAvailable = 0;

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

	LL_Init1msTick(64000000);

	/* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
	LL_SetSystemCoreClock(64000000);
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
/* --------------------------------------------------------------- */
/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
	LL_TIM_InitTypeDef TIM_InitStruct = {0};

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	/**TIM3 GPIO Configuration
	PA6   ------> TIM3_CH1
	*/
	GPIO_InitStruct.Pin = CF4_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(CF4_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	TIM_InitStruct.Prescaler = 0;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 65535;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(TIM3, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM3);
	LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_TI1FP1);
	LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_EXT_MODE1);
	LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
	LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
	LL_TIM_DisableIT_TRIG(TIM3);
	LL_TIM_DisableDMAReq_TRIG(TIM3);
	LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM3);
	LL_TIM_EnableCounter(TIM3);
}
/* --------------------------------------------------------------- */
/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{
	LL_TIM_InitTypeDef TIM_InitStruct = {0};
	LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM14);

	TIM_InitStruct.Prescaler = 32;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 1000;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(TIM14, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM14);
	TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_TOGGLE;
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.CompareValue = 1000;
	TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	LL_TIM_OC_Init(TIM14, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM14, LL_TIM_CHANNEL_CH1);
	/* USER CODE BEGIN TIM14_Init 2 */
	LL_TIM_EnableCounter(TIM14); //Starts Timer
	LL_TIM_CC_EnableChannel(TIM14, LL_TIM_CHANNEL_CH1); //Enables OC on Channel 1
	/* USER CODE END TIM14_Init 2 */
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	/**TIM14 GPIO Configuration
	PA4   ------> TIM14_CH1
	*/
	GPIO_InitStruct.Pin = PWM_1KHz_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
	LL_GPIO_Init(PWM_1KHz_GPIO_Port, &GPIO_InitStruct);
}
/* --------------------------------------------------------------- */
/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
	LL_TIM_InitTypeDef TIM_InitStruct = {0};
	LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
	LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	LL_RCC_SetTIMClockSource(LL_RCC_TIM1_CLKSOURCE_PCLK1);

	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

	/* TIM1 interrupt Init */
	NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 0);
	NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);

	TIM_InitStruct.Prescaler = 65535;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 1024;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	TIM_InitStruct.RepetitionCounter = 0;
	LL_TIM_Init(TIM1, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM1);
	LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
	TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.CompareValue = 512;
	TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
	TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
	TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
	LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM1);
	TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
	TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
	TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
	TIM_BDTRInitStruct.DeadTime = 0;
	TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
	TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
	TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
	TIM_BDTRInitStruct.BreakAFMode = LL_TIM_BREAK_AFMODE_INPUT;
	TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_DISABLE;
	TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
	TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
	TIM_BDTRInitStruct.Break2AFMode = LL_TIM_BREAK_AFMODE_INPUT;
	TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
	LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
	/* USER CODE BEGIN TIM1_Init 2 */
	LL_TIM_EnableCounter(TIM1); //Starts the timer
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1); //Enables Channel for PWM Generation
	LL_TIM_EnableIT_UPDATE(TIM1); //Enables the update interrupt (UIE) for a specific timer.
	/* USER CODE END TIM1_Init 2 */
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	/**TIM1 GPIO Configuration
	PA8   ------> TIM1_CH1
	*/
	GPIO_InitStruct.Pin = PWM_1Hz_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
	LL_GPIO_Init(PWM_1Hz_GPIO_Port, &GPIO_InitStruct);
}
/* --------------------------------------------------------------- */
/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
	LL_TIM_InitTypeDef TIM_InitStruct = {0};

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	/**TIM2 GPIO Configuration
	PA0   ------> TIM2_CH1
	*/
	GPIO_InitStruct.Pin = CF3_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
	LL_GPIO_Init(CF3_GPIO_Port, &GPIO_InitStruct);

	TIM_InitStruct.Prescaler = 0;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 4294967295;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(TIM2, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM2);
	LL_TIM_SetTriggerInput(TIM2, LL_TIM_TS_TI1FP1);
	LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_EXT_MODE1);
	LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
	LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
	LL_TIM_DisableIT_TRIG(TIM2);
	LL_TIM_DisableDMAReq_TRIG(TIM2);
	LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM2);
	/* USER CODE BEGIN TIM2_Init 2 */
	LL_TIM_EnableCounter(TIM2);
	/* USER CODE END TIM2_Init 2 */
}
/* --------------------------------------------------------------- */
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
/* --------------------------------------------------------------- */
static void startTIM14(void)
{
	//Set the CEN bit to start the timer
	TIM14->CR1 |= TIM_CR1_CEN;
}
/* --------------------------------------------------------------- */
static void startTIM2(void)
{
	//Set the CEN bit to start the timer
	TIM2->CR1 |= TIM_CR1_CEN;
}
/* --------------------------------------------------------------- */
static void startTIM3(void)
{
	//Set the CEN bit to start the timer
	TIM3->CR1 |= TIM_CR1_CEN;
}
/* --------------------------------------------------------------- */
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
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

	// Configure SPI1 pins
	GPIO_InitStruct.Pin = LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7; //SCK, MISO, MOSI
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
	tx_buffer[0] = 0x00;  /* Write command */
	tx_buffer[1] = (reg >> 8) & 0xFF;
	tx_buffer[2] = reg & 0xFF;
	tx_buffer[3] = (data >> 16) & 0xFF;
	tx_buffer[4] = (data >> 8) & 0xFF;
	tx_buffer[5] = data & 0xFF;

	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);  /* CS low */
	for(int i = 0; i < 6; i++)
	{
	while(!LL_SPI_IsActiveFlag_TXE(SPI1));
	LL_SPI_TransmitData8(SPI1, tx_buffer[i]);
	}
	while(LL_SPI_IsActiveFlag_BSY(SPI1));
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);  /* CS high */
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
  ADE9039_WriteReg(0x0010, 0x0001);
  LL_mDelay(100);  /* Wait for reset */

  /* Configure for specific needs */
  ADE9039_WriteReg(0x0120, 0x0030);  /* Enable CF3 pulse output */
  ADE9039_WriteReg(0x0102, 0x0007);  /* Set CF3 source to active power */

  // Additional configurations for pulse output
  ADE9039_WriteReg(0x420, 0x00100000);  // Set WTHR
  ADE9039_WriteReg(0x421, 0x00100000);  // Set VARTHR
  ADE9039_WriteReg(0x422, 0x00100000);  // Set VATHR
  ADE9039_WriteReg(0x425, 0x00000000);  // Set CF calibration pulse width (adjust as needed)
  ADE9039_WriteReg(0x490, 0x0001);       // Set CFMODE to enable CF3
  ADE9039_WriteReg(0x496, 0xFFFF);       // Set CF3 denominator (adjust as needed)
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
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);

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

	/* Enable EXTI4_15 interrupt */
	NVIC_EnableIRQ(EXTI4_15_IRQn);
}
/* --------------------------------------------------------------- */
/**
  * @brief  This function handles EXTI4_15 interrupt
  * @param  None
  * @retval None
  */
static void EXTI4_15_IRQHandler(void)
{
	if (LL_EXTI_ReadFallingFlag_0_31(LL_EXTI_LINE_6))
	{
		LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_6);

		uint32_t currentCapture = LL_TIM_GetCounter(TIM2);
		uint32_t diffCapture;

		if (currentCapture > lastCaptureValue)
		{
		  diffCapture = currentCapture - lastCaptureValue;
		}
		else
		{
		  diffCapture = (0xFFFFFFFF - lastCaptureValue) + currentCapture + 1;
		}

		frequency = (float)SystemCoreClock / (float)diffCapture;
		lastCaptureValue = currentCapture;
		newFrequencyAvailable = 1;
	}
}
/* --------------------------------------------------------------- */
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
