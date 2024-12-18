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

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void ADE9039_Init(void);
static void ConfigurePulseInput(void);
static void ADE9039_WriteReg(uint16_t, uint32_t);
uint32_t ADE9039_ReadReg(uint16_t);

/* Private user code ---------------------------------------------------------*/

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

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();

  uint32_t data = 0;

  //ADE9039_Init();
  ADE9039_WriteReg(0x481, 0x1002);

  while (1)
  {
	//ADE9039_WriteReg(0x481, 0x1002);
	//ADE9039_Init();
	LL_mDelay(10);

    data = ADE9039_ReadReg(0x481);
    printf("Register 0x481 value: %06lX\n", data);
  }
}

/**
  * @brief  Initialize ADE9039 metering chip
  * @param  None
  * @retval None
  */
static void ADE9039_Init(void)
{
	// Reset the chip
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
static void ADE9039_WriteReg(uint16_t reg, uint32_t data)
{
	uint8_t tx_buffer[6];
	tx_buffer[0] = (reg >> 4) & 0xFF;	// Middle reg byte								// CMD_HDR
	tx_buffer[1] = (reg & 0x0F) << 4;	// Least significant 4 bits and Write command	// CMD_HDR
	tx_buffer[2] = (data >> 24) & 0xFF;
	tx_buffer[3] = (data >> 16) & 0xFF;
	tx_buffer[4] = (data >> 8) & 0xFF;
	tx_buffer[5] = data & 0xFF;

	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
	for(int i = 0; i < 6; i++)
	{
		while(!LL_SPI_IsActiveFlag_TXE(SPI1));
		LL_SPI_TransmitData8(SPI1, tx_buffer[i]);
	}
	while(LL_SPI_IsActiveFlag_BSY(SPI1));
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
}
uint32_t ADE9039_ReadReg(uint16_t reg)
{
    uint8_t tx_buffer[2];
    uint8_t rx_buffer[4];
    uint32_t data = 0;

	tx_buffer[0] = (reg >> 4) & 0xFF;			// Middle reg byte									// CMD_HDR
	tx_buffer[1] = (reg & 0x0F) << 4 | 0x08;	// Least significant reg half-byte and Read command	// CMD_HDR

    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);

	while(!LL_SPI_IsActiveFlag_TXE(SPI1));
	LL_SPI_TransmitData8(SPI1, tx_buffer[0]);
	while(!LL_SPI_IsActiveFlag_TXE(SPI1));
	LL_SPI_TransmitData8(SPI1, tx_buffer[1]);

    for(int i = 0; i < 4; i++)
    {
    	while(!LL_SPI_IsActiveFlag_TXE(SPI1));
    	LL_SPI_TransmitData8(SPI1, 0x00); // Send dummy byte
    	LL_SPI_TransmitData8(SPI1, 0x00); // Send dummy byte

        while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
        rx_buffer[i] = LL_SPI_ReceiveData8(SPI1);
    }
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);

    data = (rx_buffer[0] << 24) | (rx_buffer[1] << 16) | (rx_buffer[2] << 8) | rx_buffer[3];
    return data;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the HSI */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  /* Set APB2 prescaler */
  LL_Init1msTick(10000000);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(10000000);
}

/**
  * @brief SPI1 Initialization Function
  * @param None
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
	GPIO_InitStruct.Pin = LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7; //CS, SCK, MISO, MOSI
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configure SPI1
	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT; // 4BIT
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8; // DIV2
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	LL_SPI_Init(SPI1, &SPI_InitStruct);
	LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);

	// Enable SPI1
	LL_SPI_Enable(SPI1);

  /*
  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Peripheral clock enable
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  //SPI1 GPIO Configuration
  //PA4   ------> SPI1_NSS
  //PA5   ------> SPI1_SCK
  //PA6   ------> SPI1_MISO
  //PA7   ------> SPI1_MOSI

  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // SPI1 parameter configuration
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_4BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_EnableNSSPulseMgt(SPI1);
  */

}

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

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
