/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2021
 *                      All rights reserved
 *
 ***************************************************************************************************
 *  Written by:         E0523454
 *                      Eaton Electrical
 *                      1000 Cherrington Parkway
 *                      Pittsburgh, PA  15108-4312
 *                      (412) 893-3300
 *//**
 * @defgroup hardware_init hardware init functions
 *
 * @brief hardware initialization functions specific to building the BR 1-pole AFCI breaker firmware
 *
 * this file will be copy to sys/hardware_init.c whenever the br_1p_afci make target is invoked
 *
 * @file br_1p_afci_hardware_init.c
 * @ingroup hardware_init
 *
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "main_internal.h"
#include "stm32g071xx.h"

#define UNUSED(_pin, _port) GPIO_InitStruct.Pin = (_pin);\
                            GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;\
                            GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;\
                            LL_GPIO_Init((_port), &GPIO_InitStruct)

void gpio_init_component(void)
{
    LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

    /**/
    LL_GPIO_ResetOutputPin(TRIP_1_GPIO_Port, TRIP_1_Pin);

    /**/
    LL_GPIO_ResetOutputPin(TRIP_2_GPIO_Port, TRIP_2_Pin);

    /**/
    LL_GPIO_SetOutputPin(REGULATE_GPIO_Port, REGULATE_Pin);

    /**/
    LL_GPIO_SetOutputPin(LOG_ENABLE_GPIO_Port, LOG_ENABLE_Pin);

    /**/
    LL_GPIO_SetOutputPin(LED_1_GPIO_Port, LED_1_Pin);

    /**/
    LL_GPIO_SetOutputPin(BTN_1_GPIO_Port, BTN_1_Pin);

    /**/
    LL_GPIO_ResetOutputPin(LOAD_CURRENT_BIAS_GPIO_Port, LOAD_CURRENT_BIAS_Pin);

    /**/
    LL_GPIO_ResetOutputPin(TEST_HF_GPIO_Port, TEST_HF_Pin);

    /**/
    LL_GPIO_ResetOutputPin(TEST_CURRENT_GPIO_Port, TEST_CURRENT_Pin);


    /****************************************************
    *               configure GPIO pins
    ****************************************************/

    /** This pin will be reconfigured to actual spi MISO when diagnostic is enabled or breaker
     * starts pin tool spi data transfer.
     */

    LL_GPIO_ResetOutputPin(SPI_MISO_GPIO_Port, SPI_MISO_Pin);
    GPIO_InitStruct.Pin = SPI_MISO_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(SPI_MISO_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = TRIP_1_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(TRIP_1_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = TRIP_2_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(TRIP_2_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = REGULATE_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(REGULATE_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = LOG_ENABLE_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LOG_ENABLE_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = LED_1_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(BTN_1_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = BTN_1_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(BTN_1_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = LOAD_CURRENT_BIAS_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LOAD_CURRENT_BIAS_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = TEST_HF_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(TEST_HF_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = TEST_CURRENT_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(TEST_CURRENT_GPIO_Port, &GPIO_InitStruct);


    /****************************************************
    *               configure unused pins
    ****************************************************/
    UNUSED(UNUSED_PA4_Pin, UNUSED_PA4_GPIO_Port);
    UNUSED(UNUSED_PA7_Pin, UNUSED_PA7_GPIO_Port);
    UNUSED(UNUSED_PA10_Pin, UNUSED_PA10_GPIO_Port);
    UNUSED(UNUSED_PA15_Pin, UNUSED_PA15_GPIO_Port);

    UNUSED(UNUSED_PB2_Pin, UNUSED_PB2_GPIO_Port);
    UNUSED(UNUSED_PB8_Pin, UNUSED_PB8_GPIO_Port);

    UNUSED(UNUSED_PC14_Pin, UNUSED_PC14_GPIO_Port);


    /****************************************************
    *           configure external interrupts
    ****************************************************/
    LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTA, LL_EXTI_CONFIG_LINE5);

    /**/
    EXTI_InitStruct.Line_0_31 = ZCD_1_EXTI_LINE;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
    LL_EXTI_Init(&EXTI_InitStruct);

    /**/
    LL_GPIO_SetPinPull(ZCD_1_GPIO_Port, ZCD_1_Pin, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinMode(ZCD_1_GPIO_Port, ZCD_1_Pin, LL_GPIO_MODE_INPUT);

    return;
}

void adc_init_component(void)
{
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
    LL_ADC_InitTypeDef ADC_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC);

    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

    GPIO_InitStruct.Pin = SHUNT_TEMPERATURE_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(SHUNT_TEMPERATURE_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = HF_MIN_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(HF_MIN_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = VLOAD_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(VLOAD_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LOAD_CURRENT_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LOAD_CURRENT_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = HF_SENSE_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(HF_SENSE_GPIO_Port, &GPIO_InitStruct);


    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
    ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
    ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
    LL_ADC_Init(ADC1, &ADC_InitStruct);
    LL_ADC_REG_SetSequencerConfigurable(ADC1, LL_ADC_REG_SEQ_FIXED);
    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
    ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_1RANK;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
    ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
    ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
    LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
    LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
    LL_ADC_SetTriggerFrequencyMode(ADC1, LL_ADC_CLOCK_FREQ_MODE_HIGH);
    LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1, LL_ADC_SAMPLINGTIME_1CYCLE_5);
    LL_ADC_DisableIT_EOC(ADC1);
    LL_ADC_DisableIT_EOS(ADC1);

    /** Configure channels
    */
    LL_ADC_REG_SetSequencerChannels(ADC1,SHUNT_TEMPERATURE_CHANNEL |
                                         HF_MIN_ADC_CHANNEL |
                                         V_PEAK_1_ADC_CHANNEL |
                                         HF_SENSE_CHANNEL |
                                         LOAD_CURRENT_CHANNEL);

    return;
}

void spi_init_component(void)
{
  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  /* Set chip select high */
  LL_GPIO_SetOutputPin(SPI_CS_GPIO_Port, SPI_CS_Pin);

  /* set up the chip select pin as a standard GPIO */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SPI_MISO_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(SPI_MISO_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SPI_MOSI_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(SPI_MOSI_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SPI_CLK_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(SPI_CLK_GPIO_Port, &GPIO_InitStruct);

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI1);

  /* set threshold to a quarter (1 byte) */
  LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);

  return;
}


