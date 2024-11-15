/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2021
 *                      All rights reserved
 *
 ***************************************************************************************************
 *  Written by:         E0523454 (Edited by E0658104)
 *                      Eaton Electrical
 *                      1000 Cherrington Parkway
 *                      Pittsburgh, PA  15108-4312
 *                      (412) 893-3300
 *//**
 *
 * @brief hardware initialization functions specific to building the Smart Breaker 2.0 2-pole GFCI
 * breaker firmware
 *
 * this file will be copy to sys/hardware_init.c whenever the sb2-0_2p_gfci make target is invoked
 *
 * @file sb2-0_2p_gfci_hardware_init.c
 * @ingroup hardware_init
 *
 *
 * TODO JF: Modify this file to match SB2.0 pinout.
 *
 **************************************************************************************************/
#include "main_internal.h"
#include "types.h"
#include "adc_.h"
#include "gpio_.h"
#include "spi_.h"
#include "firmware.config"

#define UNUSED(_pin, _port) GPIO_InitStruct.Pin = (_pin);\
                            GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;\
                            GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;\
                            LL_GPIO_Init((_port), &GPIO_InitStruct)

void gpio_init_component(void)
{

    LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOD);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);

    /**/
//    LL_GPIO_ResetOutputPin(LED_1_GPIO_Port, LED_1_Pin);//Nitin

    /**/
    LL_GPIO_ResetOutputPin(GF_OFFSET_GPIO_Port, GF_OFFSET_Pin);

    /**/
    LL_GPIO_ResetOutputPin(TEST_GF_GPIO_Port, TEST_GF_Pin);

    /**/
    LL_GPIO_ResetOutputPin(GN_GPIO_Port, GN_Pin);

    /**/
    LL_GPIO_ResetOutputPin(TRIP_2_GPIO_Port, TRIP_2_Pin);

    /**/
    LL_GPIO_ResetOutputPin(TRIP_1_GPIO_Port, TRIP_1_Pin);

    /**/
    LL_GPIO_ResetOutputPin(SS_CLOSE_EN_GPIO_Port, SS_CLOSE_EN_Pin);

    /**/
    LL_GPIO_ResetOutputPin(SS_OPEN_EN_GPIO_Port, SS_OPEN_EN_Pin);

    /**/
    LL_GPIO_ResetOutputPin(RADIO_EN_GPIO_Port, RADIO_EN_Pin);

    /**/
    LL_GPIO_ResetOutputPin(PWRSENSE_GPIO_Port, PWRSENSE_Pin);

#if defined LED_ACTIVE_LOW
    LL_GPIO_SetOutputPin(LED_Green_GPIO_Port, LED_Green_Pin);
        LL_GPIO_SetOutputPin(LED_Blue_GPIO_Port, LED_Blue_Pin);
        LL_GPIO_SetOutputPin(LED_Red_GPIO_Port, LED_Red_Pin);
        LL_GPIO_SetOutputPin(LED_LOAD_GPIO_Port, LED_LOAD_Pin);
#else
    /**/
    LL_GPIO_ResetOutputPin(LED_Red_GPIO_Port, LED_Red_Pin);
    /**/
    LL_GPIO_ResetOutputPin(LED_Green_GPIO_Port, LED_Green_Pin);
    /**/
    LL_GPIO_ResetOutputPin(LED_Blue_GPIO_Port, LED_Blue_Pin);
#ifdef SB20_PUL
    LL_GPIO_ResetOutputPin(LED_Green_LOAD_GPIO_Port, LED_Green_LOAD_Pin);
    LL_GPIO_ResetOutputPin(LED_Red_LOAD_GPIO_Port, LED_Red_LOAD_Pin);
    LL_GPIO_ResetOutputPin(LED_Blue_LOAD_GPIO_Port, LED_Blue_LOAD_Pin);
#else
    LL_GPIO_ResetOutputPin(LED_LOAD_GPIO_Port, LED_LOAD_Pin);
#endif
#endif
    /**/
    /*TODO START HERE*/
    GPIO_InitStruct.Pin = LED_Green_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED_Green_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LED_Blue_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED_Blue_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LED_Red_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED_Red_GPIO_Port, &GPIO_InitStruct);
#ifdef SB20_PUL
    GPIO_InitStruct.Pin = LED_Green_LOAD_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED_Green_LOAD_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LED_Red_LOAD_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED_Red_LOAD_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LED_Blue_LOAD_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED_Blue_LOAD_GPIO_Port, &GPIO_InitStruct);
#else
    GPIO_InitStruct.Pin = LED_LOAD_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED_LOAD_GPIO_Port, &GPIO_InitStruct);
#endif

    /**/
    GPIO_InitStruct.Pin = BTN_1_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(BTN_1_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = BTN_2_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(BTN_2_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = GF_OFFSET_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GF_OFFSET_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = TEST_GF_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(TEST_GF_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = GN_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GN_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = TRIP_2_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(TRIP_2_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = TRIP_1_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(TRIP_1_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = HAL_WATCHDOG_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(HAL_WATCHDOG_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = PWRSENSE_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(PWRSENSE_GPIO_Port, &GPIO_InitStruct);

    /*SB*/
    GPIO_InitStruct.Pin = OPEN_FDBK_1_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(OPEN_FDBK_1_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = OPEN_FDBK_2_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(OPEN_FDBK_2_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = SS_CLOSE_EN_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(SS_CLOSE_EN_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = SS_OPEN_EN_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(SS_OPEN_EN_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = RADIO_EN_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(RADIO_EN_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = PWRSENSE_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(PWRSENSE_GPIO_Port, &GPIO_InitStruct);


#if DEBUG_SYSTEM_TIMING
    GPIO_InitStruct.Pin = DATA_COLLECT_TIMING_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(DATA_COLLECT_TIMING_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ZCD_TIMING_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(ZCD_TIMING_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MAIN_TIMING_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(MAIN_TIMING_GPIO_Port, &GPIO_InitStruct);
#else

#endif

    /****************************************************
    *               configure unused pins
    ****************************************************/
    /**/
    UNUSED(EV_RSV_GPIO1_Pin, EV_RSV_GPIO1_GPIO_Port);
    UNUSED(EV_RSV_GPIO2_Pin, EV_RSV_GPIO2_GPIO_Port);
    UNUSED(EV_RSV_GPIO3_Pin, EV_RSV_GPIO3_GPIO_Port);
#ifdef SB20_PUL
#else
        UNUSED(UNUSED_PA1_Pin, UNUSED_PA1_GPIO_Port);
        UNUSED(UNUSED_PB3_Pin, UNUSED_PB3_GPIO_Port);
#endif
    UNUSED(UNUSED_PA10_Pin, UNUSED_PA10_GPIO_Port);
    UNUSED(UNUSED_PF0_Pin, UNUSED_PF0_GPIO_Port);
    UNUSED(UNUSED_PF1_Pin, UNUSED_PF1_GPIO_Port);

    /**/
    LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTB, LL_EXTI_CONFIG_LINE5);

    /**/
    LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTB, LL_EXTI_CONFIG_LINE4);

    /**/
    EXTI_InitStruct.Line_0_31 = ZCD_2_EXTI_LINE;
    EXTI_InitStruct.LineCommand = DISABLE;          /* ZCD2 disabled at startup to only try to catch ZCD1 */
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
    LL_EXTI_Init(&EXTI_InitStruct);

    /**/
    EXTI_InitStruct.Line_0_31 = ZCD_1_EXTI_LINE;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
    LL_EXTI_Init(&EXTI_InitStruct);

    /**/
    //EXTI_InitStruct.Line_0_31 = ZCD_2_EXTI_LINE;
    //EXTI_InitStruct.LineCommand = ENABLE;
    //EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    //EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
    //LL_EXTI_Init(&EXTI_InitStruct);

    /**/
    //EXTI_InitStruct.Line_0_31 = ZCD_1_EXTI_LINE;
    //EXTI_InitStruct.LineCommand = ENABLE;
    //EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    //EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
    //LL_EXTI_Init(&EXTI_InitStruct);

    /**/
    LL_GPIO_SetPinPull(ZCD_2_GPIO_Port, ZCD_2_Pin, LL_GPIO_PULL_NO);

    /**/
    LL_GPIO_SetPinPull(ZCD_1_GPIO_Port, ZCD_1_Pin, LL_GPIO_PULL_NO);

    /**/
    LL_GPIO_SetPinMode(ZCD_2_GPIO_Port, ZCD_2_Pin, LL_GPIO_MODE_INPUT);

    /**/
    LL_GPIO_SetPinMode(ZCD_1_GPIO_Port, ZCD_1_Pin, LL_GPIO_MODE_INPUT);

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

    GPIO_InitStruct.Pin = V_PEAK_1_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(V_PEAK_1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = V_PEAK_2_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(V_PEAK_2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GF_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GF_GPIO_Port, &GPIO_InitStruct);

    //GPIO_InitStruct.Pin = LOAD_CURR_1_Pin;
    //GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    //GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    //LL_GPIO_Init(LOAD_CURR_1_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = STARTUP_CAP_CONF_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(STARTUP_CAP_CONF_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LOAD_CURR_2_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LOAD_CURR_2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = _11V_MON_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(_11V_MON_GPIO_Port, &GPIO_InitStruct);

#ifdef SB20_PUL
#else
    GPIO_InitStruct.Pin = THERM_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(THERM_GPIO_Port, &GPIO_InitStruct);
#endif

    GPIO_InitStruct.Pin = HALL_IN_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(HALL_IN_GPIO_Port, &GPIO_InitStruct);

#ifdef SB20_PUL
    GPIO_InitStruct.Pin = SOL_ADC_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(SOL_ADC_GPIO_Port, &GPIO_InitStruct);
#endif
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
    adc_enable_internal_regulator();

    /** Configure channels */
    LL_ADC_REG_SetSequencerChannels(ADC1,GF_ADC_CHANNEL       |
                                         //LOAD_CURR_1_ADC_CHANNEL |
    								 STARTUP_CAP_CONF_CHANNEL |
									  LOAD_CURR_2_ADC_CHANNEL |
                                         _11V_MON_ADC_CHANNEL |
                                         THER_ADC_CHANNEL     |
#ifdef SB20_PUL
                                         HALL_IN_ADC_CHANNEL  |
                                         SOL_ADC_CHANNEL |
                                         LL_ADC_CHANNEL_TEMPSENSOR |
                                         LL_ADC_CHANNEL_VREFINT);
#else
                                         HALL_IN_ADC_CHANNEL);
#endif
    return;
}

void reconfigure_cap_config_output(void)
{
	LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    /**/
    GPIO_InitStruct.Pin = STARTUP_CAP_CONF_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(STARTUP_CAP_CONF_GPIO_Port, &GPIO_InitStruct);
}


void spi_init_component(void)
{
    LL_SPI_InitTypeDef SPI_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Peripheral clock enable
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

    // Set chip select high
    LL_GPIO_SetOutputPin(SPI_CS_GPIO_Port, SPI_CS_Pin);

    // settup the chip select pin as a standard GPIO
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

    // set threshold to a quarter (1 byte)
    LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);

    return;
}


