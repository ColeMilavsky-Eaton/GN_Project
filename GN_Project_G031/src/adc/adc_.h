#ifndef _ADC__H
#define _ADC__H
/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 **************************************************************************************************
 *  Written by:         Hank Sun
 *                      Eaton Electrical
 *                      1000 Cherrington Parkway
 *                      Pittsburgh, PA  15108-4312
 *                      (412) 893-3300
 *//**
 * @brief Internal (to the component) interface for the adc component.
 *
 * @file adc_.h
 * @ingroup adc
 *
 *//*
 *
 **************************************************************************************************/
#include "adc_internal.h"
#include "stm32g0xx_ll_adc.h"
#include "stm32g0xx.h"

/** @adc1_status macros related to status of ADC
 * @{ */
#define ADC1_ENABLED                 (LL_ADC_IsEnabled(ADC1) == 1)
#define ADC1_NOT_ENABLED             (LL_ADC_IsEnabled(ADC1) == 0)
#define ADC1_CONVERSION_ONGOING      (LL_ADC_REG_IsConversionOngoing(ADC1) == 1)
#define ADC1_CONVERSION_NOT_ONGOING  (LL_ADC_REG_IsConversionOngoing(ADC1) == 0)
#define ADC1_READY                   (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 1)
#define ADC1_NOT_READY               (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
#define ADC1_CONVERSION_NOT_COMPLETE (LL_ADC_IsActiveFlag_EOC(ADC1) == 0)

/** @} */

#define ADC_TOTAL_NUMBER_OF_CHANNELS (19)

/** @timeouts macros related to timeout periods used in component
 * @{ */
#define ADC_WAIT_FOR_READY_TIMEOUT          (100)
#define ADC_WAIT_FOR_CONVERTION_TIMEOUT     (100)
#define ADC_SET_TIMEOUT(_x)                 {adc_timeout_timer = (_x) ;}
#define ADC_TIMEOUT_EXPIRED                 ((--adc_timeout_timer) <= 0)

/** @} */

/** delay for regulator turn on */
#define  INTERNAL_REGULATOR_STABILIZATION_TIME_IN_COUNTS ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10)

/**************************************************************************************************/
/**
 * @brief Get number of channels.
 *
 * Given the channel input, calculate how many channels are selected.
 * For example, LL_ADC_CHANNEL_4|LL_ADC_CHANNEL_8 would return 2.
 *
 *
 * @param[in]  channel channel input.
 *
 *
 * @return signed number that represent the channel number. If -1 is returned means no channel
 *         found or more than one channel input.
 *
 * @exception none
 *
 *
 **************************************************************************************************/
u8 adc_get_num_channels(u32 bitmask);

/**************************************************************************************************/
/**
 * @brief Find the channel number base on channel input.
 *
 * Given the channel input, calculate the number of the channel.
 * For example, LL_ADC_CHANNEL_4 would return 4,
 *
 * NOTE: If more than one channel input or no channel input, both would return -1.
 *
 * @param[in]  channel channel input.
 *
 *
 * @return signed number that represent the channel number. If -1 is returned means no channel
 *         found or more than one channel input.
 *
 * @exception none
 *
 *
 **************************************************************************************************/
s8 adc_find_channel_number(u32 channel);

#endif /*_ ADC__H */
