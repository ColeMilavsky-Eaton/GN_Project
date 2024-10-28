/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 ***************************************************************************************************
 *  Written by:         Hank Sun
 *                      Eaton Electrical
 *                      1000 Cherrington Parkway
 *                      Pittsburgh, PA  15108-4312
 *                      (412) 893-3300
 *//**
 * @defgroup adc adc component
 *
 * @brief The adc component handles the initialization of adc and performing a single analog to
 * digital conversion on a single channel.
 *
 *
 * @file adc.c
 * @ingroup adc
 *
 *
 *//*
 *
 **************************************************************************************************/
#include "adc_.h"
#include "main_internal.h"
#include "types.h"
#include "string.h"

u16 adc_channel_data[ADC_TOTAL_NUMBER_OF_CHANNELS];

/* used for timeout calculation */
volatile s16 adc_timeout_timer;


void adc_calibration(void)
{
    if(LL_ADC_IsEnabled(ADC1) == 1)
    {
        LL_ADC_Disable(ADC1);
    }

    adc_enable_internal_regulator();

    LL_ADC_StartCalibration(ADC1);
    while(LL_ADC_IsCalibrationOnGoing(ADC1));

    return;
}

void adc_enable_internal_regulator(void)
{
    volatile u32 wait_loop_index;
    if(LL_ADC_IsInternalRegulatorEnabled(ADC1) == 0)
    {
        /* Enable ADC internal voltage regulator */
        LL_ADC_EnableInternalRegulator(ADC1);
        /* Delay for ADC internal voltage regulator stabilization. */
        wait_loop_index = INTERNAL_REGULATOR_STABILIZATION_TIME_IN_COUNTS;
        while(wait_loop_index != 0)
        {
            wait_loop_index--;
        }
    }
    return;
}


status_t adc_read_sequence(u32 channels)
{
    u8 num_channels = adc_get_num_channels(channels);
    u8 index = 0;

    /* Set channel to convert */
    LL_ADC_REG_SetSequencerChannels(ADC1, channels);

    channels = channels & ADC_CHANNEL_ID_BITFIELD_MASK;

    /* Enable ADC1 */
    LL_ADC_Enable(ADC1);

    while(num_channels > 0)
    {
        /* set timeout */
        ADC_SET_TIMEOUT(ADC_WAIT_FOR_READY_TIMEOUT);

        while(ADC1_NOT_ENABLED || ADC1_CONVERSION_ONGOING || ADC1_NOT_READY)
        {
            if(ADC_TIMEOUT_EXPIRED)
            {
                return STATUS_FAIL;
            }
        }

        /* start conversion */
        LL_ADC_REG_StartConversion(ADC1);

        /* reset the counter/timer */
        ADC_SET_TIMEOUT(ADC_WAIT_FOR_CONVERTION_TIMEOUT);

        /* Wait for conversion to complete */
        while(ADC1_CONVERSION_NOT_COMPLETE)
        {
            if(ADC_TIMEOUT_EXPIRED)
            {
                return STATUS_FAIL;
            }
        }

        while( (channels & 0x01) == 0)
        {
            channels = channels>>1;
            index++;
        }
        /* Collect converted data */
        adc_channel_data[index] = LL_ADC_REG_ReadConversionData12(ADC1);

        num_channels--;
        channels = channels>>1;
        index++;
    }

    /* Disable ADC1 */
    LL_ADC_Disable(ADC1);

    /* Need to manually clear ADRDY register after disabling adc. */
    LL_ADC_ClearFlag_ADRDY(ADC1);

    /* return */
    return STATUS_OK;
}

status_t adc_read_internal_ch(u32 channels)
{
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
            LL_ADC_PATH_INTERNAL_VREFINT | LL_ADC_PATH_INTERNAL_TEMPSENSOR);

    if(channels != LL_ADC_CHANNEL_VREFINT && channels != LL_ADC_CHANNEL_TEMPSENSOR)
    {
        return STATUS_FAIL;
    }
     LL_ADC_REG_SetSequencerChannels(ADC1, channels);
     if(channels == LL_ADC_CHANNEL_TEMPSENSOR)
     {
        // set 19.5 cycles for tempsensor channel
        LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1, LL_ADC_SAMPLINGTIME_19CYCLES_5);
     }
    /* Enable ADC1 */
    LL_ADC_Enable(ADC1);
    /* set timeout */
    ADC_SET_TIMEOUT(ADC_WAIT_FOR_READY_TIMEOUT);

    while(ADC1_NOT_ENABLED || ADC1_CONVERSION_ONGOING || ADC1_NOT_READY)
    {
        if(ADC_TIMEOUT_EXPIRED)
        {
            return STATUS_FAIL;
        }
    }

    /* start conversion */
    LL_ADC_REG_StartConversion(ADC1);

    /* reset the counter/timer */
    ADC_SET_TIMEOUT(ADC_WAIT_FOR_CONVERTION_TIMEOUT);

    /* Wait for conversion to complete */
    while(ADC1_CONVERSION_NOT_COMPLETE)
    {
        if(ADC_TIMEOUT_EXPIRED)
        {
            return STATUS_FAIL;
        }
    }

    if(channels == LL_ADC_CHANNEL_TEMPSENSOR)
    {
        /* internal temperature sensor is channel 12*/
        adc_channel_data[12] = LL_ADC_REG_ReadConversionData12(ADC1);
    }
    else
    {
        adc_channel_data[13] = LL_ADC_REG_ReadConversionData12(ADC1);
    }

    /* Disable ADC1 */
    LL_ADC_Disable(ADC1);

    /* Need to manually clear ADRDY register after disabling adc. */
    LL_ADC_ClearFlag_ADRDY(ADC1);
    // set 1.5 cycles for other channels
    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1, LL_ADC_SAMPLINGTIME_1CYCLE_5);
    // power down internal channels
    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_NONE );
    /* return */
    return STATUS_OK;
}

u16 adc_get_channel_data(u32 channel)
{
    return adc_channel_data[(channel & ADC_CHANNEL_ID_NUMBER_MASK) >> ADC_CHANNEL_ID_NUMBER_BITOFFSET_POS];
}


u8 adc_get_num_channels(u32 channels)
{
    u8 count = 0;

    channels = channels & ADC_CHANNEL_ID_BITFIELD_MASK;

    for( count = 0; channels; count++)
    {
        channels &= channels - 1;
    }

    return count;
}

