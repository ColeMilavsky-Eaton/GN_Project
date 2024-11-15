#ifndef _ADC_API_H
#define _ADC_API_H
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
 * @brief External interface for the adc component.
 *
 * @file adc_api.h
 * @ingroup adc
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "hardware.pin_config"

#define MAX_ADC_VALUE 4095

/**************************************************************************************************/
/**
 * @brief Initialize configuration for ADC
 *
 * This function initializes configuration for ADC
 *
 *
 * @return none
 *
 * @exception none
 *
 * @requirement{2009101500}
 *
 **************************************************************************************************/
void adc_init_component(void);


/**************************************************************************************************/
/**
 * @brief ADC sequence conversion
 *
 * This function performs a conversion on a sequence of channels.
 *
 *
 * @param[in]  channel The a bitmask representing the adc channels to read.
 *
 * @return status STATUS_OK if conversion is successful, STATUS_FAIL otherwise
 *
 * @exception none
 *
 * @requirement{2009101501}
 *
 **************************************************************************************************/
status_t adc_read_sequence(u32 channels);

/**************************************************************************************************/
/**
 * @brief ADC sequence conversion
 *
 * This function performs a conversion on a pacific temp sensor channels.
 *
 *
 * @param[in]  channel The a bitmask representing the two internal adc channels to read..
 *
 * @return status STATUS_OK if conversion is successful, STATUS_FAIL otherwise
 *
 * @exception none
 *
 **************************************************************************************************/
status_t adc_read_internal_ch(u32 channels);

/**************************************************************************************************/
/**
 * @brief Get the data for a specific channel.
 *
 * Get adc data for the specific channel.
 *
 * @param[in]    channel channel input.
 * @param[out]  *data location to store channel data
 *
 *
 * @return status STATUS_FAIL if more than 1 channel is selected, STATUS_OK otherwise
 *
 * @exception none
 *
 *
 **************************************************************************************************/
u16 adc_get_channel_data(u32 channel);


/**************************************************************************************************/
/**
 * @brief ADC1 calibration
 *
 * This function calibrate ADC1.
 *
 * @param[in]
 *
 * @@return status STATUS_OK if calibration is successful, STATUS_FAIL otherwise
 *
 * @exception none
 *
 *
 **************************************************************************************************/
void adc_calibration(void);


/**************************************************************************************************/
/**
 * @brief enable ADC1 internal regulator
 *
 * This function enables internal regulator for ADC1
 *
 * @param[in]
 *
 * @@return status STATUS_OK if calibration is successful, STATUS_FAIL otherwise
 *
 * @exception none
 *
 *
 **************************************************************************************************/
void adc_enable_internal_regulator(void);

#endif /*_ADC_API_H */

