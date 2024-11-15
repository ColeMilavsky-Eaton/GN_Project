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
 *
 * @brief Checks whether if there is a fault in the high frequency sensing circuit.
 *
 * A routine in the background calculates the number that represents the deviation of the analog
 * signal (hf_sense) from its average value. The purpose of this is to check the 0.01uF film
 * capacitor in the high frequency sensing circuit. If the capacitor is faulted, there
 * will be no deviation from the average of hf_sense signal.
 *
 *
 *
 * @file hf_sense.c
 * @ingroup self_test
 *
 *
 *//*
 *
 **************************************************************************************************/
#include "self_test_.h"
#include "types.h"
#include "utils.h"

u32 hf_sense_integral;
u32 hf_sense_average;
u32 hf_sense_test;
u32 hf_sense_test_previous;
u16 hf_sense_error_counter;
u32* hf_sense_data_p;
self_test_flag_t hf_sense_fault_flag;

self_test_flag_t hf_sense_stuck_high_flag;

void hf_sense_data_collect_task(u32 it)
{
    hf_sense_integral += *hf_sense_data_p;

    if(*hf_sense_data_p > hf_sense_average)
    {
        hf_sense_test += (*hf_sense_data_p - hf_sense_average);
    }
    else
    {
        hf_sense_test += (hf_sense_average - *hf_sense_data_p);
    }

    return;
}


void hf_sense_zcd_callback(u32 it)
{
    /* get hf sense previous half cycle average */
    hf_sense_average = hf_sense_integral >> 4;

    /* save previous test value */
    hf_sense_test_previous = hf_sense_test;

    /* clear hf sense values */
    hf_sense_integral = 0;
    hf_sense_test = 0;
}


void hf_sense_init(self_test_init_t self_test_init_params)
{
    /* initializing globals */
    hf_sense_integral = 0;
    hf_sense_average = 0;
    hf_sense_test = 0;
    hf_sense_test_previous = 0;
    hf_sense_error_counter = 0;
    hf_sense_fault_flag = TEST_PASS;
    hf_sense_stuck_high_flag = TEST_PASS;

    hf_sense_data_p = self_test_init_params.hf_sense_data_p;

    return;
}


status_t check_for_hf_sense_connection(void)
{
    status_t status = STATUS_NO_FAULT_DETECTED;

    /* check range */
    if( (hf_sense_test_previous > SELF_TEST_HF_SENSE_FAULT_MAXIMUM_TEST_VALUE) ||
        (hf_sense_test_previous < SELF_TEST_HF_SENSE_FAULT_MINIMUM_TEST_VALUE))
    {
        /* increase counter */
        hf_sense_error_counter++;

        set_hf_sense_fault_flag(TEST_FAIL);
    }
    else
    {
        if(hf_sense_error_counter > 0)
        {
            hf_sense_error_counter--;
        }
    }

    /* return fault detected if reach trip threshold */
    if(hf_sense_error_counter >= SELF_TEST_HF_SENSE_FAULT_TRIP_LIMIT)
    {
        status = STATUS_FAULT_DETECTED;
    }

    return status;
}


self_test_flag_t hf_sense_get_fault_flag(void)
{
    return hf_sense_fault_flag;
}


void set_hf_sense_fault_flag(self_test_flag_t flag)
{
    hf_sense_fault_flag = flag;
}


u16 get_hf_sense_error_counter(void)
{
    return hf_sense_error_counter;
}
