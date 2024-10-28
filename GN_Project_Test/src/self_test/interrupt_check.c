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
 * @brief Checks whether the expected number of interrupts occurred.
 *
 * The main purpose of this feature is to check if the proper number of interrupts occurred during
 * the previous background.
 *
 *
 *
 * @file interrupt_check.c
 * @ingroup self_test
 *
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "utils.h"
#include "self_test_.h"
#include "load_voltage_api.h"
#include "stm32g0xx_it_api.h"
#include "task_manager_api.h"

self_test_error_t interrupt_chk;

self_test_error_t frequency_check;

status_t check_for_proper_number_of_interrupts(void)
{
    status_t status = STATUS_NO_FAULT_DETECTED;

    /* check interrupt counts */
    if(it_get_previous_half_cycle_interrupt_count() == (NUM_INTERRUPTS_PER_HALFCYCLE - 1))
    {
        interrupt_chk.error_counter = 0;
    }
    else
    {
        /* increase error counter if interrupt count don't match */
        interrupt_chk.error_counter++;
    }

#warning the limits need to be discussed. currently setup for 86 seconds.
    if(interrupt_chk.error_counter >= INTERRUPT_ERROR_COUNTER_TRIP_LIMIT)
    {
        status = STATUS_FAULT_DETECTED;
    }

    if(interrupt_chk.error_counter > 0)
    {
        /* set error flag so that ptt will know this test failed. */
        interrupt_chk.error_flag = TEST_FAIL;
    }

    return status;
}

status_t check_for_proper_frequency(void)
{
    status_t status = STATUS_NO_FAULT_DETECTED;

        /* check interrupt counts */
        if(it_get_measured_frequency() < HIGH_LIMIT_OF_FREQUENCY_CHECK_RANGE &&
                it_get_measured_frequency() > LOW_LIMIT_OF_FREQUENCY_CHECK_RANGE )
        {
            if(frequency_check.error_counter)
            {
                frequency_check.error_counter --;
            }
        }
        else
        {
            /* increase error counter if frequency is out of range */
            frequency_check.error_counter++;
        }

        if(frequency_check.error_counter >= FREQUENCY_OUT_OF_RANGE_COUNTER_TRIP_LIMIT)
        {
            status = STATUS_FAULT_DETECTED;
        }

        return status;
}

void interrupt_check_init(void)
{
    /* initialize variables. */
    interrupt_chk.error_counter = 0;
    interrupt_chk.error_flag = TEST_PASS;

    return;
}

void frequency_check_init(void)
{
    /* initialize variables. */
    frequency_check.error_counter = 0;
    frequency_check.error_flag = TEST_PASS;
}
self_test_flag_t get_interrupt_check_error_flag(void)
{
    return interrupt_chk.error_flag;
}

void reset_interrupt_check_error_flag(void)
{
    interrupt_chk.error_flag = TEST_PASS;

    return;
}

u16 get_interrupt_check_error_counter(void)
{
    return interrupt_chk.error_counter;
}
