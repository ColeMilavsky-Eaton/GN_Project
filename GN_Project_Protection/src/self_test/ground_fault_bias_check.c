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
 * @brief checks ground fault bias out of range.
 *
 * check for ground fault bias out of range. If ground fault bias is out of range, an error counter
 * increases and a flag is set. Once the error counter reaches a threshold, fault is returned.
 *
 *
 *
 * @file ground_fault_bias_check.c
 * @ingroup self_test
 *
 *
 *//*
 *
 **************************************************************************************************/
#include "self_test_.h"
#include "stm32g0xx_it_api.h"
#include "types.h"

u32* gf_bias_p;
u32 gf_bias_error_counter;
self_test_flag_t gf_bias_error_flag;


void ground_fault_bias_check_init(self_test_init_t self_test_init_params)
{
    /* initialize variables */
    gf_bias_p = self_test_init_params.ground_fault_bias_data_p;
    gf_bias_error_flag = TEST_PASS;
    gf_bias_error_counter = 0;

    return;
}


status_t check_for_ground_fault_bias_out_of_range(void)
{
    status_t status = STATUS_NO_FAULT_DETECTED;

    /* check if self test is on-going */
    if((it_get_gf_bias_in_range_flag() == FALSE) &&
       (self_test_get_user_initiated_test_state() == PTT_IDLE_STATE))
    {
        gf_bias_error_flag = TEST_FAIL;
        gf_bias_error_counter += GROUND_FAULT_BIAS_ERROR_INCREMENT_COUNT;
    }
    else
    {
        if(gf_bias_error_counter >= GROUND_FAULT_BIAS_ERROR_DECREMENT_COUNT)
        {
            gf_bias_error_counter -= GROUND_FAULT_BIAS_ERROR_DECREMENT_COUNT;
        }
    }

    if(gf_bias_error_counter > GROUND_FAULT_BIAS_ERROR_TRIP_THREHOLD_COUNT)
    {
        status = STATUS_FAULT_DETECTED;
    }

    return status;
}

inline self_test_flag_t get_ground_fault_bias_error_flag(void)
{
    /* return error flag */
    return gf_bias_error_flag;
}

inline void reset_ground_fault_bias_error_flag()
{
    /* reset error flag */
    gf_bias_error_flag = TEST_PASS;

    return;
}




