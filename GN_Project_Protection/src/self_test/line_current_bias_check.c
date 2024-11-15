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
 * @brief checks line current bias out of range.
 *
 * Checks if line current bias is out of range for too long. If bias is out of range for a certain
 * amount of time, fault is detected.
 *
 *
 *
 * @file line_current_bias_check.c
 * @ingroup self_test
 *
 *
 *//*
 *
 **************************************************************************************************/
#include "self_test_.h"
#include "types.h"

u32* line_current_bias_p;
self_test_flag_t line_current_bias_error_flag;
u32 line_current_bias_error_counter;

void line_current_bias_check_init(self_test_init_t self_test_init_params)
{
    /* initialize globals */
    line_current_bias_p = self_test_init_params.line_current_bias_data_p;
    line_current_bias_error_flag = TEST_PASS;
    line_current_bias_error_counter = 0;

    return;
}

status_t check_for_line_current_bias_out_of_range(void)
{
    status_t status = STATUS_NO_FAULT_DETECTED;

    if((self_test_get_user_initiated_test_state() == PTT_IDLE_STATE))
    {
        /* check range */
        if((*line_current_bias_p < LINE_CURRENT_BIAS_MIN)||
           (*line_current_bias_p > LINE_CURRENT_BIAS_MAX))
        {
            /* increase error counter and set flag */
            line_current_bias_error_counter++;
            line_current_bias_error_flag = TEST_FAIL;
        }
        else
        {
            /* decrease error counter. */
            if(line_current_bias_error_counter > 0)
            {
                line_current_bias_error_counter--;
            }
        }

        /* if exceed trip limit */
        if(line_current_bias_error_counter > LINE_CURRENT_BIAS_ERROR_TRIP_LIMIT)
        {
            status = STATUS_FAULT_DETECTED;
        }
    }


    return status;
}

self_test_flag_t get_line_current_bias_error_flag(void)
{
    return line_current_bias_error_flag;
}

void set_line_current_bias_error_flag(self_test_flag_t flag)
{
    line_current_bias_error_flag = flag;
}

u16 get_line_current_bias_error_counter(void)
{
    return line_current_bias_error_counter;
}
