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
 * @brief Checks whether a new set of data arrived before the current was completely processed.
 *
 * The routine compares the value of the data sequence number before the data was processed with
 * the value afterwards. If data overrun condition occurs, an error counter increases and sets an
 * error flag. When error counter reaches a threshold, fault detected is returned.
 *
 * @file data_overrun_check.c
 * @ingroup self_test
 *
 *
 *//*
 *
 **************************************************************************************************/
#include "self_test_.h"
#include "types.h"


self_test_error_t  data_overrun_chk;

void data_overrun_check_init(void)
{
    /* initialize variables */
    data_overrun_chk.error_counter = 0;
    data_overrun_chk.error_flag = TEST_PASS;
}

status_t check_for_data_overrun(void)
{
    status_t status = STATUS_NO_FAULT_DETECTED;

    /* compare foreground, background sequence number */
    if(get_foreground_sequence_number() == get_background_sequence_number())
    {
        data_overrun_chk.error_counter = 0;
    }
    else
    {
        /* increase error counter if sequence number don't match */
        data_overrun_chk.error_counter++;

#warning Limits are set similar to one pole code, similar to check data sequence.
        if(data_overrun_chk.error_counter >= DATA_OVERRUN_ERROR_TRIP_LIMIT)
        {
            status = STATUS_FAULT_DETECTED;
        }

        /* Update foreground since there is an error */
        reset_sequence_numbers();
    }

    /* Flag will be set true if at any point there is an error */
    if(data_overrun_chk.error_counter > 0)
    {
        data_overrun_chk.error_flag = TEST_FAIL;
    }

    return status;
}

self_test_flag_t get_data_overrun_error_flag(void)
{
    return data_overrun_chk.error_flag;
}

void reset_data_overrun_error_flag(void)
{
    data_overrun_chk.error_flag = TEST_PASS;
}
