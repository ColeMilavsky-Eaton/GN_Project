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
 * @brief Determine whether the background has passed a continuous stream of data to foreground.
 *
 * Both the background and foreground process maintain independent sequence numbers. The background
 * sequence number will increase when the background is processing. And once the background has
 * completed and jump to foreground, the foreground number should be 1 less than the background.
 *
 *
 *
 * @file sequence_number_check.c
 * @ingroup self_test
 *
 *
 *//*
 *
 **************************************************************************************************/
#include "self_test_.h"
#include "types.h"
#include "utils.h"
#include "firmware.config"
#include "task_manager_api.h"


u32 background_sequence_number;
u32 foreground_sequence_number;
self_test_error_t  sequence_number_check;


void sequence_number_check_init(void)
{
    /* initialize globals. */
    background_sequence_number = 0;
    foreground_sequence_number = 0;
    sequence_number_check.error_counter = 0;
    sequence_number_check.error_flag = TEST_PASS;


    /** add task for zcd callback */
    task_man_add_task_to_schedule((task_t){.task_p = update_background_sequence_number,
                                           .it_bits = TSK_MAN_ZCD});

    return ;
}

status_t check_for_proper_data_sequence(void)
{
    status_t status = STATUS_NO_FAULT_DETECTED;

    if((background_sequence_number == ( foreground_sequence_number + 1 )) ||
       ((background_sequence_number == 0 ) && ( foreground_sequence_number == U32_MAX ))
      )
    {
        sequence_number_check.error_counter = 0;
    }
    else
    {
        sequence_number_check.error_counter++;

        if(sequence_number_check.error_counter >= INVALID_SEQUENCE_TRIP_LIMIT)
        {
            status = STATUS_FAULT_DETECTED;
        }
    }

    if(sequence_number_check.error_counter > 0)
    {
        /* set flag so that ptt will know this flag is set. */
        sequence_number_check.error_flag = TEST_FAIL;
    }

    /* update foregorund */
    foreground_sequence_number = background_sequence_number;

    return status;
}

#warning improve consistency between self test files by making sure similar functions (like get_error_counter()) have similar function signitures

u32 get_foreground_sequence_number(void)
{
    return foreground_sequence_number;
}

u32 get_background_sequence_number(void)
{
    return background_sequence_number;
}

void update_background_sequence_number(u32 it)
{
    background_sequence_number++;
}

u16 get_sequence_number_error_counter(void)
{
    return sequence_number_check.error_counter;
}

self_test_flag_t get_sequence_number_error_flag(void)
{
    return sequence_number_check.error_flag;
}

void reset_sequence_number_error_flag(void)
{
    sequence_number_check.error_flag = TEST_PASS;
}

void reset_sequence_numbers(void)
{
    background_sequence_number = 0;
    foreground_sequence_number = 0;

    return;
}
