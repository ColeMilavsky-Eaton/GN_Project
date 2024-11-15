/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 **************************************************************************************************
 *  Written by:         Ming Wu
 *                      Innovation
 *                      Commercial & Residential Distribution Solutions (CRDS), Eaton
 *                      1000 Cherrington Parkway
 *                      Moon Township, PA 15108
 *//**
 * @defgroup primary_switch primary_switch component
 *
 * @brief primary switch component monitors the state of the primary switch and
 * will report if it opens unexpectedly, or when it closes manually.

  * NOTE: 1.
  *
  * @file primary_switch.c
  * @ingroup primary_switch
  *
  *
  *//*
  *
  **************************************************************************************************/

#include "types.h"
#include "main_internal.h"
#include "gpio_api.h"
#include "primary_switch.h"
#include "stm32g0xx_it_api.h"
#include "adc_api.h"
#include "firmware.config"
#include "string.h"

//#include "led_api.h"

psw_t primary_switch;
u32 hall_data_adc;

static bool ps_initialized_flag = FALSE;
static pswitch_status_t ps_instant_state= SW_UNKNOWN;

void primary_switch_init(void)
{
    primary_switch.closed_counter = 0;
    primary_switch.open_counter = 0;
    primary_switch.trip_counter = 0;
    primary_switch.state = SW_UNKNOWN;

    hall_data_adc = 0;

    if(ps_initialized_flag == FALSE)
    {
        /* task type to assign tasks */
        task_t t = (task_t){.task_p = NULL, .it_bits = 0};

        /* assign task */
        t.task_p = primary_switch_zcd_callback;
        t.it_bits = TSK_MAN_ZCD;

        task_man_add_task_to_schedule(t);

        ps_initialized_flag = TRUE;
    }
    return;
}

void primary_switch_zcd_callback(u32 it)
{
    // check hall data adc value to determine state
    ps_instant_state = primary_switch_get_state_from_level(hall_data_adc);
    primary_switch_debouncing();

    if(primary_switch.open_counter > PSW_DEBOUNCE_TIMER_COUNT)
    {
        primary_switch.state = SW_OPEN;
        primary_switch.open_counter = PSW_DEBOUNCE_TIMER_COUNT; //keep it leveled.
    }
    else if (primary_switch.closed_counter > PSW_DEBOUNCE_TIMER_COUNT)
    {
        if( primary_switch.previous_state == SW_TRIP)
        {
//             If the privouse_state is trip, then don't change the state to closed.
//             as it is not possible to latch.
//             this is to prevent someone push the primary switch to closed
//             from trip position and confuse the FW
        }
        else
        {
            primary_switch.state = SW_CLOSED;
        }
        primary_switch.closed_counter = PSW_DEBOUNCE_TIMER_COUNT; //keep it leveled.
    }
    else if(primary_switch.trip_counter > PSW_DEBOUNCE_TIMER_COUNT)
    {
        primary_switch.state = SW_TRIP;
        primary_switch.trip_counter = PSW_DEBOUNCE_TIMER_COUNT; //keep it leveled.
    }
    else if(primary_switch.failed_counter > PSW_DEBOUNCE_TIMER_COUNT)
    {
        primary_switch.state = SW_FAILED;
        primary_switch.failed_counter = PSW_DEBOUNCE_TIMER_COUNT; //keep it leveled.
    }
    else
    {
        // During the transition, update the previous state.
        // Meanwhile do not use SW_UNKNOWN anymore
        primary_switch.previous_state = primary_switch.state;
        //primary_switch.state = SW_UNKNOWN;
    }
    return;
}

pswitch_status_t get_primary_switch_direct_debounced_status(void)
{
    u8 wait_for_stable_primary= 0;

    primary_switch.failed_counter = 0;
    primary_switch.closed_counter = 0;
    primary_switch.open_counter = 0;
    primary_switch.trip_counter = 0;
    primary_switch.failed_counter = 0;
    primary_switch.state = SW_UNKNOWN;

    for(wait_for_stable_primary = 0; wait_for_stable_primary < PSW_DEBOUNCE_INST_TIMEOUT; wait_for_stable_primary++)
    {
        // get hall in ADC value directly
        adc_read_sequence(HALL_IN_ADC_CHANNEL);
        hall_data_adc = adc_get_channel_data(HALL_IN_ADC_CHANNEL);

        ps_instant_state = SW_UNKNOWN;
        ps_instant_state = primary_switch_get_state_from_level(hall_data_adc);
        primary_switch_debouncing();
    }

    if(primary_switch.failed_counter > PSW_DEBOUNCE_INST_COUNT)
    {
        primary_switch.state = SW_FAILED;
    }
    else if(primary_switch.trip_counter > PSW_DEBOUNCE_INST_COUNT)
    {
        primary_switch.state = SW_TRIP;
    }
    else if(primary_switch.open_counter > PSW_DEBOUNCE_INST_COUNT)
    {
        primary_switch.state = SW_OPEN;
    }
    else if (primary_switch.closed_counter > PSW_DEBOUNCE_INST_COUNT)
    {
        primary_switch.state = SW_CLOSED;
    }
    // if none of the counter is high enough, return the last state
    else
    {
        primary_switch.state = ps_instant_state;
    }
    return primary_switch.state;
}

inline pswitch_status_t get_primary_switch_debounced_state(void)
{
	return primary_switch.state;
}

inline u32 get_primary_switch_raw_adc(void)
{
	return hall_data_adc;
}

inline pswitch_status_t primary_switch_get_state_from_level(u32 hall_data_adc_value )
{
    if(	   (hall_data_adc >= PS_CLOSED_STATE_ADC_MIN && hall_data_adc <= PS_CLOSED_STATE_ADC_MAX )
    	|| (hall_data_adc >= PS_CLOSED_STATE_ADC_MIN_ALT && hall_data_adc <= PS_CLOSED_STATE_ADC_MAX_ALT))
    {
        return SW_CLOSED;
    }
    else if(    (hall_data_adc >= PS_OPEN_STATE_ADC_MIN && hall_data_adc <= PS_OPEN_STATE_ADC_MAX)
    		||  (hall_data_adc >= PS_OPEN_STATE_ADC_MIN_ALT && hall_data_adc <= PS_OPEN_STATE_ADC_MAX_ALT))
    {
        return SW_OPEN;
    }
    else if(    (hall_data_adc >= PS_TRIP_STATE_ADC_MIN && hall_data_adc <= PS_TRIP_STATE_ADC_MAX)
    		||  (hall_data_adc >= PS_TRIP_STATE_ADC_MIN_ALT && hall_data_adc <= PS_TRIP_STATE_ADC_MAX_ALT))
    {
        return SW_TRIP;
    }
    else
    {
        return SW_FAILED;
    }
}

inline void primary_switch_debouncing()
{
    switch(ps_instant_state)
    {
    case SW_OPEN:
        primary_switch.open_counter++;
        primary_switch.closed_counter = 0;
        primary_switch.trip_counter = 0;
        primary_switch.failed_counter = 0;
        break;
    case SW_CLOSED:
        primary_switch.open_counter = 0;
        primary_switch.closed_counter++;
        primary_switch.trip_counter = 0;
        primary_switch.failed_counter = 0;
        break;
    case SW_TRIP:
        primary_switch.open_counter = 0;
        primary_switch.closed_counter = 0;
        primary_switch.trip_counter ++;
        primary_switch.failed_counter = 0;
        break;
    case SW_FAILED:
        primary_switch.open_counter = 0;
        primary_switch.closed_counter = 0;
        primary_switch.trip_counter = 0;
        primary_switch.failed_counter ++;
        break;
    default:
        break;
    }
}
