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
 *                      Commercial & Residential Distribution Solutions (CRDS), Eaton
 *						1000 Cherrington Parkway
 *						Moon Township, PA 15108
 *//**
 * @defgroup temperature_control temperature control component
 *
 * @brief Temperature control component using the internal temperature sensor to monitor the junction temperature
 * of STM32.

 * NOTE: 1.
 *
 * @file temperature_control.c
 * @ingroup temperature_control
 *
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "temperature_control_.h"
#include "main_internal.h"
#include "gpio_api.h"
#include "stm32g0xx_it_api.h"
#include "firmware.config"
#include "usart.h"
#include "secondary_solenoid_api.h"
#include "led_api.h"
#include "string.h"

s32 temperature_buffer[NUM_INTERRUPTS_PER_HALFCYCLE * 2];
u32 overheat_counter, cool_down_counter;
u8 buffer_ptr;
u32 temperature_raw_data;
//u32 vref_raw_data;
s32 temperature_c, temperature_c_intergral, temperature_c_average;
s16 temperature_c_trending;
bool temperature_management_enabled = FALSE;
bool temperature_cool_down_flag = FALSE;
bool temperature_management_on_going = FALSE;
bool temperature_cool_down_on_going = FALSE;
bool secondary_switch_gets_closed = FALSE;

extern M2M_UART_COMMN *pm2m_uart_comm;

void temperature_control_it_task(u32 it)
{
    // read temperature
    (void)adc_read_internal_ch(LL_ADC_CHANNEL_TEMPSENSOR);
    temperature_raw_data = (u32)adc_get_channel_data(LL_ADC_CHANNEL_TEMPSENSOR);

    temperature_c = __LL_ADC_CALC_TEMPERATURE(3300, temperature_raw_data, LL_ADC_RESOLUTION_12B);

    temperature_buffer[buffer_ptr++] = temperature_c;
    temperature_c_intergral +=temperature_c;

    // when the buffer is full, calculate average
    if(buffer_ptr >= NUM_INTERRUPTS_PER_HALFCYCLE * 2)
    {
        buffer_ptr = 0;
        // 32 samples takes 16 cycles, ie. 0.27s
        temperature_c_average = temperature_c_intergral / 32;
        if(temperature_management_enabled)
        {
            // when over temp is on, start to checking cool down
            if(temperature_c_average <= OVERTEMP_QUIT_TEMP)
            {
                cool_down_counter ++;
            }
            else
            {
                if(cool_down_counter >= 1)
                {
                    cool_down_counter --;
                }
            }
        }
        // If average of temperature is over limit, increase counter
        if(temperature_c_average >= OVERTEMP_TRIGGER_TEMP)
        {
            overheat_counter ++;
        }
        else
        {
            // if not, decrease the counter
            if(overheat_counter >= 1)
            {
                overheat_counter --;
            }
        }

        temperature_c_trending = (s16)temperature_c_average;
        temperature_c_intergral = 0;
        temperature_c_average = 0;
    }

}

void clear_temperature_control_variables()
{
    temperature_cool_down_flag = FALSE;
    temperature_management_enabled = FALSE;
    temperature_management_on_going = FALSE;
    overheat_counter = 0;
    cool_down_counter = 0;
}

void temperature_control_init()
{

    /* initialize globals */
    buffer_ptr = 0;
    overheat_counter = 0;
    temperature_c = 0;
    temperature_c_intergral = 0;
    temperature_c_average = 0;
    temperature_c_trending = 0;
    temperature_management_enabled = FALSE;

    memset(temperature_buffer, 0, sizeof(temperature_buffer));

    /* add zcd callback */
    task_man_add_task_to_schedule((task_t){.task_p = temperature_control_zcd_callback,
                                           .it_bits = TSK_MAN_ZCD});

    /* add auto monitor task */
    task_man_add_task_to_schedule((task_t){.task_p = temperature_control_it_task,
                                           .it_bits = TSK_MAN_IT_4 | TSK_MAN_IT_20});
}

void temperature_control_zcd_callback(u32 it)
{
    u8 trip_code = 0;
    // If the breaker is not closed, no need to do the temperature control
    if(get_primary_switch_debounced_state() != SW_CLOSED)
    {
        clear_temperature_control_variables();
        return;
    }
    // if overheat last for over 10min
    if(!temperature_management_enabled && overheat_counter >= OVERTEMP_COUNTER_LIMIT)
    {
        // Start temperature management process
        temperature_management_enabled = TRUE;
    }

    if(temperature_management_enabled)
    {
        if(temperature_management_on_going == FALSE)
        {
            // Overtemp handling
#ifdef SB20_PUL
            // Load feedback LED illuminates RED
            LOAD_PWR_LED_Red_ON;
#else
#endif
            if(get_primary_switch_debounced_state() != SW_CLOSED)
            {
                // no need to do anything
            }
            else
            {
                if(check_ss_closed())
                {
                    // open the secondary contacts.
                    request_open_secondary_solenoid();
                    // send an over temp message to ESP
                    queue_uart_message(pm2m_uart_comm, BREAKER_FAULT_STATE, OVER_TEMP_MESSAGE, NO_PAYLOAD_2, NO_PAYLOAD_2);
                }
                else // need to trip primary
                {
                    trip_code = OVER_TEMP_TRIP;
                    trip(&trip_code, 1);
                 }
            }
            temperature_management_on_going = TRUE;
            overheat_counter = 0;
        }

        // here use overheat_counter to delay 12 cycles for the secondary switch status to update
        if( temperature_management_on_going && check_ss_closed() && overheat_counter >= 24)
        {
            // if the secondary switch gets closed during the overtemp event, reset the timer
            clear_temperature_control_variables();
        }
        // if temperature management is not working
        if(overheat_counter >= OVERTEMP_COUNTER_LIMIT)
        {
            // trip the primary if the primary is closed
            if(get_primary_switch_debounced_state() == SW_CLOSED)
            {
                trip_code = OVER_TEMP_TRIP;
                trip(&trip_code, 1);
            }
            else
            {
                // Critical error, TBD
            }
        }
        //
        if(cool_down_counter >= OVERTEMP_COUNTER_LIMIT)
        {
            // Start cool down process
            temperature_cool_down_flag = TRUE;
            cool_down_counter = 0;
        }
        if(temperature_cool_down_flag)
        {
            // Cooled down handling
#ifdef SB20_PUL
            // Turn off Red load LED
            LOAD_PWR_LED_Red_OFF;
#else
#endif
            if(!check_ss_closed())
            {
                // close the secondary contacts.
                request_close_secondary_solenoid();
                // send an cooled down message to ESP
                queue_uart_message(pm2m_uart_comm, BREAKER_FAULT_STATE, COOLED_DOWN_MESSAGE, NO_PAYLOAD_2, NO_PAYLOAD_2);
            }
            else
            {
                // it has been closed already by the user
                // no cooled message is sent
                // the bracket is kept here for comments only
            }

            // means over temperature management is over
            clear_temperature_control_variables();
        }
    }
}
