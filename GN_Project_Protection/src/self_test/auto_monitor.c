/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 ***************************************************************************************************
 *  Written by:         Aaron Joseph
 *                      Eaton Electrical
 *                      1000 Cherrington Parkway
 *                      Pittsburgh, PA  15108-4312
 *                      (412) 893-3300
 *//**
 *
 * @brief Auto monitor checks the CT and solenoid 2 seconds after startup and every 20 minutes.
 *
 * At 2 seconds after startup and every 20 minutes after, auto monitor will start testing CT and
 * solenoid.
 *
 * The first test is the CT test. The breaker purposely injects a fault signal through the CT
 * causing a small fault current output that is read back. If the result is not within range for
 * four consecutive attempts, the breaker should trip.
 *
 * The second test is the solenoid test. The breaker turns on the SCR and energize the solenoids
 * near the end of the positive half cycle(with respect to the pcb board). During this time, the
 * voltage across and current passing through the solenoid is not enough to actually trip the
 * breaker, but causes an imbalance in the CT and the output of the CT is read back. If the result
 * is not with range for four consecutive attempts, the breaker will light up a solid LED.
 *
 *
 *
 *
 * @file auto_monitor.c
 * @ingroup self_test
 *
 *
 *//*
 *
 **************************************************************************************************/
#include "self_test_.h"
#include "types.h"
#include "task_manager_api.h"
#include "stm32g0xx_it_api.h"
#include "load_voltage_api.h"
#include "firmware.config"
#include "main_internal.h"
#include "trip_api.h"
#include "gpio_api.h"
#include "usart.h"
#include "main_internal.h"

#if AUTO_MONITOR_CONFIGURATION == AUTO_MONITOR_ENABLED

u8 auto_monitor_counts_per_second;
u16 auto_monitor_seconds_counter;
u32 auto_monitor_gf_test_error_counter;
u32 auto_monitor_trip_test_error_counter;
u32 auto_monitor_gf_integror;   // integrates cycle diff
u32 auto_monitor_gf_cycle_diff;  // difference between current gf data and one cycle ago.
u32* auto_monitor_gf_data_p;
u32 auto_monitor_gf_data_buffer[NUM_INTERRUPTS_PER_HALFCYCLE * 2]; // stores gf data in buffer
bool auto_monitor_gf_test_flag;
bool auto_monitor_trip_test_flag;
bool auto_monitor_trip_error_led_flag;
auto_monitor_result_t auto_monitor_result;
auto_monitor_state_t auto_monitor_state;
u32 automonitoring_cycle;
extern u32 raw_sol_adc_value;
extern M2M_UART_COMMN *pm2m_uart_comm;

void auto_monitor(u32 it)
{
    /* keeps track of cycle counts during automonitor */
    u16 current_interrupt_cnt = it_get_interrupt_count() & 0x1F;
    automonitoring_cycle = (auto_monitor_counts_per_second >> 1);

    /* calculate cycle diff */
    auto_monitor_gf_cycle_diff = ABS((s32)((s32)auto_monitor_gf_data_buffer[current_interrupt_cnt]) - ((s32)(*auto_monitor_gf_data_p)));

    /* update buffer */
    auto_monitor_gf_data_buffer[current_interrupt_cnt] = (*auto_monitor_gf_data_p);

    if((self_test_get_user_initiated_test_state() == PTT_IDLE_STATE) &&
       (load_voltage_get_primary_peak() > LOAD_VOLTAGE_UNDERVOLTAGE_LOCKOUT))
    {
        switch(auto_monitor_state)
        {
            case AUTO_MONITOR_WAIT:
                /* Monitor WAIT state until time elapses of 20min or 2secs after startup */
                if((auto_monitor_seconds_counter == AUTO_MONITOR_SECONDS_AFTER_STARTUP) &&
                   (auto_monitor_counts_per_second == 0))
                {
                    auto_monitor_counts_per_second = 0;
                    auto_monitor_state = AUTO_MONITOR_RESET_VAR;
                }
                break;

            case AUTO_MONITOR_RESET_VAR:
                /* Reset all the variables related to auto-monitoring and enable its start_flag*/
                auto_monitor_gf_test_error_counter = 0;
                auto_monitor_trip_test_error_counter = 0;
                auto_monitor_gf_integror = 0;
                /* go to next state */
                auto_monitor_state = AUTO_MONITOR_GF_TEST;

                break;

            case AUTO_MONITOR_GF_TEST:
            #if GF_CONFIGURATION != GF_NOT_ENABLED
              //Apply stimulus for GF circuits self test
              /* Fifth and Sixth test cycle is added for 2 pole. When both poles powered and
               * around 3.5mA of ground fault current on pole 2. Since the timing is all based
               * on ZCD1, the injection of the ground fault test signal pushes the ground fault
               * current to the other side(polarity). automonitor compares the ground fault data
               * (ABS data) with the previous full cycle and see no difference and causes
               * automonitor to fail gf self test. So fifth and sixth test cycle is add on the
               * other polarity to do additional tests if the first 4 fails. We can also change
               * the third and fourth test to be on the other polarity and not add the fifth and
               * sixth test. But to stay consistent with other version of breakers, keeping the
               * first four tests the same is preferred.
               *  */
                if(
                  #if NUM_POLES_CONFIGURATION == SINGLE_POLE
                    ( load_voltage_get_primary_polarity() == NEGATIVE )
                  #elif NUM_POLES_CONFIGURATION == TWO_POLE
                    ((automonitoring_cycle <= AUTO_MONITOR_GF_TEST_FOURTH_TRY_CYCLE) && load_voltage_get_primary_polarity() == POSITIVE ) ||
                     ((automonitoring_cycle > AUTO_MONITOR_GF_TEST_FOURTH_TRY_CYCLE) && load_voltage_get_primary_polarity() == NEGATIVE )
                    #else
                    #error number of poles not configured
                  #endif
                )
                {
                    /* always inject on the first attempt and do additional IF there was an error*/
                    if((automonitoring_cycle == AUTO_MONITOR_GF_TEST_FIRST_TRY_CYCLE)  ||
                       ((auto_monitor_gf_test_error_counter > 0 )&&
                       ((automonitoring_cycle == AUTO_MONITOR_GF_TEST_SECOND_TRY_CYCLE) ||
                       (automonitoring_cycle == AUTO_MONITOR_GF_TEST_THIRD_TRY_CYCLE)  ||
                       (automonitoring_cycle == AUTO_MONITOR_GF_TEST_FOURTH_TRY_CYCLE)
                      #if NUM_POLES_CONFIGURATION == TWO_POLE
                       || (automonitoring_cycle == AUTO_MONITOR_GF_TEST_FIFTH_TRY_CYCLE)
                       || (automonitoring_cycle == AUTO_MONITOR_GF_TEST_SIXTH_TRY_CYCLE)
                      #endif
                       )))
                       {
                           TEST_GF_SET;
                           auto_monitor_gf_test_flag = TRUE;
                           auto_monitor_trip_test_flag = FALSE;
                       }
                       else
                       {
                           TEST_GF_RESET;
                           auto_monitor_gf_test_flag = FALSE;
                       }
                }

                /* collect response */
                if(auto_monitor_gf_test_flag == TRUE)
                {
                    if(((current_interrupt_cnt & 0x0F) >= 10) && ((current_interrupt_cnt & 0x0F) <= 15))
                    {
                        auto_monitor_gf_integror += auto_monitor_gf_cycle_diff;

                        if((current_interrupt_cnt & 0x0F) == 15)
                        {
                            auto_monitor_gf_test_flag = FALSE;
                            TEST_GF_RESET;
                            if(auto_monitor_gf_integror > SELF_TEST_GF_COIL_OK_MIN)
                            {
                                auto_monitor_gf_test_error_counter = 0;
                                auto_monitor_state = AUTO_MONITOR_TRIP_TEST;
                            }
                            else
                            {
                                auto_monitor_gf_test_error_counter++;
                                if(auto_monitor_gf_test_error_counter >= AUTO_MONITOR_GF_TEST_TRY_TIMES)
                                {
                                    /* update result and jump to wait state */
                                    auto_monitor_result = AUTO_MONITOR_GF_TEST_FAIL;
                                    auto_monitor_state = AUTO_MONITOR_WAIT;
                                }
                            }
                        }
                    }
                    else
                    {
                        auto_monitor_gf_integror = 0;
                    }
                }
                #else
                  auto_monitor_state = AUTO_MONITOR_TRIP_TEST;
                #endif
                break;
#if BOARD_CONFIGURATION == BOARD_SB2
                // different trip test algorithm in SB2.0, it detects the SOL_ADC values
    #ifdef SB20_PUL
            case AUTO_MONITOR_TRIP_TEST:
                /* only turn on at the end of cycle */
                if((current_interrupt_cnt == 31) && (auto_monitor_trip_test_flag == FALSE))
                {
                 if((automonitoring_cycle == AUTO_MONITOR_TRIP_TEST_FIRST_TRY_CYCLE)  || ((auto_monitor_trip_test_error_counter > 0) &&
                  ((automonitoring_cycle == AUTO_MONITOR_TRIP_TEST_SECOND_TRY_CYCLE) ||
                   (automonitoring_cycle == AUTO_MONITOR_TRIP_TEST_THIRD_TRY_CYCLE)  ||
                   (automonitoring_cycle == AUTO_MONITOR_TRIP_TEST_FOURTH_TRY_CYCLE))))
                {
                    if((it_get_primary_zcd() == PRIMARY_ZCD_1))
                    {
                        //
                        auto_monitor_trip_test_flag = TRUE;
                        auto_monitor_gf_integror = 0;
                    }
                    /* when primary zcd is zcd2, which means no power on P1. we don't check and let the test pass */
                    else if(it_get_primary_zcd() == PRIMARY_ZCD_2)
                    {
                        if(auto_monitor_trip_error_led_flag == TRUE)
                        {
                          auto_monitor_result = AUTO_MONITOR_NO_FAULT;
                        }
                        auto_monitor_trip_test_error_counter = 0;
                        auto_monitor_trip_error_led_flag = FALSE;
                        auto_monitor_state = AUTO_MONITOR_WAIT;
                        break;
                    }
                }
            }
            if(auto_monitor_trip_test_flag == TRUE)
            {
                /* collect response */
                if(current_interrupt_cnt < 15)
                {
                    // auto_monitor_gf_integror is reused here to store the integral of SOL_ADC, don't get confused of the name
                    auto_monitor_gf_integror += raw_sol_adc_value;
                }
                /* check response */
                else if(current_interrupt_cnt == 15)
                {
                    auto_monitor_trip_test_flag = FALSE;
                    if(auto_monitor_gf_integror > AUTO_MONITOR_TRIP_TEST_ADC_INTEGRAL_MIN)
                    {
                        /* update results */
                        if(auto_monitor_trip_error_led_flag == TRUE)
                        {
                            auto_monitor_result = AUTO_MONITOR_NO_FAULT;
                        }
                        auto_monitor_trip_test_error_counter = 0;
                        auto_monitor_trip_error_led_flag = FALSE;
                        auto_monitor_state = AUTO_MONITOR_WAIT;
                    }
                    else
                    {
                        auto_monitor_trip_test_error_counter++;
                        if(auto_monitor_trip_test_error_counter >= AUTO_MONITOR_TRIP_TEST_TRY_TIMES)
                        {
                            auto_monitor_result = AUTO_MONITOR_TRIP_TEST_FAIL;
                            auto_monitor_trip_test_error_counter = 0;
                            auto_monitor_trip_error_led_flag = TRUE;
                            auto_monitor_state = AUTO_MONITOR_WAIT;
                            // send message to communication mcu
                            // put it here because the trip test fail will not call trip routine
                            // where the error message is sent to the communication mcu
                            queue_uart_message(pm2m_uart_comm, BREAKER_FAULT_STATE, AUTO_MONITOR_TRIP_SOLENOID, NO_PAYLOAD_2, NO_PAYLOAD_2);

                        }
                    }
                }
            }
            else
            {
                auto_monitor_gf_integror = 0;
            }
                break;
    #else
            case AUTO_MONITOR_TRIP_TEST:
                auto_monitor_gf_integror = 0;
                auto_monitor_result = AUTO_MONITOR_NO_FAULT;
                auto_monitor_trip_test_error_counter = 0;
                auto_monitor_trip_error_led_flag = FALSE;
                auto_monitor_state = AUTO_MONITOR_WAIT;
                break;
    #endif
#else
            case AUTO_MONITOR_TRIP_TEST:

                if(((current_interrupt_cnt & 0x0F) == 15) &&
                   (auto_monitor_trip_test_flag == FALSE)
                    )
                {
              #if NUM_POLES_CONFIGURATION == SINGLE_POLE
                    if((load_voltage_get_primary_polarity() == NEGATIVE))
                    {
                      #if SOLENOID_CONFIGURATION == SOLENOID_DUAL
                        if((automonitoring_cycle == AUTO_MONITOR_TRIP_TEST_FIRST_TRY_CYCLE)  || ((auto_monitor_trip_test_error_counter > 0) &&
                          ((automonitoring_cycle == AUTO_MONITOR_TRIP_TEST_SECOND_TRY_CYCLE))))
                        {
                            auto_monitor_trip_test_flag = TRUE;
                            auto_monitor_gf_integror = 0;
                        }
                        if((auto_monitor_trip_test_error_counter > 0) &&
                          ((automonitoring_cycle == AUTO_MONITOR_TRIP_TEST_THIRD_TRY_CYCLE)  ||
                           (automonitoring_cycle == AUTO_MONITOR_TRIP_TEST_FOURTH_TRY_CYCLE)))
                        {
                            auto_monitor_trip_test_flag = TRUE;
                            auto_monitor_gf_integror = 0;
                        }
                      #else
                        if((automonitoring_cycle == AUTO_MONITOR_TRIP_TEST_FIRST_TRY_CYCLE)  || ((auto_monitor_trip_test_error_counter > 0) &&
                          ((automonitoring_cycle == AUTO_MONITOR_TRIP_TEST_SECOND_TRY_CYCLE) ||
                           (automonitoring_cycle == AUTO_MONITOR_TRIP_TEST_THIRD_TRY_CYCLE)  ||
                           (automonitoring_cycle == AUTO_MONITOR_TRIP_TEST_FOURTH_TRY_CYCLE))))
                        {
                            auto_monitor_trip_test_flag = TRUE;
                            auto_monitor_gf_integror = 0;
                        }
                      #endif //DUAL_SCR
                    }

              #elif NUM_POLES_CONFIGURATION == TWO_POLE
                    /* only turn on at the end of positive half cycle */
                    if((automonitoring_cycle == AUTO_MONITOR_TRIP_TEST_FIRST_TRY_CYCLE)  || ((auto_monitor_trip_test_error_counter > 0) &&
                      ((automonitoring_cycle == AUTO_MONITOR_TRIP_TEST_SECOND_TRY_CYCLE) ||
                       (automonitoring_cycle == AUTO_MONITOR_TRIP_TEST_THIRD_TRY_CYCLE)  ||
                       (automonitoring_cycle == AUTO_MONITOR_TRIP_TEST_FOURTH_TRY_CYCLE))))
                    {
                        if((it_get_primary_zcd() == PRIMARY_ZCD_1) && (load_voltage_get_primary_polarity() == POSITIVE))
                        {
                            TRIP_1_ON;          //   Turn on SCR connected to test solenoid
                            auto_monitor_trip_test_flag = TRUE;
                            auto_monitor_gf_integror = 0;
                        }
                        /* when primary zcd is zcd2. polarity check doesn't really matter if its
                         * positive or negative in this instance, but rather trying to only enter
                         * once on that specific try cycle */
                        else if((it_get_primary_zcd() == PRIMARY_ZCD_2) && (load_voltage_get_primary_polarity() == POSITIVE))
                        {
                            /* if vpeak 1 reads voltage value then solenoid1 is open */
                            if(load_voltage_get_peak(1) > LOAD_VOLTAGE_UNDERVOLTAGE_LOCKOUT)
                            {
                                auto_monitor_trip_test_error_counter++;
                                if(auto_monitor_trip_test_error_counter >= AUTO_MONITOR_TRIP_TEST_TRY_TIMES)
                                {
                                    auto_monitor_result = AUTO_MONITOR_TRIP_TEST_FAIL;
                                    auto_monitor_trip_test_error_counter = 0;
                                    auto_monitor_trip_error_led_flag = TRUE;
                                    auto_monitor_state = AUTO_MONITOR_WAIT;
                                }
                            }
                            else
                            {
                                if(auto_monitor_trip_error_led_flag == TRUE)
                                {
                                    auto_monitor_result = AUTO_MONITOR_NO_FAULT;
                                }
                                auto_monitor_trip_test_error_counter = 0;
                                auto_monitor_trip_error_led_flag = FALSE;
                                auto_monitor_state = AUTO_MONITOR_WAIT;
                            }

                            break;
                        }
                    }
                  #else
                    #error not configured or not completed
                  #endif
                }


                if(auto_monitor_trip_test_flag == TRUE)
                {
                    /* collect response */
                    if(((current_interrupt_cnt & 0x0F) >= 0) && ((current_interrupt_cnt & 0x0F) < 10))
                    {
                        auto_monitor_gf_integror += auto_monitor_gf_cycle_diff;
                    }
                    /* check response */
                    else if((current_interrupt_cnt & 0x0F) == 10)
                    {
                        auto_monitor_trip_test_flag = FALSE;
                        if((auto_monitor_gf_integror > AUTO_MONITOR_TRIP_TEST_GF_INTEGRAL_MIN) &&
                           (auto_monitor_gf_integror < AUTO_MONITOR_TRIP_TEST_GF_INTEGRAL_MAX))
                        {
                            /* update results */
                            if(auto_monitor_trip_error_led_flag == TRUE)
                            {
                                auto_monitor_result = AUTO_MONITOR_NO_FAULT;
                            }
                            auto_monitor_trip_test_error_counter = 0;
                            auto_monitor_trip_error_led_flag = FALSE;
                            auto_monitor_state = AUTO_MONITOR_WAIT;
                        }
                        else
                        {
                            auto_monitor_trip_test_error_counter++;
                            if(auto_monitor_trip_test_error_counter >= AUTO_MONITOR_TRIP_TEST_TRY_TIMES)
                            {
                                auto_monitor_result = AUTO_MONITOR_TRIP_TEST_FAIL;
                                auto_monitor_trip_test_error_counter = 0;
                                auto_monitor_trip_error_led_flag = TRUE;
                                auto_monitor_state = AUTO_MONITOR_WAIT;
                            }
                        }
                    }
                }
                else
                {
                    auto_monitor_gf_integror = 0;
                }

                break;
#endif
            default:
                break;
        }
    }

    return;
}

void auto_monitor_init(self_test_init_t self_test_init_params)
{

    /* initialize globals */
    auto_monitor_initialize_globals();

    /* assign pointer */
    auto_monitor_gf_data_p = self_test_init_params.ground_fault_data_p;

    /* add zcd callback */
    task_man_add_task_to_schedule((task_t){.task_p = auto_monitor_zcd_callback,
                                           .it_bits = TSK_MAN_ZCD});

    /* add auto monitor task */
    task_man_add_task_to_schedule((task_t){.task_p = auto_monitor,
                                           .it_bits = TSK_MAN_IT_ALL});
#if BOARD_CONFIGURATION == BOARD_SB2
    // Not turning on solenoid in SB2.0 in trip test
#else
    /* turn off trip task */
    task_man_add_task_to_schedule((task_t){.task_p = auto_monitor_turn_off_trip_task,
                                           .it_bits = TSK_MAN_IT_ALL});
#endif
    return;
}

void auto_monitor_zcd_callback(u32 it)
{
    /* increase counts every half cycle */
    auto_monitor_counts_per_second++;

    /* increase seconds */
    if(auto_monitor_counts_per_second >= AUTO_MONITOR_ONE_SECOND_HALF_CYCLE_COUNTS)
    {
        auto_monitor_counts_per_second = 0;
        auto_monitor_seconds_counter++;

        /* clear seconds if seconds reach interval */
        if(auto_monitor_seconds_counter >= AUTO_MONITOR_INTERVAL_SECONDS)
        {
            auto_monitor_seconds_counter = 0;
        }
    }

    /* this is added in case interrupt count 15 is somehow skipped */
    TEST_GF_RESET;

    return;
}

void auto_monitor_initialize_globals(void)
{
    auto_monitor_counts_per_second = 0;
    auto_monitor_seconds_counter = 0;
    auto_monitor_gf_test_error_counter= 0;
    auto_monitor_trip_test_error_counter = 0;
    auto_monitor_gf_integror = 0;
    auto_monitor_gf_cycle_diff = 0;
    auto_monitor_gf_test_flag = FALSE;
    auto_monitor_trip_test_flag = FALSE;
    auto_monitor_trip_error_led_flag = FALSE;
    auto_monitor_result = AUTO_MONITOR_NO_FAULT;
    auto_monitor_state = AUTO_MONITOR_WAIT;
    automonitoring_cycle = 0;

    for(u32 i = 0; i < (NUM_INTERRUPTS_PER_HALFCYCLE * 2); i++)
    {
        auto_monitor_gf_data_buffer[i] = 0;
    }

    return;
}

auto_monitor_state_t auto_monitor_get_state(void)
{
    return auto_monitor_state;
}

auto_monitor_result_t auto_monitor_get_result(void)
{
    return auto_monitor_result;
}

u32 auto_monitor_get_cycle(void)
{
    return automonitoring_cycle;
}

bool auto_monitor_get_trip_test_flag(void)
{
    return auto_monitor_trip_test_flag;
}

void auto_monitor_turn_off_trip_task(u32 it)
{
    if (auto_monitor_trip_test_flag == TRUE)
    {
        TRIP_1_OFF;
        TRIP_2_OFF;
    }

    return ;
}
#endif

