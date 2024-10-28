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
 * @defgroup shunt_cal shunt_cal component
 *
 * @brief The main purpose of this component is to calibrate the breaker for electronic overload.
 *
 * The following is the calibration process.
 * 1. Breaker needs to be powered on with 60V AC.
 * 2. Provide the first stage current (20A DC) through the shunt.
 * 3. provide the second stage current depending on the rating of the breaker to calibrate to.
 *    For calibrating a 15 Amp breaker, second stage current should be 37.5 amps.
 *    For calibrating a 20 Amp breaker, second stage current should be 50 amps.
 *    For calibrating a 25 Amp breaker, second stage current should be 62.5 amps.
 *    For calibrating a 30 Amp breaker, second stage current should be 75 amps.
 * 4. if calibration is successful, breaker should trip electronic overload.
 *
 *
 *
 * @file shunt_cal.c
 * @ingroup shunt_cal
 *
 *
 *//*
 *
 **************************************************************************************************/

#include "types.h"
#include "shunt_cal_.h"
#include "firmware.config"
#include "task_manager_api.h"
#include "load_current_api.h"
#include "load_voltage_api.h"
#include "utils.h"
#include "main_internal.h"
#include "flash_api.h"

load_current_rating_t breaker_rating;
u8 breaker_cal_factor;
u8 shunt_cal_counts_per_second;
u16 shunt_cal_seconds;
bool shunt_calibrated_flag;
u8 shunt_cal_first_stage_counter;
u8 shunt_cal_second_stage_counter;
u32 shunt_cal_first_stage_current;
u32 shunt_cal_second_stage_current;
u32 shunt_cal_first_stage_min;
u32 shunt_cal_second_stage_min;
u32 shunt_cal_first_stage_max;
u32 shunt_cal_second_stage_max;
u32 shunt_cal_first_stage_ratio_15A;
u32 shunt_cal_first_stage_ratio_20A;
#if RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_25_30A
u32 shunt_cal_first_stage_ratio_25A;
u32 shunt_cal_first_stage_ratio_30A;

#elif RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_30A
u32 shunt_cal_first_stage_ratio_30A;

#endif


bool shunt_cal_first_stage_pass;
u16 shunt_cal_wait_cycles;
u32 shunt_cal_second_stage_total_diff_threshold1;
u32 shunt_cal_second_stage_total_diff_threshold2;
bool shunt_cal_rating_checked_OK;
bool shunt_cal_factor_checked_OK;
bool shunt_cal_complete_OK;

u8 shunt_cal_index ;
u32 shunt_cal_voltage_peak ;
u32 shunt_cal_positive_current;
u32 shunt_cal_negative_current;
u32 shunt_cal_half_cycle_total_current;

const u32 shunt_cal_current_table_15A[SHUNT_CAL_TABLE_SIZE] = {

    (u32)(SHUNT_CAL_15A_CENTER_CURRENT * 8. * 0.80),
    (u32)(SHUNT_CAL_15A_CENTER_CURRENT * 8. * 0.84),
    (u32)(SHUNT_CAL_15A_CENTER_CURRENT * 8. * 0.88),
    (u32)(SHUNT_CAL_15A_CENTER_CURRENT * 8. * 0.92),
    (u32)(SHUNT_CAL_15A_CENTER_CURRENT * 8. * 0.96),
    (u32)(SHUNT_CAL_15A_CENTER_CURRENT * 8. * 1.04),
    (u32)(SHUNT_CAL_15A_CENTER_CURRENT * 8. * 1.08),
    (u32)(SHUNT_CAL_15A_CENTER_CURRENT * 8. * 1.12),
    (u32)(SHUNT_CAL_15A_CENTER_CURRENT * 8. * 1.16),
    (u32)(SHUNT_CAL_15A_CENTER_CURRENT * 8. * 1.20)
};

const u32 shunt_cal_current_table_20A[SHUNT_CAL_TABLE_SIZE] = {

    (u32)(SHUNT_CAL_20A_CENTER_CURRENT * 8. * 0.80),
    (u32)(SHUNT_CAL_20A_CENTER_CURRENT * 8. * 0.84),
    (u32)(SHUNT_CAL_20A_CENTER_CURRENT * 8. * 0.88),
    (u32)(SHUNT_CAL_20A_CENTER_CURRENT * 8. * 0.92),
    (u32)(SHUNT_CAL_20A_CENTER_CURRENT * 8. * 0.96),
    (u32)(SHUNT_CAL_20A_CENTER_CURRENT * 8. * 1.04),
    (u32)(SHUNT_CAL_20A_CENTER_CURRENT * 8. * 1.08),
    (u32)(SHUNT_CAL_20A_CENTER_CURRENT * 8. * 1.12),
    (u32)(SHUNT_CAL_20A_CENTER_CURRENT * 8. * 1.16),
    (u32)(SHUNT_CAL_20A_CENTER_CURRENT * 8. * 1.20)
};

#if RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_25_30A
const u32 shunt_cal_current_table_25A[SHUNT_CAL_TABLE_SIZE] = {

    (u32)(SHUNT_CAL_25A_CENTER_CURRENT * 8. * 0.80),
    (u32)(SHUNT_CAL_25A_CENTER_CURRENT * 8. * 0.84),
    (u32)(SHUNT_CAL_25A_CENTER_CURRENT * 8. * 0.88),
    (u32)(SHUNT_CAL_25A_CENTER_CURRENT * 8. * 0.92),
    (u32)(SHUNT_CAL_25A_CENTER_CURRENT * 8. * 0.96),
    (u32)(SHUNT_CAL_25A_CENTER_CURRENT * 8. * 1.04),
    (u32)(SHUNT_CAL_25A_CENTER_CURRENT * 8. * 1.08),
    (u32)(SHUNT_CAL_25A_CENTER_CURRENT * 8. * 1.12),
    (u32)(SHUNT_CAL_25A_CENTER_CURRENT * 8. * 1.16),
    (u32)(SHUNT_CAL_25A_CENTER_CURRENT * 8. * 1.20)
};
#endif
#if ((RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_25_30A) ||\
     (RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_30A))

const u32 shunt_cal_current_table_30A[SHUNT_CAL_TABLE_SIZE] = {

    (u32)(SHUNT_CAL_30A_CENTER_CURRENT * 8. * 0.80),
    (u32)(SHUNT_CAL_30A_CENTER_CURRENT * 8. * 0.84),
    (u32)(SHUNT_CAL_30A_CENTER_CURRENT * 8. * 0.88),
    (u32)(SHUNT_CAL_30A_CENTER_CURRENT * 8. * 0.92),
    (u32)(SHUNT_CAL_30A_CENTER_CURRENT * 8. * 0.96),
    (u32)(SHUNT_CAL_30A_CENTER_CURRENT * 8. * 1.04),
    (u32)(SHUNT_CAL_30A_CENTER_CURRENT * 8. * 1.08),
    (u32)(SHUNT_CAL_30A_CENTER_CURRENT * 8. * 1.12),
    (u32)(SHUNT_CAL_30A_CENTER_CURRENT * 8. * 1.16),
    (u32)(SHUNT_CAL_30A_CENTER_CURRENT * 8. * 1.20)
};
#endif

void shunt_cal_init(void)
{
    /* initialize globals. */
    shunt_cal_initialize_globals();

    /* add zcd callback */
    task_man_add_task_to_schedule((task_t){.task_p = shunt_cal_zcd_callback,
                                           .it_bits = TSK_MAN_ZCD});

    /* read config data from breaker */
    shunt_cal_check_and_set_rating_and_factor();

    return;
}

void shunt_cal_initialize_globals(void)
{
    breaker_rating = LOAD_CURRENT_RATING_NOT_DEFINED;
    breaker_cal_factor = SHUNT_CAL_HALF_TABLE_SIZE;
    shunt_cal_counts_per_second = 0;
    shunt_cal_seconds = 0;
    shunt_calibrated_flag = FALSE;
    shunt_cal_first_stage_counter = 0;
    shunt_cal_second_stage_counter = 0;
    shunt_cal_first_stage_current = 0;
    shunt_cal_second_stage_current = 0;
    shunt_cal_first_stage_min = 40000;
    shunt_cal_second_stage_min = 40000;
    shunt_cal_first_stage_max = 0;
    shunt_cal_second_stage_max = 0;
    shunt_cal_first_stage_ratio_15A = 0;
    shunt_cal_first_stage_ratio_20A = 0;
#if RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_25_30A
    shunt_cal_first_stage_ratio_25A = 0;
#endif
#if ((RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_25_30A) ||\
     (RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_30A))
    shunt_cal_first_stage_ratio_30A = 0;
#endif
    shunt_cal_first_stage_pass = FALSE;
    shunt_cal_wait_cycles = 0;
    shunt_cal_second_stage_total_diff_threshold1 = 0;
    shunt_cal_second_stage_total_diff_threshold2 = 0;
    shunt_cal_rating_checked_OK = FALSE;
    shunt_cal_factor_checked_OK = FALSE;
    shunt_cal_complete_OK = FALSE;

    shunt_cal_index = 0;
    shunt_cal_voltage_peak = 0;
    shunt_cal_positive_current = 0;
    shunt_cal_negative_current = 0;
    shunt_cal_half_cycle_total_current = 0;

    return;
}

void shunt_cal_zcd_callback(u32 it)
{

    shunt_cal_counts_per_second++;

    if(shunt_cal_counts_per_second >= SHUNT_CAL_ONE_SECOND_COUNTS)
    {
        shunt_cal_counts_per_second = 0;
        shunt_cal_seconds ++;

        if(shunt_cal_seconds >= SHUNT_CAL_PERIOD_SECONDS)
        {
            shunt_cal_seconds = SHUNT_CAL_PERIOD_SECONDS;
        }
    }

    return;
}

void shunt_cal_and_config(void)
{
    /* initialize variables */
    shunt_cal_index = 0;
    shunt_cal_voltage_peak = load_voltage_get_primary_peak();
    shunt_cal_positive_current = load_current_get_positive_current();
    shunt_cal_negative_current = load_current_get_negative_current();
    shunt_cal_half_cycle_total_current = load_current_get_total_current_without_averaging_applied();

    /* AC Voltage shall be low within a certain range, the load current shall be DC */
    if((shunt_cal_voltage_peak >= SHUNT_CAL_MIN_VOLTAGE) &&
       (shunt_cal_voltage_peak < SHUNT_CAL_MAX_VOLTAGE) &&
       ((shunt_cal_positive_current == shunt_cal_half_cycle_total_current) ||
        (shunt_cal_negative_current == shunt_cal_half_cycle_total_current)
       )
      )
    {
        /* if current within 1st stage current (20 Amps DC) */
        if((shunt_cal_half_cycle_total_current > SHUNT_CAL_FIRST_STAGE_CURRENT_LOW) &&
           (shunt_cal_half_cycle_total_current < SHUNT_CAL_FIRST_STAGE_CURRENT_HIGH)
          )
        {
            shunt_cal_first_stage_counter++;

            /* add 10 consecutive half cycle values */
            if((shunt_cal_first_stage_counter > SHUNT_CAL_FIRST_STAGE_HALF_CYCLES) &&
               (shunt_cal_first_stage_counter <= SHUNT_CAL_FIRST_STAGE_HALF_CYCLES + 10)
              )
            {
                shunt_cal_first_stage_current += shunt_cal_half_cycle_total_current;

                /* get min max */
                shunt_cal_first_stage_min = MIN(shunt_cal_first_stage_min, shunt_cal_half_cycle_total_current);
                shunt_cal_first_stage_max = MAX(shunt_cal_first_stage_max,shunt_cal_half_cycle_total_current);

            }
            else if(shunt_cal_first_stage_counter == (SHUNT_CAL_FIRST_STAGE_HALF_CYCLES + 11))
            {
                /* remove the extremes */
                shunt_cal_first_stage_current -= (shunt_cal_first_stage_min + shunt_cal_first_stage_max);

                /* base on the 1st stage 20 amp input, calculate what the 2nd stage current should be  */
                shunt_cal_first_stage_ratio_15A = (shunt_cal_first_stage_current << 1) - (shunt_cal_first_stage_current >> 3);
                shunt_cal_first_stage_ratio_20A = (shunt_cal_first_stage_current << 1) + (shunt_cal_first_stage_current >> 1);
#if RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_25_30A
                shunt_cal_first_stage_ratio_25A = (shunt_cal_first_stage_current << 1) + shunt_cal_first_stage_current + (shunt_cal_first_stage_current >> 3);
#endif
#if ((RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_25_30A) ||\
     (RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_30A))
                shunt_cal_first_stage_ratio_30A = (shunt_cal_first_stage_current << 2) - (shunt_cal_first_stage_current >> 2);
#endif
                shunt_cal_first_stage_pass = TRUE;
            }
            else if(shunt_cal_first_stage_counter > (SHUNT_CAL_FIRST_STAGE_HALF_CYCLES + 11))
            {
                shunt_cal_first_stage_counter = (SHUNT_CAL_FIRST_STAGE_HALF_CYCLES + 12);
            }
        }

        /* second stage */
        if((shunt_cal_first_stage_pass == TRUE) && (shunt_cal_half_cycle_total_current > SHUNT_CAL_SECOND_STAGE_CURRENT_MIN))
        {
            //Clear Overcurrent counter, so the breaker have already been calibrated will not trip.
            load_current_set_overcurrent_counter(0);

            /* Wait for the data to be stabilized */
            shunt_cal_wait_cycles++;
            if(shunt_cal_wait_cycles > SHUNT_CAL_WAIT_HALF_CYCLES)
            {
                shunt_cal_wait_cycles = SHUNT_CAL_WAIT_HALF_CYCLES;
                shunt_cal_second_stage_counter++;

                /* add 10 consecutive half cycle values */
                if(shunt_cal_second_stage_counter <= 10)
                {
                    shunt_cal_second_stage_current += shunt_cal_half_cycle_total_current;

                    /* get min max */
                    shunt_cal_second_stage_min = MIN(shunt_cal_second_stage_min, shunt_cal_half_cycle_total_current);
                    shunt_cal_second_stage_max = MAX(shunt_cal_second_stage_max, shunt_cal_half_cycle_total_current);
                }

                if(shunt_cal_second_stage_counter == 10)
                {
                    /* remove the extremes */
                    shunt_cal_second_stage_current -= (shunt_cal_second_stage_min + shunt_cal_second_stage_max);

                    /* tolerance */
                    shunt_cal_second_stage_total_diff_threshold1 = (shunt_cal_second_stage_current >> 4) + (shunt_cal_second_stage_current >> 6); //7.8%??
                    shunt_cal_second_stage_total_diff_threshold2 = (shunt_cal_second_stage_current >> 4); //6.25%??

                    /* if it's a 15A calibration current */
                    if((shunt_cal_first_stage_ratio_15A > (shunt_cal_second_stage_current - shunt_cal_second_stage_total_diff_threshold1))&&
                       (shunt_cal_first_stage_ratio_15A < (shunt_cal_second_stage_current + shunt_cal_second_stage_total_diff_threshold1))&&
                       (shunt_cal_second_stage_current > shunt_cal_current_table_15A[0])&&
                       (shunt_cal_second_stage_current < shunt_cal_current_table_15A[SHUNT_CAL_TABLE_SIZE - 1])
                      )
                    {
                        /* determine calibration factor */
                        for(shunt_cal_index = 1; shunt_cal_index < SHUNT_CAL_TABLE_SIZE; shunt_cal_index++)
                        {
                            if(shunt_cal_second_stage_current < shunt_cal_current_table_15A[shunt_cal_index])
                            {
                                /* write factors */
                                shunt_cal_write_factors(LOAD_CURRENT_RATING_15A, shunt_cal_index);
                                break;
                            }
                        }
                    }
                    /* if it's a 20A calibration current */
                    else if((shunt_cal_first_stage_ratio_20A > (shunt_cal_second_stage_current - shunt_cal_second_stage_total_diff_threshold1))&&
                            (shunt_cal_first_stage_ratio_20A < (shunt_cal_second_stage_current + shunt_cal_second_stage_total_diff_threshold1))&&
                            (shunt_cal_second_stage_current > shunt_cal_current_table_20A[0])&&
                            (shunt_cal_second_stage_current < shunt_cal_current_table_20A[SHUNT_CAL_TABLE_SIZE - 1])
                           )
                    {
                        /* determine calibration factor */
                        for(shunt_cal_index = 1; shunt_cal_index < SHUNT_CAL_TABLE_SIZE; shunt_cal_index++)
                        {
                            if(shunt_cal_second_stage_current < shunt_cal_current_table_20A[shunt_cal_index])
                            {
                                /* write factors */
                                shunt_cal_write_factors(LOAD_CURRENT_RATING_20A, shunt_cal_index);
                                break;
                            }
                        }
                    }
#if RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_25_30A
                    /* if it's a 25A calibration current */
                    else if((shunt_cal_first_stage_ratio_25A > (shunt_cal_second_stage_current - shunt_cal_second_stage_total_diff_threshold2))&&
                            (shunt_cal_first_stage_ratio_25A < (shunt_cal_second_stage_current + shunt_cal_second_stage_total_diff_threshold2))&&
                            (shunt_cal_second_stage_current > shunt_cal_current_table_25A[0])&&
                            (shunt_cal_second_stage_current < shunt_cal_current_table_25A[SHUNT_CAL_TABLE_SIZE - 1])
                           )
                    {
                        /* determine calibration factor */
                        for(shunt_cal_index = 1; shunt_cal_index < SHUNT_CAL_TABLE_SIZE; shunt_cal_index++)
                        {
                            if(shunt_cal_second_stage_current < shunt_cal_current_table_25A[shunt_cal_index])
                            {
                                /* write factors */
                                shunt_cal_write_factors(LOAD_CURRENT_RATING_25A, shunt_cal_index);
                                break;
                            }
                        }
                    }
#endif
#if ((RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_25_30A) ||\
     (RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_30A))
                    /* if it's a 30A calibration current */
                    else if((shunt_cal_first_stage_ratio_30A > (shunt_cal_second_stage_current - shunt_cal_second_stage_total_diff_threshold2))&&
                            (shunt_cal_first_stage_ratio_30A < (shunt_cal_second_stage_current + shunt_cal_second_stage_total_diff_threshold1))&&
                            (shunt_cal_second_stage_current > shunt_cal_current_table_30A[0])&&
                            (shunt_cal_second_stage_current < shunt_cal_current_table_30A[SHUNT_CAL_TABLE_SIZE - 1])
                           )
                    {
                        /* determine calibration factor */
                        for(shunt_cal_index = 1; shunt_cal_index < SHUNT_CAL_TABLE_SIZE; shunt_cal_index++)
                        {
                            if(shunt_cal_second_stage_current < shunt_cal_current_table_30A[shunt_cal_index])
                            {
                                /* write factors */
                                shunt_cal_write_factors(LOAD_CURRENT_RATING_30A, shunt_cal_index);
                                break;
                            }
                        }
                    }
#endif
                    else
                    {
                        shunt_cal_second_stage_counter = 0;
                        shunt_cal_second_stage_current = 0;
                    }
                }
            }
        }
        else
        {
            shunt_cal_wait_cycles = 0;
            shunt_cal_second_stage_counter = 0;
            shunt_cal_second_stage_current = 0;
        }
    }

    return;
}

void shunt_cal_write_factors(load_current_rating_t rating, u8 factor)
{
    u64 data_to_write = 0;

    __disable_irq();

    /* add in the complement of the rating to the u64 variable */
    data_to_write += (u8)(rating ^ 0xFF);
    data_to_write = data_to_write<<8;

    /* add in the rating */
    data_to_write += (u8)rating;

    /* write rating and it's complement*/
    flash_write64(&data_to_write, (u64*)FLASH_BREAKER_CONFIG_RATING_ADDRESS);

    /* clear variable */
    data_to_write = 0;

    /* add in the complement of the calibration factor to the u64 variable */
    data_to_write += (u8)(factor ^ 0xFF);
    data_to_write = data_to_write<<8;

    /* add in the calibration factor */
    data_to_write += (u8)factor;

    /* write calibration factor and it's complement*/
    flash_write64(&data_to_write, (u64*)FLASH_BREAKER_CONFIG_CALIBRATION_ADDRESS);

    /* set flag true indicating calibration complete */
    shunt_cal_complete_OK = TRUE;

    __enable_irq();

    return;
}

void shunt_cal_check_and_set_rating_and_factor(void)
{
    u8* pointer = (u8*)FLASH_BREAKER_CONFIG_RATING_ADDRESS;

    /* check rating */
    if((*pointer) == ((*(pointer+1)) ^ 0xFF))
    {
        if(*pointer == LOAD_CURRENT_RATING_15A)
        {
            breaker_rating = LOAD_CURRENT_RATING_15A;
            shunt_cal_rating_checked_OK = TRUE;
        }
        else if (*pointer == LOAD_CURRENT_RATING_20A)
        {
            breaker_rating = LOAD_CURRENT_RATING_20A;
            shunt_cal_rating_checked_OK = TRUE;
        }
#if RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_25_30A
        else if (*pointer == LOAD_CURRENT_RATING_25A)
        {
            breaker_rating = LOAD_CURRENT_RATING_25A;
            shunt_cal_rating_checked_OK = TRUE;
        }
#endif
#if ((RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_25_30A) ||\
     (RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_30A))
        else if (*pointer == LOAD_CURRENT_RATING_30A)
        {
            breaker_rating = LOAD_CURRENT_RATING_30A;
            shunt_cal_rating_checked_OK = TRUE;
        }
#endif
    }

    /* check calibration factor */
    pointer = (u8*)FLASH_BREAKER_CONFIG_CALIBRATION_ADDRESS;

    if((*pointer) == ((*(pointer+1)) ^ 0xFF))
    {
        if((*pointer > 0) && (*pointer < SHUNT_CAL_TABLE_SIZE))
        {
            breaker_cal_factor = *pointer;
            shunt_cal_factor_checked_OK = TRUE;
        }
    }

    return;
}

void shunt_cal_run_calibration(void)
{
    /* run calibration if calibration has not run within SHUNT_CAL_PERIOD_SECONDS */
    if((!shunt_cal_complete_OK) && (shunt_cal_seconds < SHUNT_CAL_PERIOD_SECONDS))
    {
        /* run calibration */
        shunt_cal_and_config();

        /* if calibration has completed */
        if(shunt_cal_complete_OK)
        {
            /* check and set the variables */
            shunt_cal_check_and_set_rating_and_factor();

            /* set over current count so breaker will trip sooner */
            load_current_set_overcurrent_counter(SHUNT_CAL_OVER_CURRENT_PRECOUNT);
        }
    }

    return;
}

void shunt_cal_adjust_current_based_on_cal_factor(void)
{
    u32 current = load_current_get_total_current();
    u32 adjusted_current = 0;

    if((breaker_rating == LOAD_CURRENT_RATING_15A) || (breaker_rating == LOAD_CURRENT_RATING_20A)
#if RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_25_30A
       || (breaker_rating == LOAD_CURRENT_RATING_25A)
#endif
#if ((RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_25_30A) ||\
    (RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_30A))
       || (breaker_rating == (breaker_rating == LOAD_CURRENT_RATING_25A))
#endif
       )
    {
        switch (breaker_cal_factor)
        {
        case 1:
            /* increase 25% */
            adjusted_current = (current) +
                               (current >> 2);

            break;

        case 2:
            /* increase 18.75% */
            adjusted_current = (current) +
                               (current >> 3) +
                               (current >> 4);

            break;

        case 3:
            /* increase 12.5% */
            adjusted_current = (current) +
                               (current >> 3);

            break;

        case 4:
            /* increase 6.25% */
            adjusted_current = (current) +
                               (current >> 4);

            break;

        case 5:
            /* no change */
            adjusted_current = (current);

            break;

        case 6:
            /* decrease 6.25% */
            adjusted_current = (current) -
                               (current >> 4);

            break;

        case 7:
            /* decrease 12.5% */
            adjusted_current = (current) -
                               (current >> 3);

            break;

        case 8:
            /* decrease 15.625% */
            adjusted_current = (current) -
                               (current >> 3) -
                               (current >> 5);

            break;

        case 9:
            /* decrease 18.75% */
            adjusted_current = (current) -
                               (current >> 3) -
                               (current >> 4);

            break;

        default:
            adjusted_current = current;
            break;
        }
    }

    load_current_set_total_current(adjusted_current);

    return ;
}

inline bool shunt_cal_complete(void)
{
    return (shunt_cal_complete_OK);
}

inline load_current_rating_t shunt_cal_get_breaker_rating(void)
{
    return breaker_rating;
}

inline u8 shunt_cal_get_breaker_cal_factor(void)
{
    return breaker_cal_factor;
}

inline bool shunt_cal_rating_and_factor_check_ok(void)
{
    return (shunt_cal_rating_checked_OK && shunt_cal_factor_checked_OK);
}
