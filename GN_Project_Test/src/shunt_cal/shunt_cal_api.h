#ifndef __SHUNT_CAL_API_H
#define __SHUNT_CAL_API_H
/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 **************************************************************************************************
 *  Written by:         Hank Sun
 *                      Eaton Electrical
 *                      1000 Cherrington Parkway
 *                      Pittsburgh, PA  15108-4312
 *                      (412) 893-3300
 *//**
 * @brief External interface for the shunt_cal component.
 *
 * @file shunt_cal_api.h
 * @ingroup shunt_cal
 *
 *//*
 *
 **************************************************************************************************/

#include "types.h"
#include "load_current_api.h"


#define SHUNT_CAL_FIRST_STAGE_HALFCYCLES                            (60)   //0.5s
#define SHUNT_CAL_PRE_HALFCYCLES                                    (120)  //1s waiting time for load current to be stable

#define SHUNT_CAL_1A_CENTER_CURRENT                     (156.8) //centered at 1Amp DC 0.525 miliohm shunt
#define SHUNT_CAL_15A_CENTER_CURRENT                    (u32)(SHUNT_CAL_1A_CENTER_CURRENT * 37.5)
#define SHUNT_CAL_20A_CENTER_CURRENT                    (u32)(SHUNT_CAL_1A_CENTER_CURRENT * 50.)
#define SHUNT_CAL_25A_CENTER_CURRENT                    (u32)(SHUNT_CAL_1A_CENTER_CURRENT * 62.5)
#define SHUNT_CAL_30A_CENTER_CURRENT                    (u32)(SHUNT_CAL_1A_CENTER_CURRENT * 75.)
#define SHUNT_CAL_OVER_CURRENT_PRECOUNT                 (SHUNT_CAL_PRE_HALFCYCLES * 48)

#define SHUNT_CAL_FIRST_STAGE_CURRENT_LOW               (u32)(SHUNT_CAL_1A_CENTER_CURRENT * 20. * 0.75)
#define SHUNT_CAL_FIRST_STAGE_CURRENT_HIGH              (u32)(SHUNT_CAL_1A_CENTER_CURRENT * 20. * 1.25)
#define SHUNT_CAL_SECOND_STAGE_CURRENT_MIN              (u32)(SHUNT_CAL_1A_CENTER_CURRENT * 37.5 * 0.7)

#define SHUNT_CAL_TABLE_SIZE                            (10)
#define SHUNT_CAL_HALF_TABLE_SIZE                       (5)

#define SHUNT_CAL_ONE_SECOND_COUNTS                     (120)
#define SHUNT_CAL_PERIOD_SECONDS                        (600)

#define SHUNT_CAL_FIRST_STAGE_HALF_CYCLES               (60)
#define SHUNT_CAL_WAIT_HALF_CYCLES                      (120)

#warning consider moving this to load voltage component or main
#define SHUNT_CAL_MIN_VOLTAGE                           ((u32)(40.0) * LOAD_VOLTAGE_RATIO) //40 volts
#define SHUNT_CAL_MAX_VOLTAGE                           ((u32)(70.0) * LOAD_VOLTAGE_RATIO) //70 volts


/**************************************************************************************************/
/**
 * @brief Initialize shunt calibration component
 *
 * Initializes globals and assign task to task manager. Also checks if the breaker is already
 * calibrated.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void shunt_cal_init(void);

/**************************************************************************************************/
/**
 * @brief This function runs the calibration if condition permits
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void shunt_cal_run_calibration(void);

/**************************************************************************************************/
/**
 * @brief Adjust the total amount of current
 *
 * Based on the calibration factor, adjust the total amount of current every half cycle.
 *
 * This function should only be called if the breaker is calibrated.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void shunt_cal_adjust_current_based_on_cal_factor(void);

/**************************************************************************************************/
/**
 * @brief check if calibration has completed or not.
 *
 * @return TRUE if complete
 *         FALSE if not complete
 *
 * @exception none
 *
 **************************************************************************************************/
bool shunt_cal_complete(void);

/**************************************************************************************************/
/**
 * @brief Get breaker rating
 *
 * @return LOAD_CURRENT_RATING_15A,
 *         LOAD_CURRENT_RATING_20A
 *         LOAD_CURRENT_RATING_25A
 *         LOAD_CURRENT_RATING_30A
 *
 * @exception none
 *
 **************************************************************************************************/
load_current_rating_t shunt_cal_get_breaker_rating(void);

/**************************************************************************************************/
/**
 * @brief Get calibration factor
 *
 * @return a u8 number that should be between 1 and 9 (including 1 and 9)
 *
 * @exception none
 *
 **************************************************************************************************/
u8 shunt_cal_get_breaker_cal_factor(void);

/**************************************************************************************************/
/**
 * @brief check if the system has checked rating and calibration factors, and are valid values.
 *
 * @return TRUE if checked
 *         FALS if not checked
 *
 * @exception none
 *
 **************************************************************************************************/
bool shunt_cal_rating_and_factor_check_ok(void);



#endif /*__SHUNT_CAL_API_H */

