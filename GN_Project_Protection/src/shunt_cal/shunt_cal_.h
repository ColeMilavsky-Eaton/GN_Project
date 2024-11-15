#ifndef __SHUNT_CAL_H
#define __SHUNT_CAL_H
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
 * @brief Internal (to the component) interface for the shunt_cal component.
 *
 * @file shunt_cal_.h
 * @ingroup shunt_cal
 *
 *//*
 *
 **************************************************************************************************/

#include "shunt_cal_internal.h"


/**************************************************************************************************/
/**
 * @brief Initializes global variables.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void shunt_cal_initialize_globals(void);

/**************************************************************************************************/
/**
 * @brief callback function when zcd occurs.
 *
 * This function should be assigned to task manager and should run every time zcd occurs. The main
 * purpose for this callback is to count seconds for shunt calibration component.
 *
 * @param[in] it interrupts to run
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void shunt_cal_zcd_callback(u32 it);

/**************************************************************************************************/
/**
 * @brief Running the calibration process.
 *
 * This is the main function for calibration. The calibration process goes through two steps.
 * First step current is the same for all ratings. Second step current will be different based on
 * which rating of breaker to calibrate to.
 *
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void shunt_cal_and_config(void);


/**************************************************************************************************/
/**
 * @brief Write rating and calibration factors.
 *
 * write rating and calibration factors to a specific location in flash
 *
 * @param[in] rating LOAD_CURRENT_RATING_15A,
 *                   LOAD_CURRENT_RATING_20A
 *                   LOAD_CURRENT_RATING_25A
 *                   LOAD_CURRENT_RATING_30A
 *
 * @param[in] factor calibration factor. should be between 1 and 9 (including 1 and 9)
 *
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void shunt_cal_write_factors(load_current_rating_t rating, u8 factor);

/**************************************************************************************************/
/**
 * @brief Checks for rating and calibration factor in flash.
 *
 * Checks for rating and calibration factor in flash and stores in a global variable
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void shunt_cal_check_and_set_rating_and_factor(void);

#endif /*__SHUNT_CAL_H*/

