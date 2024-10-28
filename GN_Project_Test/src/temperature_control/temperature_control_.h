#ifndef _TEMPERATURE_CONTROL_H_
#define _TEMPERATURE_CONTROL_H_
/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2024
 *                      All rights reserved
 *
 **************************************************************************************************
 *  Written by:         Ming Wu
 *                      Commercial & Residential Distribution Solutions (CRDS), Eaton
 *						1000 Cherrington Parkway
 *						Moon Township, PA 15108
 *//**
 * @brief interface for the temperature component.
 *
 * @file temperature_control_.h
 * @ingroup temperature_control
 *
 *//*
 *
 **************************************************************************************************/
#include "temperature_control_internal.h"

/**************************************************************************************************/
/**
 * @brief initialize temperature control
 *
 * initialize globals.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void temperature_control_initialize_globals(void);

/**************************************************************************************************/
/**
 * @brief ZCD task for temperature control.
 *
 * This task counts up and determine the right time to run temperature_control.
 *
 *
 * @param[in] it - interrupt to run the task. (ZCD)
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void temperature_control_zcd_callback(u32 it);

/**************************************************************************************************/
/**
 * @brief temperature_control task
 *
 * This task runs the whole temperature_control sequence that check STM32 temperature.
 *
 *
 * @param[in] it - interrupt to run the task.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void temperature_control_it_task(u32 it);

/**************************************************************************************************/
/**
 * @brief Clear global variables used in temperature_control component
 *
  *
 * @param[in] none
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void clear_temperature_control_variables();
#endif /* _TEMPERATURE_CONTROL_H_ */
