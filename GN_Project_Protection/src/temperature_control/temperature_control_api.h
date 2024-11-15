#ifndef _TEMPERATURE_CONTROL_API_H_
#define _TEMPERATURE_CONTROL_API_H_

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
 * @brief External interface for the temerature control component.
 *
 * @file temperature_control_api.h
 * @ingroup temperature_control
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "gpio_api.h"
#define OVERTEMP_TRIGGER_TEMP   110     // The temperature limit to trigger overtemp
#define OVERTEMP_QUIT_TEMP      95      // The temperature limit to quit overtemp
#define OVERTEMP_COUNTER_LIMIT  2250    // take one average per 16 cycles, so 2250 equals 10 minutes
/**************************************************************************************************/
/**
 * @brief Initialize temperature control component.
 *
 * This function initializes the temperature control component and add task to task manager.
 *
 * @return none.
 *
 * @exception none
 *
 **************************************************************************************************/
void temperature_control_init(void);
#endif /* _TEMPERATURE_CONTROL_API_H_ */
