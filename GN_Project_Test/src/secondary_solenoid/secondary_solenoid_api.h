#ifndef _SECONDARY_SOLENOID_API_H_
#define _SECONDARY_SOLENOID_API_H_

/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2022
 *                      All rights reserved
 *
 **************************************************************************************************
 *  Written by:         James Frances
 *                      Commercial & Residential Distribution Solutions (CRDS), Eaton
 *						1000 Cherrington Parkway
 *						Moon Township, PA 15108
 *						mobile: (724) 759-5500
 *//**
 * @brief External interface for the secondary solenoid component.
 *
 * @file secondary_solenoid_api.h
 * @ingroup secondary_solenoid
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "gpio_api.h"

#define TIMES_TO_RETRY_STUCK					3
#define TIMES_TO_RESET_UNEXPECTED_CHANGE		3


#define DELAY_CYCLES_FOR_SS_CHANGE_DETECTION	4
//Useing a 10second delay between requests to prevent burnout.
//#define DELAY_CYCLES_FOR_SS_NEXT_REQUEST			600
#define DELAY_HALF_CYCLES_FOR_SS_NEXT_REQUEST	1200

#define SS_SET_OPEN			gpio_set_output_pin(SS_OPEN_EN_GPIO_Port,SS_OPEN_EN_Pin)
#define SS_RESET_OPEN		gpio_reset_output_pin(SS_OPEN_EN_GPIO_Port,SS_OPEN_EN_Pin)
#define SS_SET_CLOSE		gpio_set_output_pin(SS_CLOSE_EN_GPIO_Port,SS_CLOSE_EN_Pin)
#define SS_RESET_CLOSE		gpio_reset_output_pin(SS_CLOSE_EN_GPIO_Port,SS_CLOSE_EN_Pin)

typedef enum
{
	SS_UNKNOWN = 0,
    SS_OPEN,
	SS_OPENING,
    SS_CLOSED,
	SS_CLOSING,
    SS_STUCK_OPEN,
	SS_STUCK_CLOSED,

} ss_status_t;


/**************************************************************************************************/
/**
 * @brief secondary solenoid main task.
 *
 * runs in main loop and checks to confirm the solenoid transitioned correctly and processes new commands
 *
 * @return bool: Error detected/ Trip Primary
 *
 * @exception none
 *
 **************************************************************************************************/
bool secondary_solenoid_main_task(void);

/**************************************************************************************************/
/**
 * @brief Initialize secondary solenoid component.
 *
 * This function initializes the secondary solenoid component and add task to task manager.
 *
 *
 * @return none.
 *
 * @exception none
 *
 **************************************************************************************************/
void secondary_solenoid_init(void);

/**************************************************************************************************/
/**
 * @brief gets the reported state of the secondary solenoid.
 *
 * @param[in]  void
 *
 * @return ss_status_t
 *
 * @exception none
 *
 **************************************************************************************************/
ss_status_t secondary_solenoid_get_status(void);

/**************************************************************************************************/
/**
 * @brief requests that the secondary solenoid be opened if it isn't already
 *
 * @param[in] void
 *
 * @return void
 *
 * @exception none
 *
 **************************************************************************************************/
void request_open_secondary_solenoid(void);

/**************************************************************************************************/
/**
 * @brief requests that the secondary solenoid be closed if it isn't already
 *
 * @param[in] void
 *
 * @return void
 *
 * @exception none
 *
 **************************************************************************************************/
void request_close_secondary_solenoid(void);

/**************************************************************************************************/
/**
 * @brief requests that the secondary solenoid be toggled
 *
 * @param[in] void
 *
 * @return void
 *
 * @exception none
 *
 **************************************************************************************************/
void request_toggle_secondary_solenoid(void);

#endif /* _SECONDARY_SOLENOID_API_H_ */
