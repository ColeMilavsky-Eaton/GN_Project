#ifndef _SECONDARY_SOLENOID__H
#define _SECONDARY_SOLENOID__H
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
 * @brief interface for the secondary solenoid component.
 *
 * @file secondary_solenoid_api.h
 * @ingroup secondary_solenoid
 *
 *//*
 *
 **************************************************************************************************/
#include "secondary_solenoid_internal.h"

/* for diagnostics */
#define SECONDARY_SOLENOID_COMPONENT

typedef struct
{
	bool state_change_requested;
    ss_status_t requested_state;
    ss_status_t current_state;
    u8 retry_stuck_counter;
    u8 unexpected_change_counter;
    u8 wait_transition_detection;
	//#if defined ADD_SS_REQUEST_DELAY
    u16 wait_next_open_request;
    u16 wait_next_close_request;
	//#endif

} secondary_solenoid_t;


/**************************************************************************************************/
/**
 * @brief secondary solenoid task.
 *
 * checks for command to change state of secondary contacts open/close and aligns with peak voltage
 *
 * @return none.
 *
 * @exception none
 *
 **************************************************************************************************/
void secondary_solenoid_it_task(u32 it);

/**************************************************************************************************/
/**
 * @brief secondary solenoid manual close task.
 *
 * Will allow a reset of the primary handle to reset the secondaries as well.
 *
 * @return none.
 *
 * @exception none
 *
 **************************************************************************************************/
void secondary_solenoid_manual_close_task(void);

#endif /*_SECONDARY_SOLENOID__H */
