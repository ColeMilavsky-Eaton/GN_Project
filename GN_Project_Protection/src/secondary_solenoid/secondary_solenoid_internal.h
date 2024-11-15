#ifndef _SECONDARY_SOLENOID_INTERNAL_H_
#define _SECONDARY_SOLENOID_INTERNAL_H_

/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 **************************************************************************************************
 *  Written by:         James Frances
 *                      Commercial & Residential Distribution Solutions (CRDS), Eaton
 *						1000 Cherrington Parkway
 *						Moon Township, PA 15108
 *						mobile: (724) 759-5500
 *//**
 * @brief Internal interface for the secondary solenoid component.
 *
 * @file secondary_solenoid_internal.h
 * @ingroup secondary_solenoid
 *
 *//*
 *
 **************************************************************************************************/

#include "secondary_solenoid_api.h"
#include "task_manager_api.h"
#include "primary_switch_api.h"
#include "open_fdbk_api.h"
#include "timer_internal.h"

#define SS_START_OPEN_DRIVE_IT	TSK_MAN_IT_11
//#define SS_STOP_OPEN_DRIVE_IT	TSK_MAN_IT_16
#define SS_START_CLOSE_DRIVE_IT	TSK_MAN_IT_12
//#define SS_STOP_CLOSE_DRIVE_IT	TSK_MAN_IT_16
#define SS_STOP_DRIVE_IT		TSK_MAN_IT_16 //While these are the same I'm not going to separate them.

#if defined SAVE_SECONDARY_SOLENOID_STATE_IN_NVM
/**************************************************************************************************/
/**
 * @brief saves the expected state in NVM to allow for startup in desired state.
 *
 * This function will save the expected state in NVM to allow for retrieval during startup
 *
 * @param[in]: ss_status_t state: the state to be saved to NVM
 *
 * @return TRUE:  Save encountered an error
 * 		   FALSE: Completed successfully.
 *
 * @exception none
 *
 **************************************************************************************************/
bool save_ss_expected_state(ss_status_t state);

/**************************************************************************************************/
/**
 * @brief checks if the secondary solenoid is properly in the closed state.
 *
 * This function checks if the secondary solenoid is properly in the closed state.
 * It does not currently distinguish between different failures such as failure of only one pole
 * to close etc.
 *
 * @return ss_status_t the state saved in memory
 *
 * @exception none
 *
 **************************************************************************************************/
ss_status_t check_saved_ss_expected_state(void);
#endif

/**************************************************************************************************/
/**
 * @brief checks if the secondary solenoid is in a known state.
 *
 * This function checks if the secondary solenoid is in a known state.
 * If the open feedback cannot determine what state the solenoid is in it cannot reliably
 * determine what to do with the secondary solenoid.
 *
 * @return TRUE:  solenoid is properly in closed position.
 * 		   FALSE: solenoid is no in accepted closed state.
 *
 * @exception none
 *
 **************************************************************************************************/
bool check_ss_known(void);

/**************************************************************************************************/
/**
 * @brief checks if the secondary solenoid is properly in the closed state.
 *
 * This function checks if the secondary solenoid is properly in the closed state.
 * It does not currently distinguish between different failures such as failure of only one pole
 * to close etc.
 *
 * @return TRUE:  solenoid is properly in closed position.
 * 		   FALSE: solenoid is no in accepted closed state.
 *
 * @exception none
 *
 **************************************************************************************************/
bool check_ss_closed(void);

/**************************************************************************************************/
/**
 * @brief checks if the secondary solenoid is properly in the opened state.
 *
 * This function checks if the secondary solenoid is properly in the opened state.
 * It does not currently distinguish between different failures such as failure of only one pole
 * to open etc.
 *
 * @return TRUE: solenoid is properly in open position.
 * 		   FALSE: solenoid is no in accepted open state.
 *
 * @exception none
 *
 **************************************************************************************************/
bool check_ss_open(void);


#endif /*_SECONDARY_SOLENOID_INTERNAL_H_ */

