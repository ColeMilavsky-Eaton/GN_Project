#ifndef SRC_PRIMARY_SWITCH_PRIMARY_SWITCH_H_
#define SRC_PRIMARY_SWITCH_PRIMARY_SWITCH_H_
/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2023
 *                      All rights reserved
 *
 **************************************************************************************************
 *  Written by:         Ming Wu
 *                      Innovation
 *                      Commercial & Residential Distribution Solutions (CRDS), Eaton
 *                      1000 Cherrington Parkway
 *                      Moon Township, PA 15108
 *//**
 * @brief header file for the primary switch monitoring component.
 *
 * @file primary_switch_.h
 * @ingroup primary_switch
 *
 *//*
 *
 **************************************************************************************************/
#include "primary_switch_internal.h"

typedef struct
{
	u8 open_counter;
	u8 closed_counter;
	u8 trip_counter;
	u8 failed_counter;
	pswitch_status_t state;
	pswitch_status_t previous_state;
} psw_t;

/**************************************************************************************************/
/**
 * @brief primary switch task.
 *
 * checks for change in state of the primary switch and acts accordingly
 * @return none.
 *
 * @exception none
 *
 **************************************************************************************************/
void primary_switch_zcd_callback(u32 it);

/**************************************************************************************************/
/**
 * @brief primary switch get state from the ADC value.
 *
 * checks for state of the primary switch from the hall sensor
 *
 * @param[in]  Primary switch hall sensor ADC value
 * @return Primary switch state .
 *
 * @exception none
 *
 **************************************************************************************************/
pswitch_status_t primary_switch_get_state_from_level(u32);

/**************************************************************************************************/
/**
 * @brief Primary switch state debouncing.
 *
 * Changes debouncing counter from one primary switch state check
 * @return none.
 *
 * @exception none
 *
 **************************************************************************************************/
void primary_switch_debouncing(void);

#endif /*  _PRIMARY_SWITCH_H_ */
