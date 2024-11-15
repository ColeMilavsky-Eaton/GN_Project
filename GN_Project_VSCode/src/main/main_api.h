#ifndef _MAIN_API_H
#define _MAIN_API_H
/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 **************************************************************************************************
 *  Written by:         Aaron Joseph
 *                      Eaton Electrical
 *                      1000 Cherrington Parkway
 *                      Pittsburgh, PA  15108-4312
 *                      (412) 893-3300
 *//**
 * @brief External interface for the main component.
 *
 * @file main_api.h
 * @ingroup main
 *
 *//*
 *
 **************************************************************************************************/
#include "../../src/sys/firmware.config"
#include "types.h"
#include "utils.h"

//#warning this needs to be moved. only here to get code to compile. it is also defined in breaker core
#define MAX_NUM_POLES                                   (3)

typedef enum
{
    PTT_NO_ACTION = 0,
    SELF_TEST_REQUESTED = 1,
    DISPLAY_TRIP_CODE_REQUESTED = 2,
    TRIP_LOG_LED_XFER_REQUESTED = 3,

} main_ptt_status_t;

typedef struct
{
    main_ptt_status_t status;
    bool PTT_pressed_since_startup;

}main_ptt_button_t;

/*************************************************************************************************/
/**
 * @brief Sets the end of factory anchor checked flag.
 *
 * This function sets the end of factory anchor checked flag.
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void main_set_end_of_factory_anchor_checked(void);

/*************************************************************************************************/
/**
 * @brief Sets the end of factory anchor flag.
 *
 * This function sets the end of factory anchor flag.
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void main_set_end_of_factory_anchored(void);

/*************************************************************************************************/
/**
 * @brief Sets the reset anchor flag
 *
 * This function sets the reset anchor flag.
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void main_requeset_reset_anchor(void);

/*************************************************************************************************/
/**
 * @brief Gets current plus bias.
 *
 * @return current plus bias.
 *
 * @exception none
 *
 *************************************************************************************************/
u32 main_get_current_plus_bias(void);

/*************************************************************************************************/
/**
 * @brief Gets startup indicated flag
 *
 * @return startup_indicated_flag.
 *
 * @exception none
 *
 *************************************************************************************************/
bool get_startup_indicated_flag(void);

/*************************************************************************************************/
/**
 * @brief Gets startup logged flag
 *
 * @return startup_logged_flag.
 *
 * @exception none
 *
 *************************************************************************************************/
bool get_startup_logged_flag(void);

 #endif /* _MAIN_API_H */

