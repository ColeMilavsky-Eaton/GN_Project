#ifndef _OPEN_FDBK_API_H_
#define _OPEN_FDBK_API_H_

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
 * @brief External interface for the open feedback component.
 *
 * @file open_fdbk_api.h
 * @ingroup open_fdbk
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "gpio_api.h"

typedef enum
{
	PATH_UNKNOWN = 0, //may be transitioning between states.
    PATH_OPEN,
    PATH_CLOSED,
	PATH_1P_STUCK,

} fdbk_status_t;

/**************************************************************************************************/
/**
 * @brief Initialize open feedback component.
 *
 * This function initializes the open feedback component and adds it task to task manager.
 *
 * @return none.
 *
 * @exception none
 *
 **************************************************************************************************/
void open_fdbk_init(void);

/**************************************************************************************************/
/**
 * @brief open feedback main task.
 *
 * runs in main loop. may not be used
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void open_fdbk_main_task(void);

/**************************************************************************************************/
/**
 * @brief returns the current paths status
 *
 *
 * @return fdbk_status_t: last detected status
 *
 * @exception none
 *
 **************************************************************************************************/
fdbk_status_t get_path_status(void);

#endif /* _OPEN_FDBK_API_H_ */
