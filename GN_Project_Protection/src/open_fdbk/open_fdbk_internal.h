#ifndef _OPEN_FDBK_INTERNAL_H_
#define _OPEN_FDBK_INTERNAL_H_
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
 * @brief Internal interface for the open feedback component.
 *
 * @file open_fdbk_internal.h
 * @ingroup open_fdbk
 *
 *//*
 *
 **************************************************************************************************/

#include "open_fdbk_api.h"
#include "task_manager_api.h"
#include "led_api.h"

#define OPEN_FDBK_PRIMARY_IT		TSK_MAN_IT_8
#define OPEN_FDBK_SECONDARY_120_IT	TSK_MAN_IT_19
#define OPEN_FDBK_SECONDARY_180_IT	TSK_MAN_IT_24
#define OPEN_FDBK_SECONDARY_240_IT	TSK_MAN_IT_29 //Secondary leading by 120 also looks like lagging by 240


#endif /* _OPEN_FDBK_INTERNAL_H_ */
