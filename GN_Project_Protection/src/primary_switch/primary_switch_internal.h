#ifndef _PRIMARY_SWITCH_INTERNAL_H_
#define _PRIMARY_SWITCH_INTERNAL_H_
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
 * @brief internal components for the primary switch monitoring component.
 *
 * @file primary_switch_internal.h
 * @ingroup primary_switch
 *
 *//*
 *
 **************************************************************************************************/
#include "primary_switch_api.h"
#include "task_manager_api.h"

// TODO: Values are from one of the breaker, need verification

#define PS_CLOSED_STATE_ADC_MIN 400  //Min found during Haina test was 603
#define PS_CLOSED_STATE_ADC_MAX 1300 //Max found was 1091
#define PS_CLOSED_STATE_ADC_MIN_ALT 2800 //Min detected was 3033
#define PS_CLOSED_STATE_ADC_MAX_ALT 3600 //Max was 3408

// OFF(OPEN) state 670-680mV
#define PS_OPEN_STATE_ADC_MIN  1880 //Min was 2001
#define PS_OPEN_STATE_ADC_MAX  2000
#define PS_OPEN_STATE_ADC_MIN_ALT  2001
#define PS_OPEN_STATE_ADC_MAX_ALT  2169 //Max was 2162

// Trip State 510-520mV
#define PS_TRIP_STATE_ADC_MIN 1400 //1673
#define PS_TRIP_STATE_ADC_MAX 1879 // Max was 1757
#define PS_TRIP_STATE_ADC_MIN_ALT 2170 //Min was 2177
#define PS_TRIP_STATE_ADC_MAX_ALT 2700 //Max 2398

#endif /* _PRIMARY_SWITCH_INTERNAL_H_ */
