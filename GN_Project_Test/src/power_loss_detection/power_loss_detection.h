#ifndef _POWER_LOSS_DETECTION_H_
#define _POWER_LOSS_DETECTION_H_
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
 * @brief header file for the power_loss_detection monitoring component.
 *
 * @file power_loss_detection.h
 * @ingroup power_loss_detection
 *
 *//*
 *
 **************************************************************************************************/
#include "power_loss_detection_internal.h"
#define POWER_LOSS_DETECTION_LEVEL_THRESHOLD 10
#define POWER_LOSS_DETECTION_ADC_VARIANCE_BAR 10
/**************************************************************************************************/
/**
 * @brief power loss detection task.
 *
 * checks the load voltages and compares them with references
 * @return none.
 *
 * @exception none
 *
 **************************************************************************************************/
void power_loss_detection_it_task(u32 it);

/**************************************************************************************************/
/**
 * @brief power loss detection to set possibility level.
 *
 * Set The power loss possibility levelr
 *
 * @param[in]  The power loss possibility level signal, higher the number, higher the possibility
 * @return none .
 *
 * @exception none
 *
 **************************************************************************************************/
void set_power_loss_detection_level(u32);
#endif /*  _POWER_LOSS_DETECTION_H_ */
