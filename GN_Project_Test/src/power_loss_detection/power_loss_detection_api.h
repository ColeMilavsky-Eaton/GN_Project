#ifndef _POWER_LOSS_DETECTION_API_H_
#define _POWER_LOSS_DETECTION_API_H_
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
 * @brief API for the power_loss_detection monitoring component.
 *
 * @file power_loss_detection_api.h
 * @ingroup power_loss_detection
 *
 *//*
 *
 **************************************************************************************************/
#include "gfci_api.h"
//#define NUM_SAMPLES_PER_CYCLE                          (32)

typedef struct
{
    u32 number_of_poles;                // number of poles
    u32* load_voltage_data_source1;     // location where new data will be stored for pole 1
    u32* load_voltage_data_source2;     // location where new data will be stored for pole 2
    u32* load_voltage_data_source3;     // location where new data will be stored for pole 3

} power_loss_detection_init_t;

/**************************************************************************************************/
/**
 * @brief Initialize power_loss_detection component.
 *
 * This function initializes the power_loss_detection component and add task to task manager.
 *
 * @return none.
 *
 * @exception none
 *
 **************************************************************************************************/
void power_loss_detection_init(power_loss_detection_init_t);

/**************************************************************************************************/
/**
 * @brief Returns the power loss signal
 *
 * This function will return the possibility level of the power loss detection.
 *
 * @return uint32_t, higher the number, higher the possibility
 *
 * @exception none
 *
 **************************************************************************************************/
u32 get_power_loss_detection_level(void);

#endif /* _OWER_LOSS_DETECTION_API_H_ */
