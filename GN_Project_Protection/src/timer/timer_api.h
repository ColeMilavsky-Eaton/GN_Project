#ifndef _TIMER_API_H
#define _TIMER_API_H
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
 * @brief External interface for the timer component.
 *
 * @file timer_api.h
 * @ingroup timer
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "firmware.config"

/* at 16 MHz, pre-scaler 64, this results in around 2 half cycle timeout. */
#define ZCD_DEFAULT_TIMEOUT (4160) //130*16*2

#define SOFTWARE_ZCD_DEFAULT_TIMEOUT (U16_MAX)

/* at 16 MHz, pre-scaler 64, this results in 0.05s timeout. */
#define DATA_COLLECTION_DEFAULT_TIMEOUT     (0x30D4)

/* at 16 MHz, pre-scaler 64, 125% of one half cycle */
#define PLD_DEFAULT_TIMEOUT (2496) //130*16*1.2

/* at 16MHz, pre-scaler 64, and 10 out of the 16 half cycles */
#define SS_DEFAULT_TIMEOUT (1300) //(130 * 16) * (10 / 16)
inline void timer_delay(u64 delay_counts);


/*************************************************************************************************/
/**
 * @brief Initialize the zero crossing detection timer.
 *
 * Setting up zero crossing detection timer. Used to determine the 16 interrupts between zero
 * crossings and triggers a timmer interrupt when no zero crossing is detected for a duration of
 * time
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void timer_zcd_timer_init(void);

/*************************************************************************************************/
/**
 * @brief Initialize the data collection timer interrupt
 *
 * initialize data collection interrupt auto reload to be the largest value and should be updated
 * when a ZCD occurs.
 *
 * @return none
 *
 * @exception none
 *
 *
 *************************************************************************************************/
void timer_data_collection_timer_init(void);

#if SOFTWARE_ZCD_CONFIGURATION == SOFTWARE_ZCD_ENABLED
/*************************************************************************************************/
/**
 * @brief Initialize the software interrupt timer
 *
 * initialize software interrupt timer auto reload to be the largest value and should be updated
 * when a hardware ZCD occurs.
 * The setting for the timer is the same as the ZCD timer and data collection timer
 *
 * @return none
 *
 * @exception none
 *
 *
 *************************************************************************************************/
void software_ZCD_timer_init(void);
#endif

#if defined POWER_LOSS_DETECTION
/*************************************************************************************************/
/**
 * @brief Initialize the software interrupt timer
 *
 * initialize software interrupt timer auto reload to be the largest value and should be updated
 * when a power loss detected.
 * The setting for the timer is the same as the ZCD timer and data collection timer
 *
 * @return none
 *
 * @exception none
 *
 *
 *************************************************************************************************/
void power_loss_detection_timer_init (void);
#endif

/*************************************************************************************************/
/**
 * @brief Initialize the secondary switch backup timer
 *
 * initialize timer to confirm secondary pulse does not overrun if an unexpected zcd occurs.
 *
 * @return none
 *
 * @exception none
 *
 *
 *************************************************************************************************/
void ss_backup_timer_init(void);

 #endif /* _TIMER_API_H */
