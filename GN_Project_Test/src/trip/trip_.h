#ifndef _TRIP__H
#define _TRIP__H
/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 **************************************************************************************************
 *  Written by:         Hank Sun
 *                      Eaton Electrical
 *                      1000 Cherrington Parkway
 *                      Pittsburgh, PA  15108-4312
 *                      (412) 893-3300
 *//**
 * @brief Internal (to the component) interface for the trip component.
 *
 * @file trip_.h
 * @ingroup trip
 *
 *//*
 *
 **************************************************************************************************/
#include "trip_internal.h"


#define NUM_SYNCED_TRIP_ATTEMPTS                            (3)
#define NUM_PULSES_PER_SYNCED_TRIP_ATTEMPT                  (5)
#define NUM_OF_UNSYNCHED_TRIP_ATTEMPTS                      (3)
#define NUM_OF_PULSES_PER_UNSYNCHED_TRIP_ATTEMP             (10)
#define TRIP_PULSE_WIDTH_SYNCHED                            (862)
#define DELAY_BETWEEN_SYNCHED_TRIP_PULSE                    (2586)
#define DELAY_BETWEEN_SYNCHED_TRIP_ATTEMPTS                 (376660UL)
#define TRIP_PULSE_WIDTH_UNSYNCHED                          (431)
#define DELAY_BETWEEN_UNSYNCHED_TRIP_PULSE                  (862)
#define DELAY_BETWEEN_UNSYNCHED_TRIP_ATTEMPTS               (376660UL)
#define UNSYNC_TRIP_DELAY_BEFORE_WATCHDOG_RESET             (274044000UL)
#define TRIP_PULSE_WIDTH_INSTANT                            (6146)     // 6 x SYNCHED PULSE WIDTH, APPROX 13.5 ms

#define WAIT_FOR_STABLE_LOW_ZERO_CROSSING_DETECTOR_TIMEOUT     (7830)	// no ZCD timeout about 31.22 ms
#define ZERO_CROSSING_DETECTOR_STABLE_LOW_ACCEPTABLE_COUNT     (690)	// about 2.3 ms after ZCD low

/* keeping some of the tested values for future reference.
 *
 * WAIT_FOR_STABLE_LOW_ZERO_CROSSING_DETECTOR_TIMEOUT     (10000)   // no ZCD timeout about 16.18 ms
 * ZERO_CROSSING_DETECTOR_STABLE_LOW_ACCEPTABLE_COUNT     (900)     // about 1.3 ms after ZCD low
 * ZERO_CROSSING_DETECTOR_STABLE_LOW_ACCEPTABLE_COUNT     (1300)    // about 2.5 ms after ZCD low
 * ZERO_CROSSING_DETECTOR_STABLE_LOW_ACCEPTABLE_COUNT     (1800)    // about 3.4 ms after ZCD low
 *
 */

#define TRIP_PULSE_WIDTH                                    (4552)    // Reflect the power supply change. 10ms
#define DELAY_BETWEEN_TRIP_ATTEMPTS                         (375730UL)// around 50 full cycles
#define NUM_OF_TRIP_ATTEMPTS                                (2)

//#define DELAY_TO_ENSURE_MESSAGE_RECEIPT						(40000UL) //TODO We need to find a way to decrease this number! As it stands we may have a significant problem and no guarantee of messages going through.
#define DELAY_TO_ENSURE_MESSAGE_RECEIPT						(3800UL) //This number is set to allow a close match to a half cycle. The minimum number for this seems to be 1200 however.

#define IN_POSITIVE_HALF_CYCLE
#define IN_NEGATIVE_HALF_CYCLE (gpio_read_input_pin(zcd_port, zcd_pin) == GPIO_PIN_SET)

typedef enum
{
    ZERO_CROSSING_DETECTOR_HIGH_NOT_ENCOUNTERED = 0,
    ZERO_CROSSING_DETECTOR_HIGH_ENCOUNTERED = 1,

}zero_crossing_detector_t;

/*************************************************************************************************/
/**
 * @brief Unsynchronized trip routine.
 *
 * This function trips the breaker by sending trip pulses via the trip signals. The pulses are
 * NOT synchronized with zero crossing.
 *
 *
 * @return none
 *
 * @exception none
 *
 *
 *************************************************************************************************/
void unsynced_trip(void);

/*************************************************************************************************/
/**
 * @brief Used to delay the micro from resetting while it hs messages queued.
 *
 * This function will attempt to send any queued messages prior to resetting. during this time
 * if the breaker is closed or it fails to detect the HAL position it will trip the breaker.
 * additionally if the messages cannot all be dequeued after a certain amound of time the breaker
 * will give up and reset anyway.
 *
 *
 * @return none
 *
 * @exception none
 *
 *
 *************************************************************************************************/
void delayed_reset_for_m2m_queue(void);
void delayed_failed_to_trip_reset_for_m2m_queue(void);

#endif /*_TRIP__H*/


