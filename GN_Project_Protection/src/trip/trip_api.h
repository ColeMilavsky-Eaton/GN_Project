#ifndef _TRIP_API_H
#define _TRIP_API_H
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
 * @brief External interface for the trip component.
 *
 * @file trip_api.h
 * @ingroup trip
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"

/** @name trip_controls macros for controlling trip signal
 * @{ */
#if defined ENABLE_TRIPPING
#define TRIP_1_ON  gpio_set_output_pin(TRIP_1_GPIO_Port, TRIP_1_Pin)
#define TRIP_1_OFF gpio_reset_output_pin(TRIP_1_GPIO_Port, TRIP_1_Pin)

#define TRIP_2_ON  gpio_set_output_pin(TRIP_2_GPIO_Port, TRIP_2_Pin)
#define TRIP_2_OFF gpio_reset_output_pin(TRIP_2_GPIO_Port, TRIP_2_Pin)
#else
#define TRIP_1_ON
#define TRIP_1_OFF gpio_reset_output_pin(TRIP_1_GPIO_Port, TRIP_1_Pin)

#define TRIP_2_ON
#define TRIP_2_OFF gpio_reset_output_pin(TRIP_2_GPIO_Port, TRIP_2_Pin)
#endif
/** @} */

#if NUM_POLES_CONFIGURATION == SINGLE_POLE
 /*************************************************************************************************/
 /**
  * @brief Trip routine.
  *
  * This function trips the breaker by sending trip pulses via the trip signals. The pulses are
  * also synched with zero crossing detection so that the pulses are sent only during positive half
  * cycle.
  *
  * If this function doesn't trip the breaker, will call unsync trip.
  *
  * @param[in] trip_code_data - Pointer to the trip code data for the trip.
  * @param[in] num_bytes - Number of bytes for writing the trip code.
  *
  * @return none
  *
  * @exception none
  *
  *
  *************************************************************************************************/
void trip_routine(u8* trip_code_data, u8 num_bytes);

/*************************************************************************************************/
/**
 * @brief Instant trip routine.
 *
 * This function trips the breaker by setting the trip signals. This function does not sync zcds
 * nor does it send pulses.
 *
 * If this function doesn't trip the breaker, will call unsync trip.
 *
 *
 * @param[in] trip_code_data - Pointer to the trip code data for the trip.
 * @param[in] num_bytes - Number of bytes for writing the trip code.
 *
 * @return none
 *
 * @exception none
 *
 *
 *************************************************************************************************/
void trip_instant(u8* trip_code_data, u8 num_bytes);
#endif

/*************************************************************************************************/
/**
 * @brief trip function
 *
 * This function sends 2 trip pulses to trip the breaker. The pulses are 3.5 half cycles wide and
 * there is a 50 full cycle delay between the 2 pulses.
 *
 * If the first pulse did not successfully trip the breaker, after the second pulse,
 * FAILED_TRIP_ATTEMPT will be logged to memory and will turn on solid red LED.
 *
 *
 * @param[in] trip_code_data - Pointer to the trip code data for the trip.
 * @param[in] num_bytes - Number of bytes for writing the trip code.
 *
 * @return none
 *
 * @exception none
 *
 *
 *************************************************************************************************/
void trip(u8* trip_code_data, u8 num_bytes);

#endif /*_TRIP_API_H */
