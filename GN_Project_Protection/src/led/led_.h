#ifndef _LED__H
#define _LED__H
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
 * @brief Internal (to the component) interface for the led component.
 *
 * @file led_.h
 * @ingroup led
 *
 *//*
 *
 **************************************************************************************************/

#include "led_internal.h"
#include "gpio_api.h"
#include "types.h"
#include "main_internal.h"
#include "task_manager_api.h"
#include "usart.h"

typedef struct
{
    led_color_t id;
    led_request_states_t request_state;

}led_t;

typedef struct
{
    u16 led_request_cnt;
    led_t led_request_array[REQUEST_ARRAY_SIZE];

} led_request_list_t;

/**************************************************************************************************/
/**
 * @brief Update led state machine
 * called at every interrupt
 *
 * @return None
 *
 * @exception none
 *
 **************************************************************************************************/
void led_update_state(led_t* led);

/**************************************************************************************************/
/**
 * @brief Controls blinking of LED
 * Also handles period and rate of LED blinking
 *
 * @return None
 *
 * @exception none
 *
 **************************************************************************************************/
void led_rainbow(led_rainbow_param_t* rainbow);

/**************************************************************************************************/
/**
 * @brief Blink led per the identify me requirement
 *
 * @return None
 * @exception none
 *
 **************************************************************************************************/
void led_identify_me(led_rainbow_param_t*);
/**************************************************************************************************/
/**
 * @brief Monitors the request buffer at every interrupt
 * CHecks high priority request to execute
 *
 * @return None
 *
 * @exception none
 *
 **************************************************************************************************/
void led_task(u32 it);

/**************************************************************************************************/
/**
 * @brief Resets the request array
 *
 * @return None
 *
 * @exception none
 *
 **************************************************************************************************/
void led_reset_request_array(void);

#endif /*_LED__H*/

