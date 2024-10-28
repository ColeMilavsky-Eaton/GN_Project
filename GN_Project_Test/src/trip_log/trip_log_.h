#ifndef _TRIP_LOG_LED_XFER__H
#define _TRIP_LOG_LED_XFER__H
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
 * @brief Internal (to the component) interface for the trip_log component.
 *
 * @file trip_log_.h
 * @ingroup trip_log
 *
 *//*
 *
 **************************************************************************************************/

#include "trip_log_internal.h"
#include "led_api.h"

/* for diagnostics */
#define TRIP_LOG_LED_XFER_COMPONENT

/* TRIP_LOG_OUTPUT_STATUS_LOG_COUNTER_THRESHOLD should be larger than the number of interrupts per
 * half cycle. Smaller than number of interrupts per half cycle may cause breaker to enter pentool
 * mode and start sending data when powered by AC power.
 */
#define TRIP_LOG_OUTPUT_STATUS_LOG_COUNTER_THRESHOLD        (20)

#define TRIP_LOG_LED_XFER_START_DELAY                       (9) //cycles

#define UID_BEGIN                                              (0x1FFF7590UL)
#define UID_LENGTH                                             (12)

#define TRIP_LOG_TRIP_CODE_DATA_SIZE            (64)
#define TRIP_LOG_HEADER_SIZE                    10
#define TOKEN_SIZE                               1
#define RESERVED_BYTES_SIZE                      3

#define NO_TRIP_CODE 0x00

#define MSG_TOKEN   (u8)0xAA

#define ADDITIONAL_DATA_BYTES_SIZE              (TOKEN_SIZE + RESERVED_BYTES_SIZE + sizeof(__build_checksum) + sizeof(__version))

#define TRIP_LOG_MAX_TRIPCODES_FOR_PEN_TOOL    (640 - (TRIP_LOG_HEADER_SIZE + ADDITIONAL_DATA_BYTES_SIZE))

#define NUM_TIMES_TO_OUTPUT_TRIP_LOG         (1000)
#define DELAY_BETWEEN_TRIP_LOG_OUTPUTS      (13000)

#define TRIP_LOG_LED_DISPLAY_FIRST_NONZERO_BIT_HAS_BEEN_OUTPUT          (1)
#define TRIP_LOG_LED_DISPLAY_FIRST_NONZERO_BIT_HAS_NOT_BEEN_OUTPUT      (0)
#if BOARD_CONFIGURATION == BOARD_SB2
#define TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT                  (1)
#else
#define TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT                  (30)
#endif
#define TRIP_LOG_LED_DISPLAY_NUMBER_OF_BLINKS_PER_CODE                  (8) // maximum is 16
#define TRIP_LOG_LED_DISPLAY_NUMBER_OF_NONBLINKS_BETWEEN_CODES          (3)
#define TRIP_LOG_LED_DISPLAY_HALF_CYCLES_PER_LED_BLINK_ON               (48)  //24->48
#define TRIP_LOG_LED_DISPLAY_HALF_CYCLES_PER_LED_BLINK_OFF              (48)

#define TRIP_LOG_LED_DISPLAY_BITMASK                                    (((unsigned int)0x0001)<<(LED_DISPLAY_NUMBER_OF_BLINKS_PER_CODE-1))
#define TRIP_LOG_LED_DISPLAY_TOTAL_NUMBER_OF_BLINKS_PER_CODE            (TRIP_LOG_LED_DISPLAY_NUMBER_OF_BLINKS_PER_CODE + TRIP_LOG_LED_DISPLAY_NUMBER_OF_NONBLINKS_BETWEEN_CODES)
#define TRIP_LOG_LED_DISPLAY_TOTAL_HALF_CYCLES_PER_BLINK                (TRIP_LOG_LED_DISPLAY_HALF_CYCLES_PER_LED_BLINK_ON + TRIP_LOG_LED_DISPLAY_HALF_CYCLES_PER_LED_BLINK_OFF)


//for Overload quick blinks
#define QUICK_BLINK_PERIOD                                     (6)
#define QUICK_BLINK_INTERVAL                                   (35)
#define QUICK_BLINK_01                                         (6)
#define QUICK_BLINK_02                                         (QUICK_BLINK_01 + QUICK_BLINK_PERIOD)
#define QUICK_BLINK_03                                         (QUICK_BLINK_02 + QUICK_BLINK_INTERVAL)
#define QUICK_BLINK_04                                         (QUICK_BLINK_03 + QUICK_BLINK_PERIOD)
#define QUICK_BLINK_05                                         (QUICK_BLINK_04 + QUICK_BLINK_INTERVAL)
#define QUICK_BLINK_06                                         (QUICK_BLINK_05 + QUICK_BLINK_PERIOD)
#define QUICK_BLINK_07                                         (QUICK_BLINK_06 + QUICK_BLINK_INTERVAL)
#define QUICK_BLINK_08                                         (QUICK_BLINK_07 + QUICK_BLINK_PERIOD)
#define QUICK_BLINK_09                                         (QUICK_BLINK_08 + QUICK_BLINK_INTERVAL)
#define QUICK_BLINK_10                                         (QUICK_BLINK_09 + QUICK_BLINK_PERIOD)

#define PRG_BEGIN_COUNTER_MAX                                  (QUICK_BLINK_10 + 1)


typedef struct led_display_t
{
    struct
    {
        u8 total_half_cycle_per_blink;
        u8 num_of_nonblink_between_code;
        u8 times_to_blink;
    } count;
    u16 output_code;
    led_color_t led_display_color;

} latest_logcode_led_display_t;

typedef struct trip_log_led_xfer_t
{
    u16 counter;
    u8  sub_token_counter;
    trip_log_led_xfer_progress_flag_t  progress_flag;
    u8  next_status_log_value;
    u8  bit_counter;
    u8  start_counter;
    trip_log_led_xfer_message_send_state_t  message_send_state;
    u8  LRC;

} trip_log_led_xfer_t;

/*************************************************************************************************/
/**
 * @brief Updates led ON/OFF when trip log led transfer is in progress.
 *
 * Each bit uses three tokens when transferring. The first token led is always on. The second
 * token depends on the current bit that is being transferred, led ON if sending bit 1 and led off
 * if sending bit 0. Third token is always not lit.
 *
 * After all 8 bits have been transferred, led will stay off.
 *
 * @param[in] it - interrupt number
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void trip_log_update_trip_log_led_xfer_led(u32 it);

/*************************************************************************************************/
/**
 * @brief Prepare and look for the next data for trip log led transfer to send out.
 *
 * This function prepares and looks for the next data for trip log led transfer to send out. There
 * are 4 parts in this process. The first part that this function looks for is the UID. After all
 * the UID bytes are sent out, the second part is the breaker info which includes the firmware
 * checksum and calibration values. The third part is the trip logs and the fourth part is the
 * LRC(checksum) for the data that is being sent out.
 *
 * The LRC is being calculated and updated each time a new data is being prepared.
 *
 * @param[in] it - interrupt number
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void trip_log_prepare_trip_log_led_xfer_data(u32 it);

/*************************************************************************************************/
/**
 * @brief Called to map an event log value (trip code) to a corresponding LED blink pattern
 *
 * At power-up, if apropriate, the 'log_normal_startup_at_startup()'
 * routine determines the most recent cause of trip.  This function maps that trip
 * code into a pattern suitable for blinking via a status indication LED.
 *
 *@param[in]  status_log_entry - the trip code to blink.
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void trip_log_map_logcode_value_to_logcode_category( u8 status_log_entry );

/*************************************************************************************************/
/**
 * @brief For single LED muxed with test button; Sets state of LEDs, depending on program context
 *
 * This function is a task that should be assigned to task manager and called once every half cycle
 *
 * @param[in] it - interrupt number
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void trip_log_update_blink_latest_logcode_led(u32 it);

#if BOARD_CONFIGURATION != BOARD_SB2 //Smart Breaker boards do not contain spi interface
/*************************************************************************************************/
/**
 * @brief Outputs status log via spi.
 *
 * Output the last two bytes of the checksum since the pen tool is only able to display two bytes.
 * Then output all tripcodes.
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void trip_log_output_trip_log_via_spi(void);

/*************************************************************************************************/
/**
 * @brief Check whether or not to run output_status_log_via_spi
 *
 * This function reads the input pin. If it is being triggered enough times by the pen tool.
 * System will jump to output_status_log_via_spi and output data to pen tool.
 *
 * This function should be registered to task manager and called periodically.
 *
 * @param[in] it - interrupt number
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void trip_log_check_for_output_trip_log_via_spi(u32 it);
#endif

#endif /*_TRIP_LOG_LED_XFER__H*/
