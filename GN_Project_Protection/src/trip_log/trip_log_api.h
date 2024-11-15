#ifndef _TRIP_LOG_LED_XFER_API_H
#define _TRIP_LOG_LED_XFER_API_H
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
 * @brief External interface for the trip_log component.
 *
 * @file trip_log_api.h
 * @ingroup trip_log
 *
 *//*
 *
 **************************************************************************************************/
#include"types.h"
#include "usart.h"

#define TRIP_LOG_FIRST_TRIPCODE_ADDRESS     (0x0801A000)
#define TRIP_LOG_LAST_TRIPCODE_ADDRESS      (0x0801BFF8)
#define TRIP_LOG_MAX_NUM_TRIPCODE           (((TRIP_LOG_LAST_TRIPCODE_ADDRESS - TRIP_LOG_FIRST_TRIPCODE_ADDRESS) >> 3) + 1)
#define TRIP_LOG_TRIPCODE_FACTORY_VALUE     (0xFFFFFFFFFFFFFFFF)
#define TRIP_LOG_TRIP_CODE_DATA_SIZE        (64)

typedef enum trip_log_led_xfer_message_send_state_t
{
    TRIP_LOG_LED_XFER_UID_SEND_STATE,
    TRIP_LOG_LED_XFER_BREAKER_INFO_SEND_STATE,
    TRIP_LOG_LED_XFER_TRIPCODE_SEND_STATE,
    TRIP_LOG_LED_XFER_LRC_SEND_STATE

} trip_log_led_xfer_message_send_state_t;

typedef enum trip_log_led_xfer_progress_flag_t
{
    TRIP_LOG_LED_XFER_NOT_IN_PROGRESS,
    TRIP_LOG_LED_XFER_IN_PRE_PROGRESS,
    TRIP_LOG_LED_XFER_IN_PROGRESS,
} trip_log_led_xfer_progress_flag_t;

/**************************************************************************************************/
/**
 * @brief Initialize trip log led transfer feature.
 *
 * Initialize variables and assigns task to task manager. trip_log_prepare_led_xfer_data
 * should run on interrupt 0. And trip_log_update_led_xfer_led should run on every
 * interrupt. Also note that the order matters, on interrupt 0, prepare data should run before
 * update.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void trip_log_led_xfer_init(void);


/*************************************************************************************************/
/**
 * @brief function that adds trip_log_blink_latest_logcode_init to task manager.
 *
 * function that adds trip_log_blink_latest_logcode_init to task manager.
 * trip_log_blink_latest_logcode_init should be called once per half cycle.
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void trip_log_blink_latest_logcode_init(void);

#if BOARD_CONFIGURATION != BOARD_SB2 //Smart Breaker boards do not contain spi interface
/*************************************************************************************************/
/**
 * @brief Initialize the task to check whether or not to run output trip log via spi
 *
 * Initialize the task to check whether or not to run output status log via spi
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void trip_log_output_via_spi_init(void);
#endif

/*************************************************************************************************/
/**
 * @brief Get the current progress of the trip log led transfer.
 *
 * This function returns the progress of the trip log led transfer.
 *
 *
 * @return TRIP_LOG_LED_XFER_NOT_IN_PROGRESS When not in progress
 *         TRIP_LOG_LED_XFER_IN_PRE_PROGRESS When in pre-progress
 *         TRIP_LOG_LED_XFER_IN_PROGRESS When in progress
 *
 * @exception none
 *
 *************************************************************************************************/
trip_log_led_xfer_progress_flag_t trip_log_get_led_xfer_progress(void);


/*************************************************************************************************/
/**
 * @brief This function requests the trip log led xfer feature
 *
 * This function requests the trip log led xfer feature and sets the progress to pre-progress.
 *
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void trip_log_request_led_xfer_initiate(void);


/**************************************************************************************************/
/**
 * @brief write trip codes
 *
 * The function will first look for the next empty space. Then write the trip code.
 *
 *
 * @param[in] trip_code_data* - The trip code data to write.
 * @param[in] num_bytes - Number of bytes to write.
 *
 * @return STATUS_OK If write successful.
 *         STATUS_FAIL If write unsuccessful
 *                     If null trip_code_data
 *                     If num_bytes out of range
 *                     If location being written to is out of trip code area.
 *                     If num_bytes exceed available bytes in trip code area.
 *
 * @exception none
 *
 **************************************************************************************************/
status_t trip_log_write_tripcode(u8* trip_code_data, u8 num_bytes);


/*************************************************************************************************/
/**
 * @brief Log normal startup
 *
 * This function checks the latest trip code in the trip code memory and writes normal startup. If
 * the latest trip code is already normal startup, then this function will not write again.
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void trip_log_write_normal_startup(void);


/*************************************************************************************************/
/**
 * @brief blinks the latest trip code.
 *
 * This function looks for the latest trip code in the trip code area. And calls
 * map_logcode_value_to_logcode_category() to setup the led and blink.
 *
 * This function is called when requested by PTT.
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void trip_log_blink_latest_logcode_category();


/*************************************************************************************************/
/**
 * @brief This function looks for the next available trip log location
 *
 * This function looks for the next available trip log location in the trip log area of flash.
 *
 * @return a void pointer to the location of the free space.
 *
 * @exception none
 *
 *************************************************************************************************/
void* trip_log_get_next_available_location(void);


/*************************************************************************************************/
/**
 * @brief This function finds the latest fault code.
 *
 * This function finds the latest fault code. Note that only protection fault codes, continuous
 * hardware fault codes, and user initiated self tests fault codes will return from this function.
 *
 * If there are no fault codes in the history, then the first stored log code will be returned.
 *
 * @return the latest fault code, or the first log code if no fault codes are present
 *
 * @exception none
 *
 *************************************************************************************************/
u8 trip_log_find_latest_fault_code(void);


/*************************************************************************************************/
/**
 * @brief This function finds the latest log code.
 *
 * This function finds the latest log code.
 *
 * @return the latest log code.
 *
 * @exception none
 *
 *************************************************************************************************/
u8 trip_log_find_latest_log_code(void);

/*************************************************************************************************/
/**
 * @brief This function finds the latest fault code after anchor
 *
 * This function finds the latest fault code after the anchor. If there are no fault codes after
 * the anchor, then the anchor will be returned.
 *
 * However, if there are no fault codes in the history, then the first log code that is stored in
 * the trip code area of flash will be returned.
 *
 * @return the latest fault code.
 *
 * @exception none
 *
 *************************************************************************************************/
u8 trip_log_find_latest_fault_code_after_anchor(void);

/*************************************************************************************************/
/**
 * @brief function trip_log_quickblink_init enables the flag which allows quick blink for overload settings
 *
 * function trip_log_quickblink_init enables the flag which allows quick blink for overload settings
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void trip_log_quickblink_init( void );


/*************************************************************************************************/
/**
 * @brief This function Quick blinks at start-up show overload setting
 *
 *	indications are 1 quick blink - no overload  ,2 quick blinks - 15A, 3 quick blinks - 20A,
 *	4 quick blinks - 25A,5 quick blinks - 30A
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void trip_log_quickblink_indicate_overload_setting( void );


/*************************************************************************************************/
/**
 * @brief This function to get quickblink enable flag
 *
 * function to get quickblink enable flag
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
u8 trip_log_get_quickblink_flag(void);

/*************************************************************************************************/
/**
 * @brief This function request to initiate trip log led xfer process.
 *
 * set the process flag to TRIP_LOG_LED_XFER_IN_PRE_PROGRESS;
 *
 * @return     TRIP_LOG_LED_XFER_NOT_IN_PROGRESS,
               TRIP_LOG_LED_XFER_IN_PRE_PROGRESS,
               TRIP_LOG_LED_XFER_IN_PROGRESS,
 *
 * @exception none
 *
 *************************************************************************************************/
void trip_log_request_trip_log_led_xfer_initiate(void);

#if BOARD_CONFIGURATION != BOARD_SB2 //Smart Breaker boards do not contain spi interface
/*************************************************************************************************/
/**
 * @brief Initialize output trip log via spi feature (pentool)
 *
 * Initialize feature, assign task and initialize global variables.
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void trip_log_output_trip_log_via_spi_init(void);
#endif

/*************************************************************************************************/
/**
 * @brief Initialize trip log led xfer (red blink)
 *
 * Initialize feature, assign task and initialize global variables.
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void trip_log_trip_log_led_xfer_init(void);

/*************************************************************************************************/
/**
 * @brief Logs normal startup at startup.
 *
 * Checks the trip code area for the latest trip log. If the latest isn't normal startup, then
 * log normal startup.
 *
 * At the same time, if the latest trip log is a protection fault code. Then send the information
 * to blink log code category.
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void trip_log_log_normal_startup_at_startup(void);

/*************************************************************************************************/
/**
 * @brief Logs clearing of a fault condition.
 *
 * Checks the trip code area for the latest trip log. If the latest isn't normal startup,
 * or fault cleared then log fault cleared.
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void trip_log_log_fault_cleared(void);

/*************************************************************************************************/
/**
 * @brief Logs HAL fault.
 *
 * Checks the trip code area for the latest trip log. If the latest isn't already a hal fault,
 * then log fault.
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void trip_log_log_HAL_fault(void);

/*************************************************************************************************/
/**
 * @brief Check if the last three trip codes are the same.
 *
 * Read the last three consecutive trip codes, check they are the same
 *
 * @return TRUE if they are the same, otherwise FALSE
 *
 * @exception none
 *
 *************************************************************************************************/
bool trip_log_check_latest_three_codes(void);

#endif /*_TRIP_LOG_LED_XFER_API_H*/
