#ifndef _LED_API_H
#define _LED_API_H
/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 **************************************************************************************************
 *  Written by:         Sonal Barge
 *                      EIIC Eaton Electrical
 *                      Magarpatta City, Hadapsar
 *                      Pune MH-411 013
 *//**
 * @brief External interface for the led component.
 *
 * @file led_api.h
 * @ingroup led
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "gpio_api.h"
#include "firmware.config"

/* define the number of buttons. User will need to modify this */
#define REQUEST_ARRAY_SIZE (10)

#if defined LED_ACTIVE_LOW
#define LED_RED_ON  LL_GPIO_ResetOutputPin(LED_Red_GPIO_Port, LED_Red_Pin);
#define LED_RED_OFF  LL_GPIO_SetOutputPin(LED_Red_GPIO_Port, LED_Red_Pin);

#define LED_GREEN_ON  LL_GPIO_ResetOutputPin(LED_Green_GPIO_Port, LED_Green_Pin);
#define LED_GREEN_OFF LL_GPIO_SetOutputPin(LED_Green_GPIO_Port, LED_Green_Pin);

#define LED_BLUE_ON  LL_GPIO_ResetOutputPin(LED_Blue_GPIO_Port, LED_Blue_Pin);
#define LED_BLUE_OFF LL_GPIO_SetOutputPin(LED_Blue_GPIO_Port, LED_Blue_Pin);

#define LOAD_PWR_LED_ON  LL_GPIO_ResetOutputPin(LED_LOAD_GPIO_Port, LED_LOAD_Pin);
#define LOAD_PWR_LED_OFF LL_GPIO_SetOutputPin(LED_LOAD_GPIO_Port, LED_LOAD_Pin);

#else
#define LED_RED_ON  LL_GPIO_SetOutputPin(LED_Red_GPIO_Port, LED_Red_Pin);
#define LED_RED_OFF  LL_GPIO_ResetOutputPin(LED_Red_GPIO_Port, LED_Red_Pin);

#define LED_GREEN_ON  LL_GPIO_SetOutputPin(LED_Green_GPIO_Port, LED_Green_Pin);
#define LED_GREEN_OFF LL_GPIO_ResetOutputPin(LED_Green_GPIO_Port, LED_Green_Pin);

#define LED_BLUE_ON  LL_GPIO_SetOutputPin(LED_Blue_GPIO_Port, LED_Blue_Pin);
#define LED_BLUE_OFF LL_GPIO_ResetOutputPin(LED_Blue_GPIO_Port, LED_Blue_Pin);

#ifdef SB20_PUL
    #define LOAD_PWR_LED_Green_ON  LL_GPIO_SetOutputPin(LED_Green_LOAD_GPIO_Port, LED_Green_LOAD_Pin)
    #define LOAD_PWR_LED_Green_OFF LL_GPIO_ResetOutputPin(LED_Green_LOAD_GPIO_Port, LED_Green_LOAD_Pin)
    #define LOAD_PWR_LED_Red_ON  LL_GPIO_SetOutputPin(LED_Red_LOAD_GPIO_Port, LED_Red_LOAD_Pin)
    #define LOAD_PWR_LED_Red_OFF LL_GPIO_ResetOutputPin(LED_Red_LOAD_GPIO_Port, LED_Red_LOAD_Pin)
    #define LOAD_PWR_LED_Blue_ON  LL_GPIO_SetOutputPin(LED_Blue_LOAD_GPIO_Port, LED_Blue_LOAD_Pin)
    #define LOAD_PWR_LED_Blue_OFF LL_GPIO_ResetOutputPin(LED_Blue_LOAD_GPIO_Port, LED_Blue_LOAD_Pin)
#else
    #define LOAD_PWR_LED_ON  LL_GPIO_SetOutputPin(LED_LOAD_GPIO_Port, LED_LOAD_Pin);
    #define LOAD_PWR_LED_OFF LL_GPIO_ResetOutputPin(LED_LOAD_GPIO_Port, LED_LOAD_Pin);
#endif
#endif

#define CYCLE_LED_RATE                  290 //considering 16 it in 8.3ms//35  // 290ms based on 8.3ms ZCD
#define IDENTIFY_ME_LED_RATE            1000 // 1s
#define FLASH_LED_1_SECOND_PERIOD       1 //  1 second based on 8.3ms ZCD
#define FLASH_LED_2_SECOND_PERIOD       2 //considering 16 it in 8.3ms// 2 seconds based on 8.3ms ZCD
#define FLASH_LED_3_SECOND_PERIOD       3 // 3 seconds based on 8.3ms ZCD
#define FLASH_LED_5_SECOND_PERIOD       5 // 5 seconds based on 8.3ms ZCD
#define FLASH_LED_10_SECOND_PERIOD      10 // 10 seconds based on 8.3ms ZCD
#define FLASH_LED_30_SECOND_PERIOD      30 // 10 seconds based on 8.3ms ZCD

#define LED_FLASH_1_SECOND_PERIOD_CNT   120 //  1 second based on (8.3ms/16 = 0.518ms for each interrupt) (0.518*1000ms)
#define LED_FLASH_2_SECOND_PERIOD_CNT   240 //  2 second in 60Hz system
#define LED_FLASH_RATE_CNT             816 //  100 millisecond based on (8.3ms/16 = 0.518ms for each interrupt) (0.518*100ms)
#define LED_FLASH_RATE_MINIMUM_TIME      100   //100ms
#define CONVERT_TO_ms               1000

#define LED_CLEAR    0xFF

#define BLINK_FOREVER               (0xFF * 6) //converted to 50ms interval based on the current implementation.
//#define BLINK_FOREVER               (0xFF * 120) //converted to seconds based on the current implementation.
//#define BLINK_FOREVER               0xFF //converted to seconds based on the current implementation.

// Led enum definition
typedef enum
{
    LED_RED = 0,    //0
    LED_YELLOW,     //1
    LED_GREEN,      //2
    LED_BLUE,       //3
    LED_CYAN,       //4
    LED_MAGENTA,    //5
    LED_WHITE,     //6
    LED_UNASSIGNED_COLOR = LED_CLEAR,

}led_color_t;

typedef enum
{
   LED_TURN_OFF = 0,
   LED_TURN_ON,
   LED_NO_REQUEST = LED_CLEAR,

}led_request_states_t;

typedef enum
{
   LED_IDLE = 0,
   LED_STATE_TRANSITION,

}led_states_t;

typedef enum
{
    KEY_FREE = 0,
    KEY_TAKEN_BY_TLLT,
    KEY_TAKEN_BY_RAINBOW,
    KEY_TAKEN_BY_BLINK_LATEST_LOG_CODE,
    KEY_TAKEN_BY_QUICk_BLINK,
    KEY_TAKEN_BY_AUTO_MONITOR_FAILURE,
	KEY_TAKEN_BY_PROVISIONING,
	KEY_TAKEN_BY_SBLCP,
	KEY_TAKEN_BY_BUTTON_HOLD,
	KEY_TAKEN_BY_IDENTIFY_ME,
	KEY_TAKEN_BY_FAILED_TO_TRIP

}led_key_t;

typedef struct
{
    u32 led_rainbow_rate;
    u32 led_rainbow_period;
    u32 led_rainbow_period_counter;
    u32 led_rainbow_rate_counter;

}led_rainbow_param_t;

// In order to reduce the production time testing, this enum defines default and
// factory LED blinking mode for the IDENTIFY_ME command that gets
// configured accordingly.

#define LED_BLINKING_DEFAULT_MODE 0

#define LED_BLINKING_FACTORY_MODE 1

void led_init(void);

/**************************************************************************************************/
/**
 * @brief Turn on the current LED and turn off the other LEDs
 *
 * @param[in] led_color_t led - Led to be turned on
 *
 * @return None
 *
 * @exception none
 *
 **************************************************************************************************/
void led_turn_on (led_color_t led);

/**************************************************************************************************/
/**
 * @brief Turn off the all LED
 *
 * @return None
 *
 * @exception none
 *
 **************************************************************************************************/
void led_turn_off (void);


/**************************************************************************************************/
/**
 * @brief request to Turn on/off the current LED. If already turned on other fucntion then will not over write.
 *
 * @param[in] led_color_t led - Led to be flashed
 * @param[in]  state - request for led ON state or OFF state
 *
 * @return None
 *
 * @exception none
 *
 **************************************************************************************************/
void led_request(led_color_t led_id, led_request_states_t state);

/**************************************************************************************************/
/**
 * @brief get key status
 *
 * @return key status
 *
 * @exception none
 *
 **************************************************************************************************/
led_key_t led_get_key_status(void);

/**************************************************************************************************/
/**
 * @brief set key status
 *
 * @param[in] led_key_t status - status to set
 *
 * @return None
 *
 * @exception none
 *
 **************************************************************************************************/
void led_set_key_status(led_key_t status);

/**************************************************************************************************/
/**
 * @brief reset key status
 *
 * @return reset key status to KEY_FREE
 *
 * @exception none
 *
 **************************************************************************************************/
void led_reset_key_status(void);

/**************************************************************************************************/
/**
 * @brief Executes the power on rainbow flashing sequence
 *
 * @return None
 *
 * @exception none
 *
 **************************************************************************************************/
void led_rainbow_task(u32 it);

/**************************************************************************************************/
/**
 * @brief Executes the identify me flashing sequence
 *
 * @return None
 *
 * @exception none
 *
 **************************************************************************************************/
void led_identify_me_task(u32 it);
/**************************************************************************************************/
/**
 * @brief Request for rgb led to flash rainbow.
 *
 * @return None
 *
 * @exception none
 *
 **************************************************************************************************/
void led_request_rainbow(void);

/**************************************************************************************************/
/**
 * @brief  Blinking the LED(s) for provisioning states and SBLCP command
 *
 * @return None
 *
 * @exception none
 *
 **************************************************************************************************/
void BlinkingLEDStateMachine();

/**************************************************************************************************/
/**
 * @brief  To check if RGB LED is blinking or not
 *
 * @return None
 *
 * @exception none
 *
 **************************************************************************************************/
bool IsLED_Blinking(void);

#endif /*_LED_API_H*/

