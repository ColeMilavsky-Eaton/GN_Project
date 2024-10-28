#ifndef _BTN_API_H_
#define _BTN_API_H_
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
 * @brief External interface for the button component.
 *
 * @file btn_api.h
 * @ingroup btn
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "gpio_api.h"

/* define the number of buttons. User will need to modify this */
//#if BOARD_CONFIGURATION == BOARD_SB2
//	#define NUM_BUTTONS (2)
//#else
	#define NUM_BUTTONS (1)
//#endif

#define BTN_DEBOUNCE_TIMER_COUNT            (12)
#define BTN_TIMER_SINGLE_LIMIT              (0xFFFFFF00)
#define BTN_TIMER_DOUBLE_PRESSED_TIMEOUT    (50) // is this too long??
#define BTN_UP_TIMEOUT                      (60)
//#define BTN_FACTORY_RESET_TIME 				(3600) //30s
#define BTN_HELD1_TIMEOUT 					(360) //3s
#define BTN_HELD2_TIMEOUT 					(600) //5s
#define BTN_HELD3_TIMEOUT 					(600) //5s
//held time will be based off when we want it to execute.
#define BTN_HELD1_TIME						(240) //2s
#define BTN_HELD2_TIME						(600) //5s
#define BTN_HELD3_TIME						(3600)//30s

#define BTN_SINGLE_TRIGGERED_TIMEOUT        (2)
#define BTN_DOUBLE_TRIGGERED_TIMEOUT        (2)
#define BTN_HELD_TRIGGERED_TIMEOUT 			(2)

typedef enum
{
    BTN_UP = 0,
    BTN_DOWN = 1,
    BTN_FAIL = 2,

} btn_status_t;

typedef enum
{
    BTN_STATE_UNKNOWN = 0,
    BTN_NOT_PRESSED = 0x10,
    BTN_PRESSED = 0x20,
    BTN_WAIT_FOR_DOUBLE_PRESSED = 0x21,
    BTN_PRESSED_SINCE_STARTUP = 0x30,
    BTN_DOUBLE_PRESSED = 0x40,
    BTN_SINGLE_TRIGGERED = 0x50,
    BTN_DOUBLE_TRIGGERED = 0x60,
#if BOARD_CONFIGURATION == BOARD_SB2
	//order and distance matters for these as the function shifts states by +1,+2,or+3
	BTN_HELD1_PRESSED = 0x70,
	BTN_HELD1_TRIGGERED,// = 0x71,
	BTN_HELD1_CANCELED,// = 0x72,
	BTN_HELD2_PRESSED,// = 0x73,
	BTN_HELD2_TRIGGERED,// = 0x74,
	BTN_HELD2_CANCELED,// = 0x75,
	BTN_HELD3_PRESSED,// = 0x76,
	BTN_HELD3_TRIGGERED,// = 0x77,
	//BTN_HELD3_CANCELED,// = 0x78,
	BTN_OVER_HELD,// = 0x79,
#endif
    NUM_BTN_STATES

} btn_state_t;

typedef enum
{
    ACTIVE_LOW = 0,
    ACTIVE_HIGH = 1,

} btn_logic_t;


/**************************************************************************************************/
/**
 * @brief Initialize button component.
 *
 * This function initializes the button component and add task to task manager.
 *
 * @return none.
 *
 * @exception none
 *
 **************************************************************************************************/
void btn_init(void);

/**************************************************************************************************/
/**
 * @brief Add button to the button component array.
 *
 * Adds a button with all the characteristics of the button to the button array.
 *
 * NOTE: btn_id CANNOT be 0.
 *
 * @param[in]  btn_id - The button ID the the user assigns. This value can't be 0.
 * @param[in]  GPIOx - GPIO port for the button
 * @param[in]  PinMask - Pin for the button
 * @param[in]  logic - This parameter can be either ACTIVE_LOW or ACTIVE_HIGH, depending on the
 *                     configuration of the button.
 *
 * @return STATUS_FAIL If btn_id is 0 or
 *                     If the same btn_id is already added/being used or
 *                     If number of buttons exceed the number of buttons defined.
 *         STATUS_OK If button is successfully added.
 *
 * @exception none
 *
 **************************************************************************************************/
status_t btn_add_btn(u16 btn_id, GPIO_TypeDef* GPIOx, u32 PinMask, btn_logic_t logic);

/**************************************************************************************************/
/**
 * @brief Gets the state of the button.
 *
 * @param[in]  btn_id - button id of the button
 *
 * @return BTN_STATE_UNKNOWN If the button id is not correct.
 *         BTN_NOT_PRESSED When the button is not pressed.
 *         BTN_PRESSED When button is pressed and not released.
 *         BTN_PRESSED_SINCE_STARTUP If button is pressed since startup
 *         BTN_DOUBLE_PRESSED If button is pressed for the second time within double press timeout
 *         BTN_SINGLE_TRIGGERED If button single pressed and released.
 *         BTN_DOUBLE_TRIGGERED If button double pressed and released.
 *
 * @exception none
 *
 **************************************************************************************************/
btn_state_t btn_get_button_state(u16 btn_id);

/**************************************************************************************************/
/**
 * @brief Gets the status of the button.
 *
 * @param[in]  btn_id - button id of the button
 *
 * @return BTN_UP
 *         BTN_DOWN
 *         BTN_FAIL
 *
 * @exception none
 *
 **************************************************************************************************/
btn_status_t btn_get_status(u16 btn_id);

#endif /* _BTN_API_H_ */

