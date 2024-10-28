#ifndef _BTN__H
#define _BTN__H
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
 * @brief Internal (to the component) interface for the button component.
 *
 * @file btn_.h
 * @ingroup btn
 *
 *//*
 *
 **************************************************************************************************/
#include "btn_internal.h"

/* for diagnostics */
#define BUTTON_COMPONENT

/* Switch for shared pin, this is used for CH/BR 1 pole breakers. */
#define SHARED_PIN

typedef struct
{
    u16 id;
    btn_status_t status;
    btn_status_t prev_status;
    btn_state_t state;
    btn_state_t prev_state;
    GPIO_TypeDef *GPIOx;
    u32 PinMask;
    u32 timer_single;
    u32 timer_double;
    btn_logic_t logic;

} btn_t;

/*NUM_BUTTONS is not defined in this component. User should define this in their project */
typedef struct
{
    u16 button_cnt;
    btn_t button_array[NUM_BUTTONS];

} button_list_t;


/**************************************************************************************************/
/**
 * @brief Button task.
 *
 * Updates each button in the button array. Should be called periodically.
 *
 * @return none.
 *
 * @exception none
 *
 **************************************************************************************************/
void btn_task(u32 it);

/**************************************************************************************************/
/**
 * @brief Initialize a new button.
 *
 * Initialize a new button. This function is called by btn_add_btn
 *
 *
 * @param[in]  new_button - The button that is being initialized.
 *
 * @return STATUS_FAIL If new_button doesn't exist.
 *         STATUS_OK If button is successfully initialized.
 *
 * @exception none
 *
 **************************************************************************************************/
status_t init_new_btn(btn_t* new_button);

/**************************************************************************************************/
/**
 * @brief Update button.
 *
 * Update button state base on the button status return.
 *
 * @param[in]  button - button pointer that has all the characteristics of the button.
 *
 * @return BTN_STATE_UNKNOWN When button status return BTN_FAIL.
 *         BTN_NOT_PRESSED When the button is not pressed.
 *         BTN_PRESSED When button is pressed and not released.
 *         BTN_WAIT_FOR_DOUBLE_PRESSED Wait period for double press.
 *         BTN_PRESSED_SINCE_STARTUP If button is pressed since startup
 *         BTN_DOUBLE_PRESSED If button is pressed for the second time within double press timeout
 *         BTN_SINGLE_TRIGGERED If button single pressed and released.
 *         BTN_DOUBLE_TRIGGERED If button double pressed and released.
 *
 * @exception none
 *
 **************************************************************************************************/
btn_state_t btn_update_state(btn_t* btn);

/**************************************************************************************************/
/**
 * @brief Read button status.
 *
 * Reads the button.
 *
 * NOTE: In shared pin situations, make sure this function is being called at the right time.
 *
 * @param[in]  btn_id - The button ID for the button to get status from.
 *
 * @return BTN_UP if button input and button logic are not the same.
 *         BTN_DOWN if button input and button logic are the same.
 *         BTN_FAIL if button is not found in the array.
 *
 * @exception none
 *
 **************************************************************************************************/
btn_status_t btn_read(u16 btn_id);

#endif /*_BTN__H */
