/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 ***************************************************************************************************
 *  Written by:         Aaron Joseph
 *                      Eaton Electrical
 *                      1000 Cherrington Parkway
 *                      Pittsburgh, PA  15108-4312
 *                      (412) 893-3300
 *//**
 * @defgroup btn btn component
 *
 * @brief Button component handles initializing, reading and determining the states of the button.
 *
 *  @image rtf button_state.png "button state diagram"
 *
 *  @image html button_state.png "button state diagram"
 *
 * NOTE: 1. User should define the number of button that will be used in btn_api.h
 *       2. User should define a button ID for each button added.
 *
 * @file btn.c
 * @ingroup btn
 *
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "main_internal.h"
#include "btn_.h"
#include "gpio_api.h"
#include "task_manager_api.h"
#include "stm32g0xx_it_api.h"
#include "trip_log_api.h"
#include "firmware.config"
#include "string.h"
#include "led_api.h"


bool initialized_flag;

button_list_t button_list;

#if BUTTON_LED_CONFIGURATION == BUTTON_LED_SHARED_PIN
bool led_on_flag;
#endif

void btn_init(void)
{
    if(initialized_flag == FALSE)
    {
        /* task type to assign tasks */
        task_t t;

        /* assign task */
        t.task_p = btn_task;
        t.it_bits = TSK_MAN_IT_6 | TSK_MAN_IT_7 | TSK_MAN_IT_22 | TSK_MAN_IT_23;

        task_man_add_task_to_schedule(t);

        /* clear/initialize array */
        memset(button_list.button_array, 0, sizeof(button_list.button_array));

        button_list.button_cnt = 0;

        initialized_flag = TRUE;

#if BUTTON_LED_CONFIGURATION == BUTTON_LED_SHARED_PIN
        led_on_flag = FALSE;
#endif
    }

    return;
}

btn_status_t btn_read(u16 btn_id)
{
    u16 btn_index = 0;
    bool button_found = FALSE;

    /* loop through the array to look for the correct button */
    for(btn_index = 0; btn_index < NUM_BUTTONS; btn_index++)
    {
        if(button_list.button_array[btn_index].id == btn_id)
        {
            button_found = TRUE;
            break;
        }
    }

    /* If button found */
    if(button_found == TRUE)
    {
        if(button_list.button_array[btn_index].logic == ACTIVE_LOW)
        {
            if(gpio_read_input_pin(button_list.button_array[btn_index].GPIOx,
                                   button_list.button_array[btn_index].PinMask) == GPIO_PIN_RESET)
            {
                return BTN_DOWN;
            }
            else
            {
                return BTN_UP;
            }
        }
        else
        {
            if(gpio_read_input_pin(button_list.button_array[btn_index].GPIOx,
                                   button_list.button_array[btn_index].PinMask) == GPIO_PIN_RESET)
            {
                return BTN_UP;
            }
            else
            {
                return BTN_DOWN;
            }
        }
    }
    else
    {
        return BTN_FAIL;
    }

}

void btn_task(u32 it)
{
    u16 current_interrupt_count = it_get_interrupt_count();
    trip_log_led_xfer_progress_flag_t trip_log_led_xfer_progress = trip_log_get_led_xfer_progress();

    if(trip_log_led_xfer_progress == TRIP_LOG_LED_XFER_NOT_IN_PROGRESS)
    {
        for(u16 array_index = 0; array_index < NUM_BUTTONS; array_index++)
        {
            if( ((current_interrupt_count == 6) || (current_interrupt_count == 22)) )
            {
#if BUTTON_LED_CONFIGURATION == BUTTON_LED_SHARED_PIN

                /* set pin mode to input */
                LL_GPIO_SetPinMode(button_list.button_array[array_index].GPIOx, button_list.button_array[array_index].PinMask, LL_GPIO_MODE_INPUT);

                /* saving the led state so that after sampling button, if the led is on can be
                 * turned back on. Not equal is becasue setting pin low is led on */
                if((READ_BIT(LED_1_GPIO_Port -> ODR, LED_1_Pin)) != LED_1_Pin)
                {
                    led_on_flag = TRUE;
                }
                else
                {
                    led_on_flag = FALSE;
                }
                /* have to set led off for button to read. */
                LED_RED_OFF;
#else
                /* no need to change pin mode if pins are not shared between led and button. */
#endif
            }
            else
            {
                btn_update_state(&button_list.button_array[array_index]);

#if BUTTON_LED_CONFIGURATION == BUTTON_LED_SHARED_PIN

                /* set pin mode back to output */
                LL_GPIO_SetPinMode(button_list.button_array[array_index].GPIOx, button_list.button_array[array_index].PinMask, LL_GPIO_MODE_OUTPUT);

                if(led_on_flag == TRUE)
                {
                    LED_RED_ON;
                }
                else
                {
                    LED_RED_OFF;
                }
#else
                /* no need to change pin mode if pins are not shared between led and button. */
#endif
            }
        }
    }

    return;
}

btn_state_t btn_update_state(btn_t* button)
{
    button->prev_status = button->status;
    button->status = btn_read(button->id);

    u32 dynamic_state_timing;

    if(button->status == BTN_FAIL)
    {
        return BTN_STATE_UNKNOWN;
    }

    switch (button->state)
    {
        case BTN_NOT_PRESSED:
            /* If button status is BTN_DOWN and reached de-bounce timeout,
             * update state to BTN_PRESSED
             */
            if(button->status == BTN_DOWN)
            {
                button->timer_single++;

                if(button->timer_single >= BTN_DEBOUNCE_TIMER_COUNT)
                {
                    button->timer_single = 0;
                    button->timer_double = 0;
                    button->state = BTN_PRESSED;
                }
            }
            else
            {
                button->timer_single = 0;
            }

            break;

        case BTN_PRESSED:
            /* If button status is BTN_UP and reached de-bounce timeout,
             * update state to BTN_WAIT_FOR_DOUBLE_PRESSED
             */
            if(button->status == BTN_UP)
            {
                button->timer_single++;
                button->timer_double = 0;
                if(button->timer_single >= BTN_DEBOUNCE_TIMER_COUNT)
                {
                    button->timer_single = 0;
                    button->state = BTN_WAIT_FOR_DOUBLE_PRESSED;
                }
            }
            else
            {
				#if BOARD_CONFIGURATION == BOARD_SB2
                button->timer_double++;
                button->timer_single = 0;
                if(button->timer_double > BTN_HELD1_TIME)
                {
                    button->timer_double = 0;
                    button->timer_single = 0;
                    button->state = BTN_HELD1_PRESSED;
                }
				#else
                button->timer_single = 0;
                #endif
            }


            break;

        case BTN_PRESSED_SINCE_STARTUP:
            /* If button status is BTN_UP and reached de-bounce timeout,
             * update state to BTN_NOT_PRESSED. Otherwise, stay in this state. */
            if(button->status == BTN_UP)
            {
                button->timer_single++;

                if((button->timer_single) > BTN_DEBOUNCE_TIMER_COUNT)
                {
                    button->state = BTN_NOT_PRESSED;
                }
            }

            break;

        case BTN_WAIT_FOR_DOUBLE_PRESSED:
            /* If button status is BTN_UP and reached double press timeout,
             * update state to BTN_SINGLE_TRIGGERED.
             *
             * If button status is BTN_DOWN and reached de-bounce timeout,
             * update state to BTN_DOUBLE_PRESSED.
             */
            if(button->status == BTN_UP)
            {
                button->timer_single++;

                if(button->timer_single > BTN_TIMER_DOUBLE_PRESSED_TIMEOUT)
                {
                    button->timer_single = 0;
                    button->state = BTN_SINGLE_TRIGGERED;
                }
            }
            else
            {
                button->timer_double++;
                if(button->timer_double > BTN_DEBOUNCE_TIMER_COUNT)
                {
                    button->timer_single = 0;
                    button->timer_double = 0;
                    button->state = BTN_DOUBLE_PRESSED;
                }
            }

            break;

        case BTN_SINGLE_TRIGGERED:
            /* Once single triggered timeout is reached, update state to BTN_NOT_PRESSED*/
           button->timer_single++;
           if(button->timer_single >= BTN_SINGLE_TRIGGERED_TIMEOUT)
           {
               button->state = BTN_NOT_PRESSED;
               button->timer_single = 0;
           }

           break;

        case BTN_DOUBLE_PRESSED:
            /* If button status is BTN_UP and reached debounce timeout,
             * update state to BTN_DOUBLE_TRIGGERED
             */
            if(button->status == BTN_UP)
            {
                button->timer_double++;
                if(button->timer_double > BTN_DEBOUNCE_TIMER_COUNT)
                {
                    button->timer_double = 0;
                    button->state = BTN_DOUBLE_TRIGGERED;
                }
            }
            else
            {
                button->timer_double = 0;
            }

            break;

        case BTN_DOUBLE_TRIGGERED:
            /* Once double triggered timeout is reached, update state to BTN_NOT_PRESSED*/
            button->timer_double++;
            if(button->timer_double >= BTN_DOUBLE_TRIGGERED_TIMEOUT)
            {
                button->state = BTN_NOT_PRESSED;
                button->timer_double = 0;
            }

            break;

#if BOARD_CONFIGURATION == BOARD_SB2
        case BTN_HELD1_PRESSED:
        case BTN_HELD2_PRESSED:
        case BTN_HELD3_PRESSED:

        	//Was intentionally modified to allow for dynamic timing against engineering advice
        	//to account for QA definition of ease of use.
        	switch (button->state)
        	{
        	 	case BTN_HELD1_PRESSED:
        	 		//timing to affectively cancel single button press
        	 		dynamic_state_timing = BTN_HELD1_TIMEOUT;
        	 		 break;
        	    case BTN_HELD2_PRESSED:
        	    	dynamic_state_timing = BTN_HELD2_TIMEOUT;
        	    	break;
        	    case BTN_HELD3_PRESSED:
        	    	dynamic_state_timing = BTN_HELD3_TIMEOUT;
        	         break;
        	        default:
        	        	//Should not execute ever but we will handle the error just in case.
        	        	button->state = BTN_NOT_PRESSED;
        	        	button->timer_double = 0;
        	        	button->timer_single = 0;
        	        	break;
        	}

        /* If button status is BTN_UP and reached debounce timeout,
         * update state to BTN_HELD_TRIGGERED
         */
        if(button->status == BTN_UP)
        {
        	button->timer_single++;
        	button->timer_double = 0;
        	if(button->timer_single > BTN_DEBOUNCE_TIMER_COUNT)
        	{
            	button->timer_single = 0;
            	(button->state)++;// = BTN_HELDX_TRIGGERED;
        	}
        }
        else
        {
        	button->timer_double++;
        	button->timer_single = 0;
        	if(button->timer_double > dynamic_state_timing)
        	{
        		button->timer_double = 0;
        		button->state += 2;// BTN_HELDX_CANCELED;
        	}
        }
        break;

        case BTN_HELD1_TRIGGERED:
        case BTN_HELD2_TRIGGERED:
        case BTN_HELD3_TRIGGERED:
            /* Once held triggered timeout is reached, update state to BTN_NOT_PRESSED*/
            button->timer_double++;
            button->timer_single = 0;
            if(button->timer_double >= BTN_HELD_TRIGGERED_TIMEOUT)
            {
            	button->state = BTN_NOT_PRESSED;
                button->timer_double = 0;
            }
            break;

        case BTN_HELD1_CANCELED:
        case BTN_HELD2_CANCELED:
        //case BTN_HELD3_CANCELED:
        	/* If button status is BTN_UP and reached debounce timeout,
        	 * update state to BTN_NOT_PRESSED to get out of cancled state.
        	 */
        	switch (button->state)
			{
				case BTN_HELD1_CANCELED:
					//timing will take when it wants to execute and subtract the time
					//it should have taken to get here.
					dynamic_state_timing = BTN_HELD2_TIME - BTN_HELD1_TIME - BTN_HELD1_TIMEOUT;
					 break;
				case BTN_HELD2_CANCELED:
					dynamic_state_timing = BTN_HELD3_TIME - BTN_HELD2_TIME - BTN_HELD2_TIMEOUT;
					 break;
				//case BTN_HELD3_CANCELED:
					//dynamic_state_timing = 0; //there is no 4th held state
					//Removing this stated allows it to go directly into overheld
					 //break;
					default:
						//Should not execute ever but we will handle the error just in case.
						button->state = BTN_NOT_PRESSED;
						button->timer_double = 0;
						button->timer_single = 0;
						break;
			}
        	if(button->status == BTN_UP)
        	{
        	  	button->timer_double++;
        	  	button->timer_single = 0;
        	  	if(button->timer_double > BTN_DEBOUNCE_TIMER_COUNT)
        	  	{
        	       	button->timer_double = 0;
        	       	button->state = BTN_NOT_PRESSED;
        	   	}
        	}
        	else
        	{
        		button->timer_double = 0;
				button->timer_single++;
				if(button->timer_single > dynamic_state_timing)
				{
					button->timer_single = 0;
					(button->state)++;// BTN_HELD_NEXT_PRESSED;
				}
        	  	//button->timer_double = 0;
        	}
            break;

        case BTN_OVER_HELD:
			/* If button status is BTN_UP and reached debounce timeout,
			 * update state to BTN_NOT_PRESSED to get out of overheld state.
			 */
			if(button->status == BTN_UP)
			{
				button->timer_double++;
				button->timer_single = 0;
				if(button->timer_double > BTN_DEBOUNCE_TIMER_COUNT)
				{
					button->timer_double = 0;
					button->state = BTN_NOT_PRESSED;
				}
			}
			else
			{
				button->timer_double = 0;
			}
			break;
#endif
        case BTN_STATE_UNKNOWN:
        default:
            button->status = BTN_UP;
            button->timer_single = 0;
            button->timer_double = 0;
    }

    return button->state;
}

status_t btn_add_btn(u16 btn_id, GPIO_TypeDef* GPIOx, u32 PinMask, btn_logic_t logic)
{
    u16 btn_index = 0;

    /* ID shouldn't be 0 */
    if(btn_id == 0)
    {
        return STATUS_FAIL;
    }

    /* check if ID has already been added. */
    for(btn_index = 0; btn_index <= button_list.button_cnt; btn_index++)
    {
        if(button_list.button_array[btn_index].id == btn_id)
        {
            return STATUS_FAIL;
        }
    }

    /* check if exceed number of buttons defined. */
    if(button_list.button_cnt < NUM_BUTTONS)
    {
        if(init_new_btn(&button_list.button_array[button_list.button_cnt]) != STATUS_OK)
        {
            return STATUS_FAIL;
        }

        button_list.button_array[button_list.button_cnt].id = btn_id;
        button_list.button_array[button_list.button_cnt].GPIOx = GPIOx;
        button_list.button_array[button_list.button_cnt].PinMask = PinMask;
        button_list.button_array[button_list.button_cnt].logic = logic;

        button_list.button_cnt++;
    }

    return STATUS_OK;
}

status_t init_new_btn(btn_t* new_button)
{
    /* initialize test button_list */
    if(new_button == NULL)
    {
        return STATUS_FAIL;
    }

    new_button->timer_single = 0;
    new_button->timer_double = 0;
    new_button->state = BTN_PRESSED_SINCE_STARTUP;

    return STATUS_OK;
}

btn_state_t btn_get_button_state(u16 btn_id)
{
    u16 btn_index = 0;
    bool index_found = FALSE;

    /* loop through the array to find the correct button.*/
    for(btn_index = 0; btn_index < NUM_BUTTONS; btn_index++)
    {
        if(button_list.button_array[btn_index].id == btn_id)
        {
            index_found = TRUE;
            break;
        }
    }

    /* if button is found */
    if(index_found == TRUE)
    {
        /* remove And with 0xF0 as we need to include middle states like BTN_HELD10_TRIGGERED*/
#if BOARD_CONFIGURATION == BOARD_SB2
        return (btn_state_t)((button_list.button_array[btn_index].state));
#else
        return (btn_state_t)((button_list.button_array[btn_index].state) & (0xF0));
#endif
    }
    else
    {
        return BTN_STATE_UNKNOWN;
    }
}

btn_status_t btn_get_status(u16 btn_id)
{
    u16 btn_index = 0;
    bool index_found = FALSE;

    /* loop through the array to find the correct button.*/
    for(btn_index = 0; btn_index < NUM_BUTTONS; btn_index++)
    {
        if(button_list.button_array[btn_index].id == btn_id)
        {
            index_found = TRUE;
            break;
        }
    }
    /* if button is found */
    if(index_found == TRUE)
    {
        /* And with 0xF0 to eliminate the temporary middle states like BTN_WAIT_FOR_DOUBLE_PRESSED*/
        return (button_list.button_array[btn_index].status);
    }
    else
    {
        return BTN_FAIL;
    }
}



