/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 ***************************************************************************************************
 *  Written by:         Sonal Barge
 *                      EIIC Eaton Electrical
 *                      Magarpatta City, Hadapsar
 *                      Pune MH-411 013
 *//**
 * @defgroup led led component
 *
 @brief The LED component handles initialization, prioritization of led on/off requests and determining state of LED
 *
 *  @image rtf led_component_process_flow.png "led component process flow diagram"
 *
 *  @image html led_component_process_flow.png "led component process flow diagram"
 *
 *  @image rtf led_state_process_flow.png "led state machine process flow diagram"
 *
 *  @image html led_state_process_flow.png "led state machine process flow diagram"
 *
 *  @image rtf led_state_machine.png "led state machine diagram"
 *
 *  @image html  led_state_machine.png "led state machine diagram"
 *
 * # Overview
 * The LED component performs the following functions:
 *  initialization of the led component and add task to task manager.
 *
 *  request to Turn on/off, blink the current LED.
 *   Other component will request for led along with LED_ID(color), state.
 *   For blinking period and rate need to be predefined based on type of request. eg: rainbow flash request period will be 2 sec rate will be 290msec
 *   The request is added to queue.
 *
 *   LED state machine is called every interrupt count.
 *   If in Idle state it will check for pending request.
 *   If pending request in queue it will of each request.
 *   Request with highest priority will be served.
 *   For blinking rate and period are mentioned in millisec.
 *   counter used for blinking rate or period will decrement every interrupt count that is 8.3/16 0.51msec every IT.
 *   hence error in counting actual rate or period can vary upto 0.51msec.
 *
 * NOTE: 1. User should define the number of LEDs that will be used in led_api.h
 *       2. User should define a flashing rate for each type of blink.
 * @file led.c
 * @ingroup led
 *
 *
 *//*
 *
 **************************************************************************************************/

#include "led_.h"
#include "main_internal.h"
#include "types.h"
#include "task_manager_api.h"
#include "btn_api.h"
#include "string.h"

static u8 service_index;

u8 led_rainbow_delay_counter;
bool led_rainbow_request_flag;
bool led_idenfify_me_request_flag;
bool led_idenfify_me_on_going_flag;
u8 led_identify_me_blink_mode;

led_states_t   led_state;
led_request_list_t led_request_list;

led_key_t key;
// reuse the led_rainbow_param_t as it has all for identify_me. Do not get confused by the name

extern bool empty_led_rate_received_flag;
extern st_rgb_led_blink rgb_led_blink;

void led_init(void)
{
    /* task type to assign tasks */
    task_t t;

    /* assign task */
    t.task_p = led_task;
    t.it_bits = TSK_MAN_IT_ALL;

    task_man_add_task_to_schedule(t);

  #if NUM_POLES_CONFIGURATION == TWO_POLE
    /* assign task */
    t.task_p = led_rainbow_task;
    t.it_bits = TSK_MAN_IT_1 | TSK_MAN_IT_17;

    task_man_add_task_to_schedule(t);
  #endif

    // Identify Me LED task
    /* assign task */
    t.task_p = led_identify_me_task;
    t.it_bits = TSK_MAN_IT_3 | TSK_MAN_IT_19;
    task_man_add_task_to_schedule(t);

    /* clear/initialize array */
    memset(led_request_list.led_request_array, LED_CLEAR, sizeof(led_request_list.led_request_array));

    led_request_list.led_request_cnt = 0;
    led_state = LED_IDLE;
    service_index = 0;
    led_rainbow_delay_counter = 0;
    led_rainbow_request_flag = FALSE;
    led_idenfify_me_request_flag = FALSE;
    led_idenfify_me_on_going_flag = FALSE;
    key = KEY_FREE;


    		//rgb_led_blink.rgb_led_color              = LED_YELLOW;
    		//rgb_led_blink.blinking_time_interval     = 240;
    		//rgb_led_blink.blinking_time_expire       = 0xFF;
    		//rgb_led_blink.start_blinking_leds_flag   = TRUE;

return;
}
void led_task(u32 it)
{
    //Pass the LED request array in FIFO format

    if(led_request_list.led_request_cnt != 0)
    {
        if(led_state == LED_IDLE)
        {
            if(service_index >= 10)
            {
                service_index = 0;
            }
            if(led_request_list.led_request_array[service_index].id != LED_UNASSIGNED_COLOR)
            {
                led_state = LED_STATE_TRANSITION;
            }
        }
        //Execute request only if valid LED ID present in array
        if( led_state == LED_STATE_TRANSITION)
        {
            switch(led_request_list.led_request_array[service_index].request_state)
            {
               case LED_TURN_ON:
                   //Execute LED Turn On request
                    led_turn_on (led_request_list.led_request_array[service_index].id);
                    led_state = LED_IDLE;
                    led_reset_request_array();
                    //printf("\nLED_TURN_ON\n");
                    break;
                case LED_TURN_OFF:
                    //Execute LED Turn Off request
                    led_turn_off();
                    led_state= LED_IDLE;
                    led_reset_request_array();
                    //printf("\nLED_TURN_OFF\n");
                    break;

                 default:
                      break;
            }
        }
    }
}
void led_turn_on (led_color_t led)
{
    //Turn On LED based on requested color
    switch(led)
    {
        case LED_RED:
            LED_RED_ON;
          #if LED_CONFIGURATION == RGB_LED
            LED_BLUE_OFF;
            LED_GREEN_OFF;
            break;
        case LED_GREEN:
            LED_RED_OFF;
            LED_BLUE_OFF;
            LED_GREEN_ON;
            break;
        case LED_BLUE:
            LED_RED_OFF;
            LED_BLUE_ON;
            LED_GREEN_OFF;
            break;
        case LED_YELLOW:
            LED_RED_ON;
            LED_BLUE_OFF;
            LED_GREEN_ON;
            break;
        case LED_CYAN:
            LED_RED_OFF;
            LED_BLUE_ON;
            LED_GREEN_ON;
            break;
        case LED_MAGENTA:
            LED_RED_ON;
            LED_BLUE_ON;
            LED_GREEN_OFF;
            break;
        case LED_WHITE:
            LED_RED_ON;
            LED_BLUE_ON;
            LED_GREEN_ON;
          #endif

            break;
        default:
            /* Do nothing */
            break;
    }

}


void led_turn_off (void)
{
    //Turn Off LED
    LED_RED_OFF;
  #if LED_CONFIGURATION == RGB_LED
    LED_BLUE_OFF;
    LED_GREEN_OFF;
  #endif

    return;
}

void led_rainbow(led_rainbow_param_t* rainbow)
{
    //Flash the LED
    static led_color_t led_num = LED_RED;// used for rainbow flash

    if(rainbow->led_rainbow_period_counter != LED_FLASH_1_SECOND_PERIOD_CNT * rainbow->led_rainbow_period)        //Check if flash_period count has completed
    {
        if(rainbow->led_rainbow_rate_counter >= (rainbow->led_rainbow_rate * LED_FLASH_RATE_MINIMUM_TIME)/LED_FLASH_RATE_CNT)        //Check the flash_rate count and turn on/off based on prev state
        {
            rainbow->led_rainbow_rate_counter = 0; //

            // Handle LED cycling
            led_request(led_num, LED_TURN_ON);
            led_num++;
            if (led_num > LED_WHITE)
            {
               led_num = LED_RED;
            }

        }
        rainbow->led_rainbow_rate_counter++;
    }
    rainbow->led_rainbow_period_counter++;
}

void led_rainbow_task(u32 it)
{
    static led_rainbow_param_t rainbow = {0};

    //Check if flash flag is enabled at startup and user button is not pressed and request for rainbow sequence
    if(led_rainbow_request_flag==TRUE)
    {
        // Flash LED rainbow to indicate unit reset if button is not pressed
        if((led_get_key_status() == KEY_FREE))
        {
            led_set_key_status(KEY_TAKEN_BY_RAINBOW);
            rainbow.led_rainbow_rate = CYCLE_LED_RATE;
            rainbow.led_rainbow_period = FLASH_LED_2_SECOND_PERIOD;
            rainbow.led_rainbow_period_counter = 0;
            rainbow.led_rainbow_rate_counter = ((CYCLE_LED_RATE * LED_FLASH_RATE_MINIMUM_TIME)/LED_FLASH_RATE_CNT);
            led_rainbow_request_flag = FALSE;
        }
    }

    if((led_get_key_status() == KEY_TAKEN_BY_RAINBOW))
    {
        led_rainbow(&rainbow);

        if(rainbow.led_rainbow_period_counter >= (FLASH_LED_2_SECOND_PERIOD*  LED_FLASH_1_SECOND_PERIOD_CNT))
        {
            led_request(LED_RED, LED_TURN_OFF);
            led_reset_key_status();
        }
    }

    return;
}

void led_identify_me(led_rainbow_param_t* idenfify_me)
{
    //Flash the LED
    static led_color_t led_num = LED_WHITE;// used for identify_me flash

    if(idenfify_me->led_rainbow_period_counter != LED_FLASH_2_SECOND_PERIOD_CNT * idenfify_me->led_rainbow_period)        //Check if flash_period count has completed
    {
        // Reset rate counter every 2 seconds
        if(idenfify_me->led_rainbow_rate_counter >= (2 * idenfify_me->led_rainbow_rate * LED_FLASH_RATE_MINIMUM_TIME)/LED_FLASH_RATE_CNT)        //Check the flash_rate count and turn on/off based on prev state
        {
            idenfify_me->led_rainbow_rate_counter = 0;
        }

        // in the 1-2 second period, LED OFF
        if(idenfify_me->led_rainbow_rate_counter >= (idenfify_me->led_rainbow_rate * LED_FLASH_RATE_MINIMUM_TIME)/LED_FLASH_RATE_CNT)        //Check the flash_rate count and turn on/off based on prev state
        {
           led_request(led_num, LED_TURN_OFF);
        }
        // in the 0-1 second period, LED ON
        else
        {
            led_request(led_num, LED_TURN_ON);
        }
        idenfify_me->led_rainbow_rate_counter++;
    }

    idenfify_me->led_rainbow_period_counter++;
}

void led_identify_me_task(u32 it)
{
    static led_rainbow_param_t identify_me = {0};
    static led_key_t saved_key = KEY_FREE;

    //Check if identify_me requested is received
    if(led_idenfify_me_request_flag == TRUE)
    {
        // Flash LED white for 30 seconds for identify me
        if(led_get_key_status() != KEY_TAKEN_BY_IDENTIFY_ME)
        {
            // save the current LED key
            if(led_get_key_status() != KEY_FREE)
            {
                saved_key = led_get_key_status();
            }
            led_set_key_status(KEY_TAKEN_BY_IDENTIFY_ME);
            // Cheil requirement - In the factory mode, Identify_Me LED shall blink for 1 second period with a rate of 3-4 times per second.
            identify_me.led_rainbow_rate = led_identify_me_blink_mode == LED_BLINKING_DEFAULT_MODE ? IDENTIFY_ME_LED_RATE : LED_FLASH_RATE_MINIMUM_TIME;
            identify_me.led_rainbow_period = led_identify_me_blink_mode == LED_BLINKING_DEFAULT_MODE ? FLASH_LED_30_SECOND_PERIOD : FLASH_LED_1_SECOND_PERIOD;
            identify_me.led_rainbow_period_counter = 0;
            identify_me.led_rainbow_rate_counter = ((identify_me.led_rainbow_rate  * LED_FLASH_RATE_MINIMUM_TIME)/LED_FLASH_RATE_CNT);
            led_idenfify_me_request_flag = FALSE;
            led_idenfify_me_on_going_flag = TRUE;
        }
    }

    if((led_get_key_status() == KEY_TAKEN_BY_IDENTIFY_ME))
    {
        led_identify_me(&identify_me);

        if(identify_me.led_rainbow_period_counter >= (identify_me.led_rainbow_period * LED_FLASH_1_SECOND_PERIOD_CNT))
        {
            led_request(LED_WHITE, LED_TURN_OFF);
            led_idenfify_me_on_going_flag = FALSE;
            led_reset_key_status();
            led_set_key_status(saved_key);
        }
    }
    else
    {
        //else the set key was not successful or key was taken
        //reset the on going flag to false

        led_idenfify_me_on_going_flag = FALSE;
    }
}

void led_request(led_color_t led_id, led_request_states_t state)
{
    static u8 request_index=0;
    //Add request to queue

    if((led_id != LED_UNASSIGNED_COLOR) && (led_request_list.led_request_cnt<REQUEST_ARRAY_SIZE))
    {
        if(led_request_list.led_request_array[request_index].id == LED_UNASSIGNED_COLOR)
        {
            led_request_list.led_request_array[request_index].id = led_id;
            led_request_list.led_request_array[request_index].request_state = state;
            led_request_list.led_request_cnt++;
            request_index++;
        }
        if(request_index >= 10)
        {
            request_index=0;
        }
    }
}

void led_reset_request_array(void)
{
    //Reset all the parameters in array after execution
    led_request_list.led_request_array[service_index].id = LED_UNASSIGNED_COLOR;
    led_request_list.led_request_array[service_index].request_state = LED_NO_REQUEST;
    led_request_list.led_request_cnt--;
    service_index++;

}

inline led_key_t led_get_key_status(void)
{
    return key;
}

void led_set_key_status(led_key_t status)
{
    if(status == KEY_TAKEN_BY_IDENTIFY_ME)
    {
       // Identify me has a higher priority
    }
    else if(key != KEY_FREE)
    {
        // others the same priority
        return;
    }

    key = status;

    return;
}

inline void led_reset_key_status(void)
{
    key = KEY_FREE;
}

void led_request_rainbow(void)
{
    led_rainbow_request_flag = TRUE;

    return;
}


#if defined USE_NETWORK_STATUS_LED_APPROACH
/**************************************************************************************************/
/**
 * @brief  Blinking the LED(s) for provisioning states and SBLCP command
 *
 * @return None
 *
 * @exception none
 *
 **************************************************************************************************/
void BlinkingLEDStateMachine()
{
	static uint8_t led_status = 0;
	//check current LED status
	if((led_get_key_status() == KEY_FREE)
		&& (get_primary_switch_debounced_state() == SW_CLOSED))
	{
		led_set_key_status(KEY_TAKEN_BY_PROVISIONING);
	}
	if(led_get_key_status() != KEY_TAKEN_BY_PROVISIONING)
	{
		return;
	}
	if ((rgb_led_blink.blinking_counter < rgb_led_blink.blinking_time_expire)
			&& (get_primary_switch_debounced_state() == SW_CLOSED))
	{
		if((rgb_led_blink.blinking_time_interval == BLINK_FOREVER) && (rgb_led_blink.blinking_time_expire == BLINK_FOREVER))
		{
			led_request(rgb_led_blink.rgb_led_color, LED_TURN_ON);
		}
		else
		{
			if (!(rgb_led_blink.blinking_counter%rgb_led_blink.blinking_time_interval))
			{
				if (led_status == 0)
				{
					led_status = 1;
					led_request(rgb_led_blink.rgb_led_color, LED_TURN_OFF);
				}
				else
				{
					led_status = 0;
					led_request(rgb_led_blink.rgb_led_color, LED_TURN_ON);
				}
				if (rgb_led_blink.blinking_time_expire == BLINK_FOREVER)
				{
					rgb_led_blink.blinking_counter = 0;
				}
			}
			rgb_led_blink.blinking_counter++;
		}
	}
	else
	{
		//Execute LED Turn Off request
		led_turn_off();
		//reset LED states to stop operation and blinking
		memset(&rgb_led_blink, 0x00, sizeof(st_rgb_led_blink));
		led_reset_key_status();
	}
}
#else
/**************************************************************************************************/
/**
 * @brief  Blinking the LED(s) for provisioning states and SBLCP command
 *
 * @return None
 *
 * @exception none
 *
 **************************************************************************************************/
void BlinkingLEDStateMachine()
{
	static uint8_t led_status = 0;
	//TODO if the time interval has changed it may be worth resetting our counter.

	//check current LED status
	if((led_get_key_status() == KEY_FREE || led_get_key_status() == KEY_TAKEN_BY_IDENTIFY_ME)
		&& (get_primary_switch_debounced_state() == SW_CLOSED))
	{
        // if not a empty blinking rate command received
        if(!empty_led_rate_received_flag)
        {

            if(led_get_key_status() == KEY_TAKEN_BY_IDENTIFY_ME)
            {
                led_request(LED_WHITE, LED_TURN_OFF);
                led_idenfify_me_on_going_flag = FALSE;
                led_reset_key_status();
            }
	        led_set_key_status(KEY_TAKEN_BY_PROVISIONING);
	    }
	}
	if(led_get_key_status() != KEY_TAKEN_BY_PROVISIONING)
	{
		return;
	}
	if ((rgb_led_blink.blinking_counter < rgb_led_blink.blinking_time_expire)
			&& (get_primary_switch_debounced_state() == SW_CLOSED))
	{
		//Allow for solid on.
		if((rgb_led_blink.blinking_time_interval == BLINK_FOREVER) && (rgb_led_blink.blinking_time_expire == BLINK_FOREVER))
		{
			led_request(rgb_led_blink.rgb_led_color, LED_TURN_ON);
		}
		else
		{
			if (!(rgb_led_blink.blinking_counter%rgb_led_blink.blinking_time_interval))
			{
				if (led_status == 0)
				{
					led_status = 1;
					led_request(rgb_led_blink.rgb_led_color, LED_TURN_OFF);
				}
				else
				{
					led_status = 0;
					led_request(rgb_led_blink.rgb_led_color, LED_TURN_ON);
				}
				if (rgb_led_blink.blinking_time_expire == BLINK_FOREVER)
				{
					rgb_led_blink.blinking_counter = 0;
				}
			}
			rgb_led_blink.blinking_counter++;
		}
	}
	else
	{
		//Execute LED Turn Off request
		led_turn_off();
		//reset LED states to stop operation and blinking
		memset(&rgb_led_blink, 0x00, sizeof(st_rgb_led_blink));
		led_reset_key_status();
		rgb_led_blink.start_blinking_leds_flag = FALSE;
	}
}

#endif
/**************************************************************************************************/
/**
 * @brief  To check if RGB LED is blinking or not
 *
 * @return None
 *
 * @exception none
 *
 **************************************************************************************************/
bool IsLED_Blinking()
{
    return rgb_led_blink.start_blinking_leds_flag;
}

