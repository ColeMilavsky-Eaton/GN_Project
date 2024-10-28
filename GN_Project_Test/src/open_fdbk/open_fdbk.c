/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 **************************************************************************************************
 *  Written by:         James Frances
 *                      Commercial & Residential Distribution Solutions (CRDS), Eaton
 *						1000 Cherrington Parkway
 *						Moon Township, PA 15108
 *						mobile: (724) 759-5500
 *//**
 * @defgroup open_fdbk open_fdbk component
 *
 * @brief open feedback component handles monitoring the open feedback hardware and keeping track of the path state.

 * NOTE: 1.
 *
 * @file open_fdbk.c
 * @ingroup open_fdbk
 *
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "open_fdbk.h"
#include "main_internal.h"
#include "gpio_api.h"
#include "led_api.h"
#include "stm32g0xx_it_api.h"
#include "firmware.config"
#include "string.h"

static bool open_fdbk_initialized_flag = FALSE;

void open_fdbk_init(void)
{
    if(open_fdbk_initialized_flag == FALSE)
    {
        /* task type to assign tasks */
        task_t t;

        /* assign task */
        t.task_p = open_fdbk_it_task;
        t.it_bits = OPEN_FDBK_PRIMARY_IT | OPEN_FDBK_SECONDARY_180_IT | OPEN_FDBK_SECONDARY_120_IT | OPEN_FDBK_SECONDARY_240_IT;

        task_man_add_task_to_schedule(t);

        path.status = PATH_UNKNOWN;

        open_fdbk_initialized_flag = TRUE;
    }

    return;
}

void open_fdbk_it_task(u32 it)
{
	if(OPEN_FDBK_PRIMARY_IT & (1 << it))
	{
		path.state_fdbk1_first = FDBK1_STATUS;

		#if NUM_POLES_CONFIGURATION == TWO_POLE
			path.state_fdbk2_first = FDBK2_STATUS;
        #elif NUM_POLES_CONFIGURATION == THREE_POLE
			path.state_fdbk2_first = FDBK2_STATUS;
			path.state_fdbk3_first = FDBK3_STATUS;
		#endif
	}
	switch(it_get_input_voltage_scenario())
	{
		#warning both 1p need to be tested
	    case SCENARIO_1P:
	    case SCENARIO_2P_ONLY_1P_CONNECTED:
	    case SCENARIO_2P_PRIMARY_SAME_PHASE:
	    	if((path.state_fdbk1_first == PATH_CLOSED_1P)
	    		|| (path.state_fdbk2_first == PATH_CLOSED_1P))
	    	{
	    		path.status = PATH_CLOSED;
#ifdef SB20_PUL
	    		LOAD_PWR_LED_Green_ON;
#else
	    		LOAD_PWR_LED_ON;
#endif
	    	}
	    	else
	    	{
	    		path.status = PATH_OPEN;
#ifdef SB20_PUL
	    		LOAD_PWR_LED_Green_OFF;
#else
	    		LOAD_PWR_LED_OFF;
#endif
	    	}
	    	break;


	    case SCENARIO_2P_PRIMARY_LEAD_180:
	    	if(OPEN_FDBK_SECONDARY_180_IT & (1 << it))
	    	{
	    		path.state_fdbk1_second = FDBK1_STATUS;
	    		path.state_fdbk2_second = FDBK2_STATUS;
	    		set_path_status_2poles();
	    	}
	        break;

	    case SCENARIO_2P_PRIMARY_LEAD_120:
	    	if(OPEN_FDBK_SECONDARY_120_IT & (1 << it))
	    	{
	    		path.state_fdbk1_second = FDBK1_STATUS;
	    		path.state_fdbk2_second = FDBK2_STATUS;
	    		set_path_status_2poles();
	    	}
	    	break;

	    case SCENARIO_2P_PRIMARY_LEAD_240:
	    	if(OPEN_FDBK_SECONDARY_240_IT & (1 << it))
	    	{
	    		path.state_fdbk1_second = FDBK1_STATUS;
	    		path.state_fdbk2_second = FDBK2_STATUS;
	    		set_path_status_2poles();
	    	}
	        break;
	    case SCENARIO_3P:
	    	// logic has not been created to account for 3P.
	        break;

	    default:
	        break;
    }
    return;
}

void set_path_status_2poles(void)
{
	u8 state = 6;
	state = ((path.state_fdbk1_first << 3 )
			| (path.state_fdbk2_first << 2)
			| (path.state_fdbk1_second << 1)
		 	| (path.state_fdbk2_second));

	switch(state)
	{
		case PATH_OPEN_2P:
			path.status = PATH_OPEN;
#ifdef SB20_PUL
			LOAD_PWR_LED_Green_OFF;
#else
			LOAD_PWR_LED_OFF;
#endif
			break;

		case PATH_CLOSED_FDBK1_PRIMARY_2P:
		case PATH_CLOSED_FDBK2_PRIMARY_2P:
			path.status = PATH_CLOSED;
#ifdef SB20_PUL
			LOAD_PWR_LED_Green_ON;
#else
			LOAD_PWR_LED_ON;
#endif
			break;

		case PATH_STUCK1_2P:
		case PATH_STUCK2_2P:
		case PATH_STUCK3_2P:
		case PATH_STUCK4_2P:
		case PATH_STUCK1_2P_LOAD:
		case PATH_STUCK2_2P_LOAD:
			path.status = PATH_1P_STUCK;
			blink_load_powered_led();
		break;

		default:
			path.status = PATH_UNKNOWN;
			blink_load_powered_led();
		break;
	}
	return;
}

inline fdbk_status_t get_path_status(void)
{
	return path.status;
}

void blink_load_powered_led(void)
{
	//down and dirty implementation for quick testing.
	static u8 blink_counter = 0;
	blink_counter++;
	if(blink_counter > 60)
	{
		blink_counter = 0;
	}

	if(blink_counter < 30)
	{
#ifdef SB20_PUL
	    LOAD_PWR_LED_Green_ON;
#else
		LOAD_PWR_LED_ON;
#endif
	}
	else
	{
#ifdef SB20_PUL
	    LOAD_PWR_LED_Green_OFF;
#else
	    LOAD_PWR_LED_OFF;
#endif
	}
}
