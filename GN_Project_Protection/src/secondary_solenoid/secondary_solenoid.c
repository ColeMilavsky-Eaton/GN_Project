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
 * @defgroup secondary_solenoid secondary_solenoid component
 *
 * @brief secondary_solenoid component handles monitoring and control of the secondary solenoid driver

 * NOTE: 1.
 *
 * @file secondary_solenoid.c
 * @ingroup secondary_solenoid
 *
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "secondary_solenoid.h"
#include "main_internal.h"
#include "gpio_api.h"
#include "primary_switch_api.h"
#include "firmware.config"
//#include "string.h"

#if defined REPORT_SS_DIAGNOSTICS
#include "usart.h"

extern M2M_UART_COMMN *pm2m_uart_comm;
#endif

extern bool startup_conf;

static bool ss_initialized_flag;

secondary_solenoid_t secondary_solenoid;

void secondary_solenoid_init(void)
{
	u16 cap_adc_val = 0;
    if(ss_initialized_flag == FALSE)
    {
        /* task type to assign tasks */
        task_t t;

        /* assign task */
		t.task_p = secondary_solenoid_it_task;
		#if defined USE_SS_COMBINED_PULSE_TIMING
		t.it_bits = SS_START_DRIVE_IT | SS_STOP_DRIVE_IT; //May allow for transitions on negative as well. future improvment.
		#else
		t.it_bits = SS_START_OPEN_DRIVE_IT | SS_START_CLOSE_DRIVE_IT | SS_STOP_DRIVE_IT; //May allow for transitions on negative as well. future improvment.
		#endif
		task_man_add_task_to_schedule(t);

        secondary_solenoid.current_state = SS_UNKNOWN; //Should use first set of cycles to detect state.
        secondary_solenoid.requested_state = SS_UNKNOWN;
        secondary_solenoid.state_change_requested = FALSE;
        secondary_solenoid.retry_stuck_counter = TIMES_TO_RETRY_STUCK;
        secondary_solenoid.unexpected_change_counter = TIMES_TO_RESET_UNEXPECTED_CHANGE;
        secondary_solenoid.wait_transition_detection = DELAY_CYCLES_FOR_SS_CHANGE_DETECTION;

		#if defined ADD_SS_REQUEST_DELAY
        secondary_solenoid.wait_next_open_request = 0;//multiply by 3 because it is called 3 times in a half cycle
        secondary_solenoid.wait_next_close_request = 0;//multiply by 3 because it is called 3 times in a half cycle
		#endif

        if(!startup_conf)
	    {
	    	//The cap is being used as a memory state. If it is a logic 0
	    	//we will auto open the breaker.
	    	secondary_solenoid.requested_state = SS_OPEN;
	    	secondary_solenoid.state_change_requested = TRUE;
	    }

        ss_initialized_flag = TRUE;
    }

    return;
}

void secondary_solenoid_it_task(u32 it)
{
	it_number_t bitwise_it;
	bitwise_it = (it_number_t)(1 << it);

	switch(bitwise_it)
	{
#if defined USE_SS_COMBINED_PULSE_TIMING
	case SS_START_DRIVE_IT:
		#if defined ADD_SS_REQUEST_DELAY
		if(secondary_solenoid.wait_next_request > 0)
		{
			//Must wait to protect secondary solenoid from overheating.
			secondary_solenoid.wait_next_open_request--;
			return;
		}
		#endif
		if(secondary_solenoid.state_change_requested)
		{
			switch(secondary_solenoid.requested_state)
			{
				case SS_CLOSED:
					if(secondary_solenoid.current_state != SS_STUCK_OPEN)
					{
						secondary_solenoid.current_state = SS_CLOSING;
					}
					SS_SET_CLOSE;
					SS_RESET_OPEN;
					break;

				default: //defaulting to open request if somehow requested without a specific state.
					//setting this is necessary to prevent unintended lockup
					secondary_solenoid.requested_state = SS_OPEN;
				case SS_OPEN:
					if(secondary_solenoid.current_state != SS_STUCK_CLOSED)
					{
						secondary_solenoid.current_state = SS_OPENING;
					}
					SS_SET_OPEN;
					SS_RESET_CLOSE;
					break;
			}
			secondary_solenoid.state_change_requested = FALSE;
			secondary_solenoid.wait_transition_detection = DELAY_CYCLES_FOR_SS_CHANGE_DETECTION;

			//start backup timer.
		    LL_TIM_DisableCounter(SS_BACKUP_TIMER);
		    LL_TIM_SetAutoReload(SS_BACKUP_TIMER, SS_DEFAULT_TIMEOUT);
		    LL_TIM_SetCounter(SS_BACKUP_TIMER, 0);
		    LL_TIM_EnableCounter(SS_BACKUP_TIMER);

			#if defined ADD_SS_REQUEST_DELAY
		    secondary_solenoid.wait_next_open_request = DELAY_HALF_CYCLES_FOR_SS_NEXT_REQUEST;
			#endif
		}
		break;
#else

	case SS_START_OPEN_DRIVE_IT:
		#if defined ADD_SS_REQUEST_DELAY
		if(secondary_solenoid.wait_next_open_request > 0)
		{
			//Must wait to protect secondary solenoid from overheating.
			return;
		}
		#endif

		if(secondary_solenoid.state_change_requested && (secondary_solenoid.requested_state != SS_CLOSED))
		{
			secondary_solenoid.requested_state = SS_OPEN; //ensure requested state is either open or close and default to open if unknown
			if(secondary_solenoid.current_state != SS_STUCK_CLOSED)
			{
				secondary_solenoid.current_state = SS_OPENING;
			}
			SS_SET_OPEN;
			SS_RESET_CLOSE;

			secondary_solenoid.state_change_requested = FALSE;
			secondary_solenoid.wait_transition_detection = DELAY_CYCLES_FOR_SS_CHANGE_DETECTION;

			//start backup timer.
			LL_TIM_DisableCounter(SS_BACKUP_TIMER);
			LL_TIM_SetAutoReload(SS_BACKUP_TIMER, SS_DEFAULT_TIMEOUT);
			LL_TIM_SetCounter(SS_BACKUP_TIMER, 0);
			LL_TIM_EnableCounter(SS_BACKUP_TIMER);

			#if defined ADD_SS_REQUEST_DELAY
			secondary_solenoid.wait_next_open_request = DELAY_HALF_CYCLES_FOR_SS_NEXT_REQUEST;
			#endif
		}
		break;

	case SS_START_CLOSE_DRIVE_IT:

		#if defined ADD_SS_REQUEST_DELAY
		if(secondary_solenoid.wait_next_close_request > 0)
		{
			//Must wait to protect secondary solenoid from overheating.
			return;
		}
		#endif

		if(secondary_solenoid.state_change_requested && (secondary_solenoid.requested_state == SS_CLOSED))
		{
			if(secondary_solenoid.current_state != SS_STUCK_OPEN)
			{
				secondary_solenoid.current_state = SS_CLOSING;
			}
			SS_SET_CLOSE;
			SS_RESET_OPEN;

			secondary_solenoid.state_change_requested = FALSE;
			secondary_solenoid.wait_transition_detection = DELAY_CYCLES_FOR_SS_CHANGE_DETECTION;

			//start backup timer.
			LL_TIM_DisableCounter(SS_BACKUP_TIMER);
			LL_TIM_SetAutoReload(SS_BACKUP_TIMER, SS_DEFAULT_TIMEOUT);
			LL_TIM_SetCounter(SS_BACKUP_TIMER, 0);
			LL_TIM_EnableCounter(SS_BACKUP_TIMER);

			#if defined ADD_SS_REQUEST_DELAY
			secondary_solenoid.wait_next_close_request = DELAY_HALF_CYCLES_FOR_SS_NEXT_REQUEST;
			#endif
		}

	#endif

		break;

	case SS_STOP_DRIVE_IT:
		LL_TIM_DisableCounter(SS_BACKUP_TIMER);
		SS_RESET_OPEN;
		SS_RESET_CLOSE;
		break;

	default:
		//nothing will happen for unsupported or disabled ITs.
		break;

	}
    return;
}


//inputs to this function. secondary_solenoid, primary open, open/close status
bool secondary_solenoid_main_task(void)
{
	secondary_solenoid_manual_close_task();

	//this gets called every half cycle. we are using these as spacers so this should ensure it
	//waits at least 10 seconds before a back to back fire.
#if defined ADD_SS_REQUEST_DELAY
	if(secondary_solenoid.wait_next_close_request > 0)
	{
		secondary_solenoid.wait_next_close_request--;
	}
	if(secondary_solenoid.wait_next_open_request > 0)
	{
		secondary_solenoid.wait_next_open_request--;
	}
#endif

    if((get_primary_switch_debounced_state() != SW_CLOSED)
			|| (secondary_solenoid.state_change_requested == TRUE))
	{
		//Below states are meant to allow for detection of completed transition state.
		//Must wait for primary to close prior to checking for completion or accuracy of current state.
		return FALSE;
	}
	if(secondary_solenoid.wait_transition_detection > 0)
	{
		//Must wait for feedback circuit to stablize
		if(check_ss_known())
		{
			secondary_solenoid.wait_transition_detection--;
		}
		return FALSE;
	}

	switch(secondary_solenoid.current_state)
	{
		case SS_OPEN:
			if(!check_ss_open())
			{
				if(secondary_solenoid.unexpected_change_counter > 0)
				{
					secondary_solenoid.unexpected_change_counter--;
					if(secondary_solenoid.unexpected_change_counter > 0)
					{
						#if defined REPORT_SS_DIAGNOSTICS
						//report unexpected closing of secondary solenoid
						queue_uart_message(pm2m_uart_comm, BREAKER_FAULT_STATE, SECONDARY_SWITCH_UNEXPECTED_CLOSED, NO_PAYLOAD_1, NO_PAYLOAD_2);
						#endif
						secondary_solenoid.requested_state = SS_OPEN;
						secondary_solenoid.state_change_requested = TRUE;
					}
					else
					{
						#if defined REPORT_SS_DIAGNOSTICS
						//report overload of unexpected change
						queue_uart_message(pm2m_uart_comm, BREAKER_FAULT_STATE, SECONDARY_SWITCH_UNEXPECTED_OVERLOAD, NO_PAYLOAD_1, NO_PAYLOAD_2);
						#endif
						return TRUE;
					}
				}
			}
			break;
		case SS_CLOSED:
			if(!check_ss_closed())
			{
				if(secondary_solenoid.unexpected_change_counter > 0)
				{
					secondary_solenoid.unexpected_change_counter--;
					if(secondary_solenoid.unexpected_change_counter > 0)
					{
						#if defined REPORT_SS_DIAGNOSTICS
						//report unexpected opening of secondary solenoid
						queue_uart_message(pm2m_uart_comm, BREAKER_FAULT_STATE, SECONDARY_SWITCH_UNEXPECTED_OPEN, NO_PAYLOAD_1, NO_PAYLOAD_2);
						#endif
						//This may happen due to a mechanical design to blow open the secondary contacts with the primary.
						//Have to determine if this should just leave the contact open.

						//secondary_solenoid.requested_state = SS_CLOSED;
						//secondary_solenoid.state_change_requested = TRUE;
					}
					else
					{
						#if defined REPORT_SS_DIAGNOSTICS
						//report overload of unexpected change
						queue_uart_message(pm2m_uart_comm, BREAKER_FAULT_STATE, SECONDARY_SWITCH_UNEXPECTED_OVERLOAD, NO_PAYLOAD_1, NO_PAYLOAD_2);
						#endif
						return TRUE;
					}
				}
			}
			break;
		case SS_OPENING:
			secondary_solenoid.retry_stuck_counter = TIMES_TO_RETRY_STUCK;
			if(check_ss_open())
			{
				secondary_solenoid.current_state = SS_OPEN;
			}
			else
			{
				secondary_solenoid.current_state = SS_STUCK_CLOSED;
				secondary_solenoid.state_change_requested = TRUE;
			}
			break;
		case SS_CLOSING:
			secondary_solenoid.retry_stuck_counter = TIMES_TO_RETRY_STUCK;
			if(check_ss_closed())
			{
				secondary_solenoid.current_state = SS_CLOSED;
			}
			else
			{
				secondary_solenoid.current_state = SS_STUCK_OPEN;
				secondary_solenoid.state_change_requested = TRUE;
			}
			break;
		case SS_STUCK_OPEN:
			if(check_ss_closed())
			{
				secondary_solenoid.current_state = SS_CLOSED;
			}
			else
			{
				if(secondary_solenoid.retry_stuck_counter > 0)
				{
					secondary_solenoid.retry_stuck_counter--;
					if(secondary_solenoid.retry_stuck_counter > 0)
					{
						//currently do not report a single stuck state?
						secondary_solenoid.state_change_requested = TRUE;
					}
					else
					{
						#if defined REPORT_SS_DIAGNOSTICS
						//report you exceeded your stuck counter limit.
						queue_uart_message(pm2m_uart_comm, BREAKER_FAULT_STATE, SECONDARY_SWITCH_STUCK_OPEN, NO_PAYLOAD_1, NO_PAYLOAD_2);
						#endif
						return TRUE;
					}
				}
			}
			break;
		case SS_STUCK_CLOSED:
			if(check_ss_open())
			{
				secondary_solenoid.current_state = SS_OPEN;
			}
			else
			{
				if(secondary_solenoid.retry_stuck_counter > 0)
				{
					secondary_solenoid.retry_stuck_counter--;
					if(secondary_solenoid.retry_stuck_counter > 0)
					{
						//currently do not report a single stuck state?
						secondary_solenoid.state_change_requested = TRUE;
					}
					else
					{
						#if defined REPORT_SS_DIAGNOSTICS
						//report you exceeded your stuck counter limit.
						queue_uart_message(pm2m_uart_comm, BREAKER_FAULT_STATE, SECONDARY_SWITCH_STUCK_CLOSED, NO_PAYLOAD_1, NO_PAYLOAD_2);
						#endif
						return TRUE;
					}
				}
			}
			break;
			//should not get in default state.
			//If we somehow get here follow same protocol for unknown state.
		default:
		case SS_UNKNOWN:
			if(check_ss_closed())
			{
			    // When path is closed, it is conclusive that both switches are closed
				secondary_solenoid.current_state = SS_CLOSED;
			}
			else if(check_ss_open())
			{
			    /*
			     *  When path is open, it is not conclusive that the secondary switch
			     *  is open as the primary switch status is unknown.
			     *  Also the primary switch status is based on the hall sensor only,
			     *  if the primary handle is pushed toward closed position while the
			     *  breaker is tripped, although the primary switch status can be reported
			     *  closed, it is not really closed.
			     *
			     *  Reverted. Changes made in primary switch states detection, the primary switch c
			     *  cannot change from trip to closed directly. Now the closed state now
			     *  has more reliability.
			     */
			    secondary_solenoid.current_state = SS_OPEN;
			}
			else
			{
				// was not in a clear open or closed state so default to open.
				#if defined REPORT_SS_DIAGNOSTICS
				//report default to closed state.
				queue_uart_message(pm2m_uart_comm, BREAKER_FAULT_STATE, SECONDARY_SWITCH_DEFAULT_TO_CLOSED, NO_PAYLOAD_1, NO_PAYLOAD_2);
				#endif
				secondary_solenoid.requested_state = SS_CLOSED;
				secondary_solenoid.state_change_requested = TRUE;
			}
			break;
		//default:
			//break;
	}
	return FALSE;
}

ss_status_t secondary_solenoid_get_status(void)
{
	return secondary_solenoid.current_state;
}

void request_open_secondary_solenoid(void)
{
	if((secondary_solenoid.current_state != SS_OPEN))
	{
		secondary_solenoid.requested_state = SS_OPEN;
		secondary_solenoid.state_change_requested = TRUE;
	}
	else if(secondary_solenoid.state_change_requested == TRUE)
	{
		secondary_solenoid.state_change_requested = FALSE;
	}
}

void request_close_secondary_solenoid(void)
{
	if(secondary_solenoid.current_state != SS_CLOSED)
	{
		secondary_solenoid.requested_state = SS_CLOSED;
		secondary_solenoid.state_change_requested = TRUE;
	}
	else if(secondary_solenoid.state_change_requested == TRUE)
	{
		secondary_solenoid.state_change_requested = FALSE;
	}
}

void request_toggle_secondary_solenoid(void)
{
	if( (secondary_solenoid.current_state == SS_CLOSED)  ||
	    (secondary_solenoid.current_state == SS_CLOSING) ||
	    (secondary_solenoid.current_state == SS_STUCK_CLOSED))
	{
		secondary_solenoid.requested_state = SS_OPEN;
		secondary_solenoid.state_change_requested = TRUE;
	}
	else if((secondary_solenoid.current_state == SS_OPEN)  ||
		    (secondary_solenoid.current_state == SS_OPENING) ||
		    (secondary_solenoid.current_state == SS_STUCK_OPEN))
	{
		secondary_solenoid.requested_state = SS_CLOSED;
		secondary_solenoid.state_change_requested = TRUE;
	}
	// Other unknown states already default to requesting for open.
}

void secondary_solenoid_manual_close_task(void)
{
#if defined USE_OLD_MANUAL_CLOSE
	static pswitch_status_t previous_primay_state = SW_UNKNOWN;

		if((get_primary_switch_debounced_state() == SW_CLOSED)
			&&( (previous_primay_state == SW_OPEN) /*|| (previous_primay_state == SW_TRIP)*/)
		   )
		{
			request_close_secondary_solenoid();
		}
		previous_primay_state = get_primary_switch_debounced_state();
#else
		//This may cause chatter if closed was not verified before hand.
		static bool saw_open = FALSE;
		if((get_primary_switch_debounced_state() == SW_CLOSED)
			&&(saw_open == TRUE)
		  )
		{
			//let the device figure out what state it is in first.
			if((secondary_solenoid.current_state != SS_UNKNOWN) && (secondary_solenoid.current_state != SS_CLOSING))
			{
				if(!check_ss_closed())
				{
					request_close_secondary_solenoid(); //will not attempt to close if already closed.
				}
				saw_open = FALSE;
			}
		}
		if(get_primary_switch_debounced_state() == SW_OPEN)
		{
			saw_open = TRUE;
		}

#endif
}

bool check_ss_known(void)
{
	if(get_path_status() != PATH_UNKNOWN)
	{
		return TRUE;
	}
	return FALSE;
}

bool check_ss_closed(void)
{
	if(get_path_status() == PATH_CLOSED)
	{
		return TRUE;
	}
	return FALSE;
}

bool check_ss_open(void)
{
	if(get_path_status() == PATH_OPEN)
	{
		return TRUE;
	}
	return FALSE;
}

void ss_backup_timer_irq_handler (void)
{
	//queue_uart_message(pm2m_uart_comm, BREAKER_FAULT_STATE, SECONDARY_SWITCH_BACKUP_TRIGGERED, NO_PAYLOAD_1, NO_PAYLOAD_2);
    LL_TIM_DisableCounter(SS_BACKUP_TIMER);
    LL_TIM_SetAutoReload(SS_BACKUP_TIMER, SS_DEFAULT_TIMEOUT);
    LL_TIM_SetCounter(SS_BACKUP_TIMER, 0);
    LL_TIM_ClearFlag_UPDATE(SS_BACKUP_TIMER);

	SS_RESET_OPEN;
	SS_RESET_CLOSE;
}



