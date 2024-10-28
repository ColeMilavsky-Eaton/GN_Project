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
 * @defgroup trip trip component
 *
 * @brief The trip component handles the activation of the breaker's trip unit
 *
 * # Overview
 * The trip component performs the following functions:
 *  - Trip routine that synchronize zcd events and trips the breaker with pulses. Trip code is
 *    also being passed over to trip log component to log trip code.
 *  - Unsync trip that doesn't sync zcd events and trips the breaker with pulses. If the breaker
 *    still doesn't trip after a series of unsync pulses, turn on red LED and request trip log
 *    component to log failed trip attempt.
 *  - Instant trip that doesn't sync zcd events and trips the breaker. Trip code is also being
 *    passed over to trip log component to log trip code.
 *
 *
 * @file trip.c
 * @ingroup trip
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "stm32g0xx_it_api.h"
#include "gpio_api.h"
#include "main_internal.h"
#include "trip_.h"
#include "trip_log_api.h"
#include "timer_api.h"
#include "firmware.config"
#include "iwdg_api.h"
#include "led_api.h"

GPIO_TypeDef *zcd_port;
u32 zcd_pin;
extern bool trip_class_1_flag;
extern bool primary_switch_open_flag;

#if NUM_POLES_CONFIGURATION == SINGLE_POLE
void trip_routine(u8* trip_code_data, u8 num_bytes)
{

	return;

    u32 num_trip_attempts = 0;
    u32 num_pulses = 0;
    u16 zero_crossing_stable_low_counter;
    u16 wait_for_stable_low_zero_crossing = 0;
    zero_crossing_detector_t zero_crossing_high_reading_encountered = ZERO_CROSSING_DETECTOR_HIGH_NOT_ENCOUNTERED;
    bool trip_code_written_flag = FALSE;
    pswitch_status_t trip_status;

    /* stop interrupts */
    __disable_irq();

    /* watchdog refresh */
    iwdg_kick_watchdog();

#ifdef ZCD_2
    /* if primary ZCD is zcd 2 */
    if(it_get_primary_zcd() == PRIMARY_ZCD_2)
    {
        zcd_port = ZCD_2_GPIO_Port;
        zcd_pin = ZCD_2_Pin;
    }
    else
    {
        zcd_port = ZCD_1_GPIO_Port;
        zcd_pin = ZCD_1_Pin;
    }
#else // else ZCD_2 not defined
    zcd_port = ZCD_1_GPIO_Port;
    zcd_pin = ZCD_1_Pin;
#endif

#if GF_CONFIGURATION != GF_NOT_ENABLED
    /* Minimize CT load */
    gpio_set_output_pin(GF_OFFSET_GPIO_Port, GF_OFFSET_Pin);
#endif

#warning make sure the version names are correct
#ifndef _2P_GFCI
  #if defined( ADV_GF_ONLY ) || defined( ADV_GF_ONLY_30mA )
    /* on these version of the BR board, the HF sense circuit is de-populated other than the
     * resistor between 3.3 and  hf_sense pin. Setting the pin high allows less current draw
     * CH GFCI and 30mAGF boards dont have the HF circuit, so this step is not needed */
    gpio_set_output_pin(HF_SENSE_GPIO_Port, HF_SENSE_Pin);
  #else

    #if AF_CONFIGURATION != AF_NOT_ENABLED
        /* disable the log amp to maximize the power supply hold-up time, allowing trip code write */
        gpio_reset_output_pin(LOG_ENABLE_GPIO_Port, LOG_ENABLE_Pin);
    #endif
  #endif
#endif

    /* outer loop determines how many attempts and inner loop determines how many pulses
     * per attempt
     */
    for(num_trip_attempts = 0; num_trip_attempts < NUM_SYNCED_TRIP_ATTEMPTS; num_trip_attempts++)
    {
        for(num_pulses = 0; num_pulses < NUM_PULSES_PER_SYNCED_TRIP_ATTEMPT; num_pulses++)
        {
            /* watchdog refresh */
            iwdg_kick_watchdog();

            if( trip_code_written_flag == FALSE)
            /* if trip code not written, this is the first trip pulse, do not wait to sync with ZCD,
             *  turn on trip signals, write trip code, "long" delay, turn off trip signals */
            {
              #if TRIP_CONFIGURATION == TRIP_CONFIG_INDIVIDUAL_FIRING
                #error INDIVIDUAL FIRING NOT COMPLETE.
              #elif TRIP_CONFIGURATION == TRIP_CONFIG_COMBINED_FIRING
                /* turn on trip signal */
                TRIP_1_ON;
                TRIP_2_ON;
              #else
                #error NO TRIP CONFIGURATION SELECTED
              #endif
                /* time to write trip code was tested, and found to vary depending on the position of the next free storage location
                 * trip code write time from 100 to 1200 microseconds */
                trip_log_write_tripcode(trip_code_data, num_bytes);
                trip_code_written_flag = TRUE;

                /* not synched with ZCD, use wide instant trip pulse width */
                timer_delay(TRIP_PULSE_WIDTH_INSTANT);

                /* turn off trip signal */
                TRIP_1_OFF;
                TRIP_2_OFF;
            }
            else
            /* else trip code was written, try to sync remaining pulses with ZCD signal to wait for power supply charge cycle */
            {
                zero_crossing_stable_low_counter = 0;
                zero_crossing_high_reading_encountered = ZERO_CROSSING_DETECTOR_HIGH_NOT_ENCOUNTERED;

                for (wait_for_stable_low_zero_crossing = 0; wait_for_stable_low_zero_crossing < WAIT_FOR_STABLE_LOW_ZERO_CROSSING_DETECTOR_TIMEOUT; wait_for_stable_low_zero_crossing++)
                /* measured no ZCD timeout to be about 31.22 milliseconds */
                {
                   if (gpio_read_input_pin(zcd_port, zcd_pin) == GPIO_PIN_RESET)  /* ZCD low during positive half-cycle */
                   {
                      zero_crossing_stable_low_counter++;
                   }
                   else
                   {
                      zero_crossing_stable_low_counter = 0;
                      zero_crossing_high_reading_encountered = ZERO_CROSSING_DETECTOR_HIGH_ENCOUNTERED;
                   }
                   if (( zero_crossing_stable_low_counter > ZERO_CROSSING_DETECTOR_STABLE_LOW_ACCEPTABLE_COUNT ) &&
                       ( zero_crossing_high_reading_encountered == ZERO_CROSSING_DETECTOR_HIGH_ENCOUNTERED ))
                   {
                      /* if ZCD was high and is now low for about 2.3 milliseconds */
                      break;
                   }
                }

              #if TRIP_CONFIGURATION == TRIP_CONFIG_INDIVIDUAL_FIRING
                #error INDIVIDUAL FIRING NOT COMPLETE.
              #elif TRIP_CONFIGURATION == TRIP_CONFIG_COMBINED_FIRING
                /* turn on trip signal */
                TRIP_1_ON;
                TRIP_2_ON;
              #else
                #error NO TRIP CONFIGURATION SELECTED
              #endif

                if(wait_for_stable_low_zero_crossing < WAIT_FOR_STABLE_LOW_ZERO_CROSSING_DETECTOR_TIMEOUT)
                /* if zero crossing detected */
                {
                    /* use synched pulse width  */
                    timer_delay(TRIP_PULSE_WIDTH_SYNCHED);
                }
                else
                /* else zero crossing detector timeout */
                {
                    /* use wide instant trip pulse width */
                    timer_delay(TRIP_PULSE_WIDTH_INSTANT);
                }

                /* turn off trip signal */
                TRIP_1_OFF;
                TRIP_2_OFF;

                /* a delay between pulses */
                timer_delay(DELAY_BETWEEN_SYNCHED_TRIP_PULSE);
            }

            #if defined POWER_REMAINS_AFTER_TRIP
            trip_status = get_primary_switch_direct_debounced_status();
            if((trip_status == SW_OPEN) || (trip_status == SW_TRIP))
            {
                #if defined RESET_ON_TRIP
                __NVIC_SystemReset();
                #else
                __enable_irq();
                return;
            #endif
            }
			#endif
        }

        /* a delay between trip attempts. */
        timer_delay(DELAY_BETWEEN_SYNCHED_TRIP_ATTEMPTS);
    }

    /* If breaker still has power then proceed to unsynced trip */
    unsynced_trip();

    return;
}

void trip_instant(u8* trip_code_data, u8 num_bytes)
{

	return;

	pswitch_status_t trip_status;
    /* stop interrupts */
    __disable_irq();

    /* watchdog refresh */
    iwdg_kick_watchdog();

#if GF_CONFIGURATION != GF_NOT_ENABLED
    /* Minimize CT load */
    gpio_set_output_pin(GF_OFFSET_GPIO_Port, GF_OFFSET_Pin);
#endif

#ifndef _2P_GFCI
#warning make sure the version names are correct
  #if defined( ADV_GF_ONLY ) || defined( ADV_GF_ONLY_30mA )
    /* on these version of the BR board, the HF sense circuit is de-populated other than the
     * resistor between 3.3 and  hf_sense pin. Setting the pin high allows less current draw
     * CH GFCI and 30mAGF boards dont have the HF circuit, so this step is not needed */
    gpio_set_output_pin(HF_SENSE_GPIO_Port, HF_SENSE_Pin);
  #else
    #if AF_CONFIGURATION != AF_NOT_ENABLED
    /* disable the log amp to maximize the power supply hold-up time, allowing trip code write */
    gpio_reset_output_pin(LOG_ENABLE_GPIO_Port, LOG_ENABLE_Pin);
    #endif
  #endif
#endif

  #if TRIP_CONFIGURATION == TRIP_CONFIG_INDIVIDUAL_FIRING
    #error INDIVIDUAL FIRING NOT COMPLETE.
  #elif TRIP_CONFIGURATION == TRIP_CONFIG_COMBINED_FIRING
    /* turn on trip signal */
    TRIP_1_ON;
    TRIP_2_ON;
  #else
    #error NO TRIP CONFIGURATION SELECTED
  #endif

    /* write trip code */
    trip_log_write_tripcode(trip_code_data, num_bytes);

    /* extend trip pulse width to ensure trip extends into positive half-cycle */
    timer_delay(TRIP_PULSE_WIDTH_INSTANT);

    /* turn off trip signal */
    TRIP_1_OFF;
    TRIP_2_OFF;

	#if defined POWER_REMAINS_AFTER_TRIP
    trip_status = get_primary_switch_direct_debounced_status();
    if((trip_status == SW_OPEN) || (trip_status == SW_TRIP))
    {
		#if defined RESET_ON_TRIP
		__NVIC_SystemReset();
		#else
        __enable_irq();
        return;
		#endif
    }
	#endif

    /* If breaker still has power then proceed to unsynced trip */
    unsynced_trip();

    return;
}

void unsynced_trip(void)
{

	return;

    u8 num_of_trip_attempts = 0;
    u8 num_of_pulses_per_trip_attempt = 0;
    const u8 trip_code_data = FAILED_TRIP_ATTEMPT;
    pswitch_status_t trip_status;

    /* stop interrupts */
    __disable_irq();

    /* outer loop determines how many attempts and inner loop determines how many pulses
     * per attempt
     */
    for(num_of_trip_attempts = 0;
        num_of_trip_attempts < NUM_OF_UNSYNCHED_TRIP_ATTEMPTS;
        num_of_trip_attempts++)
    {
        for(num_of_pulses_per_trip_attempt = 0;
            num_of_pulses_per_trip_attempt < NUM_OF_PULSES_PER_UNSYNCHED_TRIP_ATTEMP;
            num_of_pulses_per_trip_attempt++)
        {
            /* watchdog refresh */
            iwdg_kick_watchdog();

          #if TRIP_CONFIGURATION == TRIP_CONFIG_INDIVIDUAL_FIRING
            #error INDIVIDUAL FIRING NOT COMPLETE.
          #elif TRIP_CONFIGURATION == TRIP_CONFIG_COMBINED_FIRING
            /* turn on trip signal */
            TRIP_1_ON;
            TRIP_2_ON;
          #else
            #error NO TRIP CONFIGURATION SELECTED
          #endif

            /* delay between pulse width */
            timer_delay(TRIP_PULSE_WIDTH_UNSYNCHED);

            /* turn off trip signal */
            TRIP_1_OFF;
            TRIP_2_OFF;

            /* delay between pulses */
            timer_delay(DELAY_BETWEEN_UNSYNCHED_TRIP_PULSE);
        }

		#if defined POWER_REMAINS_AFTER_TRIP
        	trip_status = get_primary_switch_direct_debounced_status();
            if((trip_status == SW_OPEN) || (trip_status == SW_TRIP))
            {
				#if defined RESET_ON_TRIP
        		__NVIC_SystemReset();
				#else
            	__enable_irq();
            	return;
				#endif
            }
		#endif

        /* delay between trip attempts */
        timer_delay(DELAY_BETWEEN_UNSYNCHED_TRIP_ATTEMPTS);
    }

    /* once completed the tripping sequences.write failed attempt */
    trip_log_write_tripcode((u8*)&trip_code_data, 1);

    /* turn on the led */
    LED_RED_ON;
  #if LED_CONFIGURATION == RGB_LED
    LED_BLUE_OFF;
    LED_GREEN_OFF;
  #endif

    /* delay so that breaker won't reset right away. this is to prevent the solenoid
     * from burning up. this delay is roughly 10 minutes.
     * */
    timer_delay(UNSYNC_TRIP_DELAY_BEFORE_WATCHDOG_RESET);

    /* cause watchdog reset */
     while(1);

}
#endif

void trip(u8* trip_code_data, u8 num_bytes)
{
    u8 num_of_trip_attempts = 0;
    u8 failed_trip_attempt_code = FAILED_TRIP_ATTEMPT;
    bool trip_code_written = FALSE;
    pswitch_status_t trip_status;

    // class 2 trip and primary switch shows open: do not trip
    if(trip_class_1_flag == FALSE)
    {
        if(primary_switch_open_flag)
        {
            return;
        }
    }
    // for class 1 trip, go forward

    //disable secondary pulses just in case it is being driven when a trip occurs.
	//LL_TIM_DisableCounter(SS_BACKUP_TIMER);
	SS_RESET_OPEN;
	SS_RESET_CLOSE;

    /* stop interrupts */
    __disable_irq();

    /* watchdog refresh */
    iwdg_kick_watchdog();

#if GF_CONFIGURATION != GF_NOT_ENABLED
    /* Minimize CT load */
    gpio_set_output_pin(GF_OFFSET_GPIO_Port, GF_OFFSET_Pin);

    /* stop watchdog pulse */
    gpio_reset_output_pin(HAL_WATCHDOG_GPIO_Port, HAL_WATCHDOG_Pin);
#endif

#ifndef _2P_GFCI
#warning make sure the version names are correct
  #if defined( ADV_GF_ONLY ) || defined( ADV_GF_ONLY_30mA )
    /* on these version of the BR board, the HF sense circuit is de-populated other than the
     * resistor between 3.3 and  hf_sense pin. Setting the pin high allows less current draw
     * CH GFCI and 30mAGF boards dont have the HF circuit, so this step is not needed */
    gpio_set_output_pin(HF_SENSE_GPIO_Port, HF_SENSE_Pin);
  #else
    #if AF_CONFIGURATION != AF_NOT_ENABLED
    /* disable the log amp to maximize the power supply hold-up time, allowing trip code write */
    gpio_reset_output_pin(LOG_ENABLE_GPIO_Port, LOG_ENABLE_Pin);
    #endif
  #endif
#endif


    for(num_of_trip_attempts = 0;
        num_of_trip_attempts < NUM_OF_TRIP_ATTEMPTS;
        num_of_trip_attempts++)
    {

    	/* start watchdog pulse */
    	gpio_set_output_pin(HAL_WATCHDOG_GPIO_Port, HAL_WATCHDOG_Pin);

#if TRIP_CONFIGURATION == TRIP_CONFIG_INDIVIDUAL_FIRING
  #error INDIVIDUAL FIRING NOT COMPLETE.
#elif TRIP_CONFIGURATION == TRIP_CONFIG_COMBINED_FIRING
        /* turn on trip signal */
        TRIP_1_ON;
        TRIP_2_ON;
#else
  #error NO TRIP CONFIGURATION SELECTED
#endif

        /* write trip code */
        if(trip_code_written == FALSE)
        {
            trip_log_write_tripcode(trip_code_data, num_bytes);
            trip_code_written = TRUE;
        }

        /* 2p pulse with should be almost 4 half cycles. */
        timer_delay(TRIP_PULSE_WIDTH);

        /* turn off trip signal */
        TRIP_1_OFF;
        TRIP_2_OFF;

        #if defined POWER_REMAINS_AFTER_TRIP
            // some sample need more time for Hall sensor to pick up the correct state
            // add a 20ms delay here
            timer_delay(TRIP_PULSE_WIDTH * 2);
            trip_status = get_primary_switch_direct_debounced_status();
            if((trip_status == SW_OPEN) || (trip_status == SW_TRIP))
            {
                // If the last three codes are the same and are not normal
                // means HW integrity is in doubt. will not back to normal operation
                // Stay here to open the circuit in case the user closes the circuit

            	//TODO I think this may be able to be forced to happen if you don't clear the fault when you keep resetting the breaker.
            	//We need to investigate this further!
                if(trip_log_check_latest_three_codes())
                {
                    num_of_trip_attempts = 0;
                    while(TRUE)
                    {

                    	/* stop watchdog pulse */
                    	gpio_reset_output_pin(HAL_WATCHDOG_GPIO_Port, HAL_WATCHDOG_Pin);
                        trip_status = get_primary_switch_direct_debounced_status();
                        if((trip_status == SW_CLOSED) || (trip_status == SW_FAILED))
                        {
                            num_of_trip_attempts ++;
                            TRIP_1_ON;
                            TRIP_2_ON;
                            timer_delay(TRIP_PULSE_WIDTH);
                            /* turn off trip signal */
                            TRIP_1_OFF;
                            TRIP_2_OFF;
                        }
                        /* delay between trip attempts */
                        timer_delay(DELAY_BETWEEN_TRIP_ATTEMPTS/2);
                        /* start watchdog pulse */
                        gpio_set_output_pin(HAL_WATCHDOG_GPIO_Port, HAL_WATCHDOG_Pin);
                        timer_delay(DELAY_BETWEEN_TRIP_ATTEMPTS/2);
                        if(num_of_trip_attempts == 3)
                        {
                            // Long cool down time
                            timer_delay(UNSYNC_TRIP_DELAY_BEFORE_WATCHDOG_RESET);
                            num_of_trip_attempts = 0;
                        }
                    }
                }
                #if defined RESET_ON_TRIP
                if(!a_m2m_message_is_queued())
                {
                	__NVIC_SystemReset();
                }
                else
                {
                	delayed_reset_for_m2m_queue();
                }
                #else
                __enable_irq();
                return;
                #endif
            }
        #endif

        /* stop watchdog pulse */
        gpio_reset_output_pin(HAL_WATCHDOG_GPIO_Port, HAL_WATCHDOG_Pin);

        /* delay between trip attempts */
        timer_delay(DELAY_BETWEEN_TRIP_ATTEMPTS);
    }

    /* once completed the tripping sequences.write failed attempt */
    trip_log_write_tripcode((u8*)&failed_trip_attempt_code, 1);

    led_reset_key_status();
	led_set_key_status(KEY_TAKEN_BY_FAILED_TO_TRIP);
	//This should prevent all but the identify me from operating.
	led_turn_on(LED_RED);

    /* turn on the led */
    //LED_RED_ON;
  //#if LED_CONFIGURATION == RGB_LED
    //LED_BLUE_OFF;
    //LED_GREEN_OFF;
  //#endif

    //JF TODO can the primaries be burn out by constantly resetting into a fault?
    //I can reset in a few seconds and if it takes 10 minutes after a few tries does this damage the solenoid?

    /* delay so that breaker won't reset right away. this is to prevent the solenoid
     * from burning up. this delay is roughly 10 minutes.
     * */

    //timer_delay(UNSYNC_TRIP_DELAY_BEFORE_WATCHDOG_RESET);
    delayed_failed_to_trip_reset_for_m2m_queue();

    /* cause watchdog reset */
     while(1);
}

void delayed_failed_to_trip_reset_for_m2m_queue(void)
{
	#define LOOP_TIMING_SPACER_DF 11 //This number represents the time the loop takes to execute excluding the delays.
	//This allows for the set delay count to approximate the time for a normal timer_delay function with each count subtracting the delays and loop time.
	//The time will however be off by at most a factor of (DELAY_TO_ENSURE_MESSAGE_RECEIPT + LOOP_TIMING_SPACER_DF)
	s32 delay_counts = UNSYNC_TRIP_DELAY_BEFORE_WATCHDOG_RESET;// this should approximate to 10 min
	while(delay_counts > 0)
	{
		/* start watchdog pulse */
		gpio_set_output_pin(HAL_WATCHDOG_GPIO_Port, HAL_WATCHDOG_Pin);
		timer_delay(DELAY_TO_ENSURE_MESSAGE_RECEIPT/2);
		/* stop watchdog pulse */
		gpio_reset_output_pin(HAL_WATCHDOG_GPIO_Port, HAL_WATCHDOG_Pin);
		timer_delay(DELAY_TO_ENSURE_MESSAGE_RECEIPT/2);
		delay_counts = delay_counts - DELAY_TO_ENSURE_MESSAGE_RECEIPT - LOOP_TIMING_SPACER_DF;
		read_uart_user_command();
	}
	__NVIC_SystemReset();
}

void delayed_reset_for_m2m_queue(void)
{
	//#define DELAY_FOR_QUEUE_TIME  (916000UL) //should be a bit less than 2s
	#define DELAY_FOR_QUEUE_TIME  (2800000UL) //increased to over 3 seconds to allow for HRGF_COLD_START
	#define LOOP_TIMING_SPACER_DR 260
	s32 delay_counts = DELAY_FOR_QUEUE_TIME;//this will allow for a max of 2 additional pulses before resetting.
	u32 uart_spacing_counter = 0;
	pswitch_status_t trip_status;
	bool watchdog_on = 0;

	while(delay_counts > 0)
	{
		//if we do not have any messages queued go ahead and reset.
		if(!a_m2m_message_is_queued())
		{
			break;
		}

		iwdg_kick_watchdog();
		delay_counts -= LOOP_TIMING_SPACER_DR;
		uart_spacing_counter += LOOP_TIMING_SPACER_DR;
		trip_status = get_primary_switch_direct_debounced_status();
		if((trip_status == SW_CLOSED) || (trip_status == SW_FAILED))
		{
			//If this occurs both times it is unlikely the UART messages will clear.
			//however we are limiting this as it is meant to protect the solenoid and protection functions.
			TRIP_1_ON;
			TRIP_2_ON;
			timer_delay(TRIP_PULSE_WIDTH);
			/* turn off trip signal */
			TRIP_1_OFF;
			TRIP_2_OFF;

			/* delay between trip attempts */
			timer_delay(DELAY_BETWEEN_TRIP_ATTEMPTS);
			// we could swap this delay for something that allows the UART to run however I don't want to risk it on an unlikely scenario.

			//delay_counts = delay_counts - TRIP_PULSE_WIDTH - DELAY_BETWEEN_TRIP_ATTEMPTS;
			delay_counts = delay_counts - (DELAY_FOR_QUEUE_TIME/3); //we want to allow this a maximum of 2 times especially with the 1s delay.
			uart_spacing_counter = DELAY_TO_ENSURE_MESSAGE_RECEIPT;
		}
		if(uart_spacing_counter >= DELAY_TO_ENSURE_MESSAGE_RECEIPT)
		{
			if(watchdog_on)
			{
				watchdog_on = FALSE;
				/* start watchdog pulse */
				gpio_set_output_pin(HAL_WATCHDOG_GPIO_Port, HAL_WATCHDOG_Pin);
			}
			else
			{
				watchdog_on = TRUE;
				/* stop watchdog pulse */
				gpio_reset_output_pin(HAL_WATCHDOG_GPIO_Port, HAL_WATCHDOG_Pin);
			}
			read_uart_user_command();
			uart_spacing_counter = 0;
		}
	}
	__NVIC_SystemReset();
}


