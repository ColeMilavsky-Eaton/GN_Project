/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 ***************************************************************************************************
 *  Written by:         Hank Sun
 *                      Eaton Electrical
 *                      1000 Cherrington Parkway
 *                      Pittsburgh, PA  15108-4312
 *                      (412) 893-3300
 *//**
 * @defgroup trip_log trip_log component
 *
 * @brief This component consists of four main features, trip log led xfer, blink latest log code,
 *        output trip log via spi and write trip code.
 *
 * @image rtf tllt state diagram.png "T.L.L.T (RED BLINK) state machine diagram"
 * @image rtf tllt process flow.png "T.L.L.T (RED BLINK) process flow diagram"
 *
 * # Trip Log Led Xfer
 * Trip log led xfer blinks the log code via a led. After being requested by the system, the
 * component starts to prepare and looks for the next data to be sent out. The blinking pattern
 * uses a three token protocol as described below.
 *
 * Each bit is represented by three tokens. token 2 is always lit, token 1 is lit IF the bit
 * representing is 1, and token 0 is always not lit. Every timer interrupt will update the led
 * depending on what token the component is currently blinking.
 *
 * Each byte has 8 bits which means each byte will require at least 24 interrupt to send out. In
 * order for the receiver to clearly distinguish between bytes, after a byte has sent, there will
 * be 8 interrupts that just constantly sets the led to low. which means each byte will take 32
 * interrupts to send out.
 *
 * For example a 1 bit is encoded as|H|H|L   , and a 0 bit is encoded as |H|L|L
 *
 * which means 0x56 is represented as
 * |H|L|L|H|H|L|H|L|L|H|H|L|H|L|L|H|H|L|H|H|L|H|L|L |L|L|L|L|L|L|L|L
 *  0     1     0     1     0     1     1     0
 *
 *
 * Trip log led xfer sends out four piece of data in the following order.
 * 1. UID
 * 2. Breaker info which includes the firmware checksum, overload and shunt calibration.
 * 3. Trip log.
 * 4. LRC. (a form of checksum that confirms the validity of the data being transfered)
 *
 *
 * # Blink Latest Log Code
 * Blink latest log code will blink the latest log code that is stored in trip log area. Depending
 * on the log code, one to six blinks will blink thirty times via the led.
 *
 * There are two ways of blinking the latest log code.
 * 1. If the user holds down the ptt button when turning the breaker on and release ptt button
 *    within two seconds
 * 2. If the breaker just tripped.
 *
 *
 * #Output Status Log Via SPI
 * Output status log via spi uses spi communication to transfer out trip log data. This feature is
 * activated when a pin on the debug pad is being held high for a certain amount of time. Once
 * activated, a sequence of data will be transfered. Breaker configuration and trip logs will be
 * sent out.
 *
 *
 *
 * @file trip_log_led_xfer.c
 * @ingroup trip_log_led_xfer
 *
 *
 *//*
 *
 **************************************************************************************************/

#include "trip_log_.h"
#include "types.h"
#include "led_api.h"
#include "main_internal.h"
#include "task_manager_api.h"
#include "flash_api.h"
#include "main_api.h"
#if BOARD_CONFIGURATION != BOARD_SB2  //Smart Breaker boards do not contain spi interface
	#include "spi_api.h"
#endif
#include "self_test_api.h"
#include "self_test_.h"
#include "firmware.config"
#include "btn_api.h"
#include "shunt_cal_api.h"
#include "load_current_api.h"
#include "iwdg_api.h"
#include "load_voltage_api.h"


trip_log_led_xfer_t trip_log_led_xfer;
latest_logcode_led_display_t  latest_logcode_led_display;
u16 output_status_log_counter;
u8 quickblink_enable_flag = 0 ;    // enable flag for quickblink function
extern u32 __build_checksum;
//extern u64 __version;

extern M2M_UART_COMMN *pm2m_uart_comm;

void trip_log_zcd_callback(u32 it)
{
    output_status_log_counter = 0;

    return;
}

void trip_log_trip_log_led_xfer_init(void)
{
    trip_log_led_xfer.counter = 0;
    trip_log_led_xfer.sub_token_counter = 0;
    trip_log_led_xfer.bit_counter = 0;
    trip_log_led_xfer.progress_flag = TRIP_LOG_LED_XFER_NOT_IN_PROGRESS;
    trip_log_led_xfer.next_status_log_value = 0x00;
    trip_log_led_xfer.start_counter = 0;
    trip_log_led_xfer.LRC = 0;

    task_t t;

    /* prepare data is called once per cycle */
    t.task_p = trip_log_prepare_trip_log_led_xfer_data;
    t.it_bits = TSK_MAN_IT_0;
    task_man_add_task_to_schedule(t);

    /* assign task */
    t.task_p = trip_log_update_trip_log_led_xfer_led;
    t.it_bits = TSK_MAN_IT_ALL;

    task_man_add_task_to_schedule(t);


    /** register zcd callback */
    t.task_p = trip_log_zcd_callback;
    t.it_bits = TSK_MAN_ZCD;

    task_man_add_task_to_schedule(t);

    return;
}

void trip_log_blink_latest_logcode_init(void)
{
    task_t t;

    /* assign task */
    t.task_p = trip_log_update_blink_latest_logcode_led;
    t.it_bits = TSK_MAN_IT_1 | TSK_MAN_IT_17;

    task_man_add_task_to_schedule(t);
}

#if BOARD_CONFIGURATION != BOARD_SB2 //Smart Breaker boards do not contain spi interface
void trip_log_output_trip_log_via_spi_init(void)
{
    /* task type to assign tasks */
    task_t t;

    /* assign task */
    t.task_p = trip_log_check_for_output_trip_log_via_spi;
    t.it_bits = TSK_MAN_IT_ALL;

    task_man_add_task_to_schedule(t);

    output_status_log_counter = 0;

    return;
}
#endif

void trip_log_update_trip_log_led_xfer_led(u32 it)
{
    if(trip_log_led_xfer.progress_flag == TRIP_LOG_LED_XFER_IN_PROGRESS)
    {
        //first sub-token. Always lit
        if ( trip_log_led_xfer.sub_token_counter == 2 )
        {
             trip_log_led_xfer.sub_token_counter = 1;
             led_request(LED_RED, LED_TURN_ON);
        }
        //second sub-token. lit if sending "1". else not lit.
        else if ( trip_log_led_xfer.sub_token_counter == 1 )
        {
            trip_log_led_xfer.sub_token_counter = 0;
            if ( trip_log_led_xfer.next_status_log_value & 0x80 )
            {
                 led_request(LED_RED, LED_TURN_ON);
            }
            else
            {
                 led_request(LED_RED, LED_TURN_OFF);
            }
            //prepare to send next bit, next time around:
            trip_log_led_xfer.next_status_log_value = trip_log_led_xfer.next_status_log_value & 0x7F;   //clear upper bit to avoid overflow.
            trip_log_led_xfer.next_status_log_value = trip_log_led_xfer.next_status_log_value << 1 ;   //shift to next bit to be sent
        }
        //3rd (last) subtoken. always not lit
        else if ( trip_log_led_xfer.sub_token_counter == 0 )
        {
            trip_log_led_xfer.sub_token_counter = 2;
            led_request(LED_RED, LED_TURN_OFF);

          trip_log_led_xfer.bit_counter++;
            if (trip_log_led_xfer.bit_counter >=8){
                trip_log_led_xfer.sub_token_counter = 0;  //keep at this value once we reach 8 bits.
          }
        }
        //if higher than 2, allows for a timing offset to delay start.
        else
        {
            led_request(LED_RED, LED_TURN_OFF);
            trip_log_led_xfer.sub_token_counter --;
        }
    }

    return;
}

void trip_log_prepare_trip_log_led_xfer_data(u32 it)
{
    u8* data_eeprom_pointer;
    static u8 breaker_info_buffer[10] = {0};
    u8 breaker_handle_rating = 0;
    u8 breaker_cal_factor = 0;

    if((led_get_key_status() == KEY_FREE) && (trip_log_led_xfer.progress_flag == TRIP_LOG_LED_XFER_IN_PRE_PROGRESS))
    {
        led_set_key_status(KEY_TAKEN_BY_TLLT);
    }
    //manage the delay before start of transmission:
    if (trip_log_led_xfer.progress_flag == TRIP_LOG_LED_XFER_IN_PRE_PROGRESS && (led_get_key_status() == KEY_TAKEN_BY_TLLT))
    {
        trip_log_led_xfer.start_counter++;
        if(trip_log_led_xfer.start_counter >= TRIP_LOG_LED_XFER_START_DELAY)
        {
            /* Historically in 8 bit, there are 10 bytes in the memory that stores the breaker
             * info ( 16B checksum repeated 3 times + handle rating and complement + calibration
             * factor and complement, total 10 bytes). To match the 8 bit breakers format, we store
             * similar information in the 10 bytes buffer. First 4 bytes will be the 32B CRC,
             * pad the next two bytes with 0, followed by handle rating and complement and
             * calibration factor and complement.
             */
            for(u8 counter = 0; counter < 4; counter++)
            {
                /* crc information */
                breaker_info_buffer[counter] = (u8)(__build_checksum >> (3-counter)* 8);
            }

            breaker_handle_rating = (u8)shunt_cal_get_breaker_rating();
            breaker_cal_factor = shunt_cal_get_breaker_cal_factor();

            /* pad the non used with 0 */
            breaker_info_buffer[4] = 0;
            breaker_info_buffer[5] = 0;

            /* include the rating and shunt cal factor information */
            breaker_info_buffer[6] = breaker_handle_rating;
            breaker_info_buffer[7] = ~breaker_handle_rating;
            breaker_info_buffer[8] = breaker_cal_factor;
            breaker_info_buffer[9] = ~breaker_cal_factor;

            trip_log_led_xfer.progress_flag = TRIP_LOG_LED_XFER_IN_PROGRESS;
            trip_log_led_xfer.message_send_state = TRIP_LOG_LED_XFER_UID_SEND_STATE;
        }
    }

    //choose the next value to send:
    if ((trip_log_led_xfer.progress_flag == TRIP_LOG_LED_XFER_IN_PROGRESS) && (led_get_key_status() == KEY_TAKEN_BY_TLLT))
    {
        //sending the UID:
        if(trip_log_led_xfer.message_send_state == TRIP_LOG_LED_XFER_UID_SEND_STATE)
        {
            /* due to little-endian. start from the end of the address and come back.  */
            data_eeprom_pointer = (u8*)(UID_BEGIN) + UID_LENGTH - trip_log_led_xfer.counter -1;
            trip_log_led_xfer.next_status_log_value = *data_eeprom_pointer;
            trip_log_led_xfer.LRC = trip_log_led_xfer.LRC ^ trip_log_led_xfer.next_status_log_value;
            trip_log_led_xfer.counter++;

            if ( trip_log_led_xfer.counter >= (UID_LENGTH))
            {
                trip_log_led_xfer.message_send_state = TRIP_LOG_LED_XFER_BREAKER_INFO_SEND_STATE;
                trip_log_led_xfer.counter = 0;
            }
        }
        //sending the breaker info that is stored in the flash:
        else if(trip_log_led_xfer.message_send_state == TRIP_LOG_LED_XFER_BREAKER_INFO_SEND_STATE)
        {
            data_eeprom_pointer = &breaker_info_buffer[0] + trip_log_led_xfer.counter;
            trip_log_led_xfer.next_status_log_value = *data_eeprom_pointer;
            trip_log_led_xfer.LRC = trip_log_led_xfer.LRC ^ trip_log_led_xfer.next_status_log_value;
            trip_log_led_xfer.counter++;

            if ( trip_log_led_xfer.counter >= FLASH_BREAKER_CONFIG_BYTES )
            {
                    trip_log_led_xfer.message_send_state = TRIP_LOG_LED_XFER_TRIPCODE_SEND_STATE;
                    trip_log_led_xfer.counter = 0;
            }
        }
        //sending the trip code:
        else if(trip_log_led_xfer.message_send_state == TRIP_LOG_LED_XFER_TRIPCODE_SEND_STATE)
        {
            data_eeprom_pointer = (u8*)(TRIP_LOG_FIRST_TRIPCODE_ADDRESS) + trip_log_led_xfer.counter;
            trip_log_led_xfer.next_status_log_value = *data_eeprom_pointer;
            trip_log_led_xfer.LRC = trip_log_led_xfer.LRC ^ trip_log_led_xfer.next_status_log_value;
            /* since each trip code is spaced 8 bytes apart */
            trip_log_led_xfer.counter+=8;

            if ( trip_log_led_xfer.counter >= (TRIP_LOG_MAX_NUM_TRIPCODE) * 8 || (*(data_eeprom_pointer+=8) == 0xFF) )
            {
                    trip_log_led_xfer.message_send_state = TRIP_LOG_LED_XFER_LRC_SEND_STATE;
                    trip_log_led_xfer.counter = 0;
            }
        }
        //sending LRC byte:
        else if(trip_log_led_xfer.message_send_state == TRIP_LOG_LED_XFER_LRC_SEND_STATE)
        {
            trip_log_led_xfer.next_status_log_value = trip_log_led_xfer.LRC;
            trip_log_led_xfer.counter++;

            if ( trip_log_led_xfer.counter >= 2)
            {
                trip_log_led_xfer.progress_flag = TRIP_LOG_LED_XFER_NOT_IN_PROGRESS;
                led_reset_key_status();
            }
        }

        trip_log_led_xfer.sub_token_counter = 2 + 1;   //Shifted by +1 for long_slot
        trip_log_led_xfer.bit_counter = 0;
    }

    return;
}

inline trip_log_led_xfer_progress_flag_t trip_log_get_led_xfer_progress(void)
{
    return trip_log_led_xfer.progress_flag;
}

inline void trip_log_request_trip_log_led_xfer_initiate(void)
{
    trip_log_led_xfer.progress_flag = TRIP_LOG_LED_XFER_IN_PRE_PROGRESS;

    return;
}

status_t trip_log_write_tripcode(u8* trip_code_data, u8 num_bytes)
{
    status_t status = STATUS_UNKNOWN;
    u64 tripcode_copy = 0;
    u64* address_pointer = (u64*)trip_log_get_next_available_location();
    u8 trip_code = 0;

    if(trip_code_data == NULL)
    {
        return STATUS_NULL_P;
    }

    trip_code = *trip_code_data;

#if defined FACTORY_RESET_BOTH
    //at this time do not save a factory reset "fault"
    //we want to trip the primary but saving this would not be ideal for the non looping buffer.
    //Additionally
    if(trip_code == SB2_FACTORY_RESET)
    {
        return STATUS_OK;
    }
#endif

    queue_uart_message(pm2m_uart_comm, BREAKER_FAULT_STATE, trip_code, NO_PAYLOAD_2, NO_PAYLOAD_2);

    /* if too much bytes to write or no more space available. */
    if((num_bytes > TRIP_LOG_TRIP_CODE_DATA_SIZE) || (address_pointer == NULL))
    {
        return STATUS_FAIL;
    }

    /* check if there's enough space. */
    if((u32)num_bytes > ((u32)TRIP_LOG_LAST_TRIPCODE_ADDRESS - (u32)address_pointer + (u32)FLASH_STANDARD_WRITE_BYTES))
    {
        return STATUS_FAIL;
    }

    /* write maximum of 8 bytes each time */
    while(num_bytes > 0)
    {
        /* clear variable */
        tripcode_copy = 0;

        /* Copy data to a u64 variable */
        for(u8 i = 0; i < MIN(FLASH_STANDARD_WRITE_BYTES,num_bytes); i++)
        {
            tripcode_copy += ( (u64)(*trip_code_data) << (i*8) );
            trip_code_data++;
        }

        /* write data */
        status = flash_write64(&tripcode_copy, (u64*)address_pointer);

        if(status != STATUS_OK)
        {
            return status;
        }

        /* move pointer to the next location */
        address_pointer++;

        num_bytes -= MIN(FLASH_STANDARD_WRITE_BYTES,num_bytes);
    }


    return status;
}

void trip_log_log_normal_startup_at_startup(void)
{
    u8 trip_code_to_write = NORMAL_STARTUP;

    //I see no reason why this looks for the latest log code here.
    //u8 latest_trip_code = trip_log_find_latest_log_code();
    u8 latest_trip_code;// = trip_log_find_latest_log_code();
    /* get the next available space */
    u8* data_p = (u8*)trip_log_get_next_available_location();

    /* If no tripcode is in the memory*/
    if(data_p == (u8*)TRIP_LOG_FIRST_TRIPCODE_ADDRESS)
    {
        trip_log_write_tripcode(&trip_code_to_write, 1);

        latest_trip_code = NORMAL_STARTUP;
    }

    /* when trip code area is neither full nor empty */
    else if((data_p <= (u8*)TRIP_LOG_LAST_TRIPCODE_ADDRESS) &&
            (data_p > (u8*)TRIP_LOG_FIRST_TRIPCODE_ADDRESS)
           )
    {
        data_p-=8;
        latest_trip_code = *data_p;
        //do not write normal startup of starting up from a trip.
        if(*data_p != NORMAL_STARTUP
			#if defined SAVE_ALL_CLEAR
        		//If you are saving all clear you need to make sure this isn't clearing while
        		//a fault is still present.
        		&& (get_primary_switch_debounced_state() != SW_TRIP)
			#endif
		  )
        {
            trip_log_write_tripcode(&trip_code_to_write, 1);
        }
    }

    /* when trip code area is full, don't need to draw anchor*/
    // warning when the area is full. writing normal startup will fail.
    else
    {
        latest_trip_code =*(u8*)TRIP_LOG_LAST_TRIPCODE_ADDRESS;

        main_set_end_of_factory_anchor_checked();
        main_set_end_of_factory_anchored();
    }

    /* if the breaker just tripped, breaker will blink the latest trip code */
    trip_log_map_logcode_value_to_logcode_category( latest_trip_code );

    return;
}

void trip_log_log_fault_cleared(void)
{
    u8 trip_code_to_write = NO_TRIP;
    /* get the next available space */
    u8* data_p = (u8*)trip_log_get_next_available_location();

    /* Should not be the case that no trip code is in memory
     * but just in case.*/
    if(data_p == (u8*)TRIP_LOG_FIRST_TRIPCODE_ADDRESS)
    {
        trip_log_write_tripcode(&trip_code_to_write, 1);
    }

    /* when trip code area is neither full nor empty */
    else if((data_p <= (u8*)TRIP_LOG_LAST_TRIPCODE_ADDRESS) &&
            (data_p > (u8*)TRIP_LOG_FIRST_TRIPCODE_ADDRESS)
           )
    {
        data_p-=8;
        //do not write normal startup of starting up from a trip.
        if((*data_p != NORMAL_STARTUP)
        	&& (*data_p != NO_TRIP)
		    )
        {
            trip_log_write_tripcode(&trip_code_to_write, 1);
        }
        else
        {
        	//allows messages to be sent even if last saved value should already have cleared it.
        	queue_uart_message(pm2m_uart_comm, BREAKER_FAULT_STATE, NO_TRIP, NO_PAYLOAD_1, NO_PAYLOAD_2);
        }
    }
    return;
}

void trip_log_log_HAL_fault(void)
{
	u8 trip_code_to_write = PRIMARY_HAL_TRIP_OR_OFF;
	u8* data_p = (u8*)trip_log_get_next_available_location();

	if((data_p <= (u8*)TRIP_LOG_LAST_TRIPCODE_ADDRESS) &&
		(data_p > (u8*)TRIP_LOG_FIRST_TRIPCODE_ADDRESS)
	   )
	{
		data_p-=8;
		//do not write normal startup of starting up from a trip.
		if(*data_p != PRIMARY_HAL_TRIP_OR_OFF)
		{
		    // add another check to make sure if a fault trip code was written last time, do not write HAL
		    if(*data_p < SB2_DIAGNOSTIC_BEGIN)
		    {
		        trip_log_write_tripcode(&trip_code_to_write, 1);
		    }
		}
		else
		{
			//allows messages to be sent even if last saved value should already have cleared it.
			//queue_uart_message(pm2m_uart_comm, BREAKER_FAULT_STATE, PRIMARY_HAL_TRIP_OR_OFF, NO_PAYLOAD_2, NO_PAYLOAD_2);
		}
	}
}

void trip_log_map_logcode_value_to_logcode_category( u8 status_log_entry )
{
#if LED_CONFIGURATION == RGB_LED

    if (( status_log_entry == GENERAL_ARC_DETECTION_LOW_CURRENT_TRIP ) ||
           ( status_log_entry == DIMMER_ARC_DETECTION_LOW_CURRENT_TRIP ))
    {
      latest_logcode_led_display.output_code = 2;
      latest_logcode_led_display.led_display_color = LED_MAGENTA;
      latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
    }
   else if (( status_log_entry == GENERAL_ARC_DETECTION_HIGH_CURRENT_TRIP ) ||
            ( status_log_entry == DIMMER_ARC_DETECTION_HIGH_CURRENT_TRIP ))
   {
      latest_logcode_led_display.output_code = 3;
      latest_logcode_led_display.led_display_color = LED_MAGENTA;
      latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
   }
   else if( status_log_entry == PARALLEL_ARC_FAULT_TRIP )
   {
      latest_logcode_led_display.output_code = 4;
      latest_logcode_led_display.led_display_color = LED_MAGENTA;
      latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
   }
   else if (status_log_entry == OVERLOAD_TRIP)
   {
      latest_logcode_led_display.output_code = 4;
      latest_logcode_led_display.led_display_color = LED_BLUE;
      latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
   }
   else if (status_log_entry == OVERVOLTAGE_TRIP)
   {
      latest_logcode_led_display.output_code = 3;
      latest_logcode_led_display.led_display_color = LED_BLUE;
      latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
   }
   else if (status_log_entry == GROUND_FAULT_OVERCURRENT_TRIP)
   {
      latest_logcode_led_display.output_code = 4;
      latest_logcode_led_display.led_display_color = LED_WHITE;
      latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
   }
   else if(status_log_entry == HRGF_COLDSTART_TRIP || status_log_entry == HRGF_RUNNING_TRIP)

   {
      latest_logcode_led_display.output_code =  3;
      latest_logcode_led_display.led_display_color = LED_WHITE;
      latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
   }
   else if ((status_log_entry == GN_TRIP) || (status_log_entry == HIGH_GN_TRIP))
   {
       latest_logcode_led_display.output_code =  2;
       latest_logcode_led_display.led_display_color = LED_WHITE;
       latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
   }
   else if (status_log_entry == SHORT_DELAY_FAULT_OVERCURRENT_TRIP)
   {
       latest_logcode_led_display.output_code =  2;
       latest_logcode_led_display.led_display_color = LED_BLUE;
       latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
   }
   else if(status_log_entry == PRIMARY_HAL_TRIP_OR_OFF)
   {
       latest_logcode_led_display.output_code = 5;
       latest_logcode_led_display.led_display_color = LED_CYAN;
       latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
   }
   else if(status_log_entry == SELF_TEST_SUCCESSFUL_TRIP)
   {
       latest_logcode_led_display.output_code = 3;
       latest_logcode_led_display.led_display_color = LED_CYAN; //determine actual color.
       latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
   }
   else if ((( status_log_entry >= CONTINUOUS_HW_FAULT_CODES_BEGIN ) && ( status_log_entry <= CONTINUOUS_HW_FAULT_CODES_END )) ||
           (( status_log_entry >= USER_INITIATED_HW_FAULT_CODES_BEGIN ) && ( (u16)status_log_entry <= USER_INITIATED_HW_FAULT_CODES_END )))
   {
       if( ((status_log_entry >= FAILED_STARTUP_RAM_TEST) && (status_log_entry <= FAILED_STARTUP_PROCESSOR_TEST)) ||
           ((status_log_entry >= FAILED_CONTINUOUS_RAM_TEST_TRIP) && (status_log_entry <= NONHANDLED_INTERRUPT_TRIP)))
       {
           latest_logcode_led_display.output_code = 3;
           latest_logcode_led_display.led_display_color = LED_YELLOW;
           latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
       }
       else if(status_log_entry == DATA_OVERRUN_TRIP || status_log_entry == INVALID_DATA_SEQUENCE_TRIP)
       {
           latest_logcode_led_display.output_code = 2;
           latest_logcode_led_display.led_display_color = LED_YELLOW;
           latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
       }
       else if(((status_log_entry >= LINE_CURRENT_BIAS_ERROR_TRIP) && (status_log_entry <= LOG_HF_MIN_DETECTOR_STUCK_ERROR_TRIP)) ||
               ((status_log_entry >= FAILED_SELF_CHECK_GF_INPUT) && (status_log_entry <= FAILED_SELF_CHECK_CT_DIRECTION)))
       {
           latest_logcode_led_display.output_code = 4;
           latest_logcode_led_display.led_display_color = LED_YELLOW;
           latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
       }
       else if(status_log_entry == FAILED_TRIP_ATTEMPT)
       {
           latest_logcode_led_display.output_code = 5;
           latest_logcode_led_display.led_display_color = LED_YELLOW;
           latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
       }
       else if(status_log_entry == HF_SENSE_FAULT_TRIP)
       {
           latest_logcode_led_display.output_code = 5;
           latest_logcode_led_display.led_display_color = LED_MAGENTA;
           latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
       }
       else if(status_log_entry == INCORRECT_INTERRUPT_COUNT_TRIP)
       {
           latest_logcode_led_display.output_code = 2;
           latest_logcode_led_display.led_display_color = LED_CYAN;
           latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
       }
       else if(status_log_entry == OVER_TEMP_TRIP)
       {
           latest_logcode_led_display.output_code = 3;
           latest_logcode_led_display.led_display_color = LED_YELLOW;
           latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
       }
       else if(status_log_entry == POWER_FREQUENCY_OUT_OF_RANGE_TRIP)
       {
           latest_logcode_led_display.output_code = 4;
           latest_logcode_led_display.led_display_color = LED_YELLOW;
           latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
       }
   }
   else
   {
        latest_logcode_led_display.output_code = 0;
        latest_logcode_led_display.led_display_color = LED_UNASSIGNED_COLOR;
        #if !defined SAVE_ALL_CLEAR
        latest_logcode_led_display.count.times_to_blink = 1;
        //value of 1 will cause fault clear message to be sent on startup if no fault preveously present..
        #else
        latest_logcode_led_display.count.times_to_blink = 0;
        #endif
    }
#elif LED_CONFIGURATION == SINGLE_RED_LED

    if (( status_log_entry == GENERAL_ARC_DETECTION_LOW_CURRENT_TRIP ) ||
        ( status_log_entry == DIMMER_ARC_DETECTION_LOW_CURRENT_TRIP ))
    {
       latest_logcode_led_display.output_code = 1;
       latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
    }
    else if (( status_log_entry == PARALLEL_ARC_FAULT_TRIP ) ||
             ( status_log_entry == GENERAL_ARC_DETECTION_HIGH_CURRENT_TRIP ) ||
             ( status_log_entry == DIMMER_ARC_DETECTION_HIGH_CURRENT_TRIP ))
    {
       latest_logcode_led_display.output_code = 2;
       latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
    }
    else if (status_log_entry == OVERLOAD_TRIP)
    {
       latest_logcode_led_display.output_code = 3;
       latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
    }
    else if (status_log_entry == OVERVOLTAGE_TRIP)
    {
       latest_logcode_led_display.output_code = 4;
       latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
    }
    else if ((status_log_entry == GROUND_FAULT_OVERCURRENT_TRIP) ||
             (status_log_entry == HRGF_COLDSTART_TRIP) ||
             (status_log_entry == HRGF_RUNNING_TRIP) ||
             (status_log_entry == GN_TRIP) ||
             (status_log_entry == HIGH_GN_TRIP))
    {
       latest_logcode_led_display.output_code = 5;
       latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
    }
    else if ((( status_log_entry >= CONTINUOUS_HW_FAULT_CODES_BEGIN ) && ( status_log_entry <= CONTINUOUS_HW_FAULT_CODES_END )) ||
             (( status_log_entry >= USER_INITIATED_HW_FAULT_CODES_BEGIN ) && ( (u16)status_log_entry <= USER_INITIATED_HW_FAULT_CODES_END )))
    {
       latest_logcode_led_display.output_code = 6;
       latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;
    }
    else
    {
       latest_logcode_led_display.output_code = 0;
       latest_logcode_led_display.count.times_to_blink = 0;
    }

#endif

}

void trip_log_update_blink_latest_logcode_led(u32 it)
{
	//if(get_startup_logged_flag() == FALSE)
	//{
	//	return;// did not get startup yet so we may not be initialized properly yet.
	//}
#if BOARD_CONFIGURATION == BOARD_SB2
	if((get_primary_switch_debounced_state() == SW_TRIP)
			&& (get_startup_logged_flag() == TRUE)
		)
	{
		latest_logcode_led_display.count.times_to_blink = TRIP_LOG_LED_DISPLAY_NUMBER_OF_CODES_TO_OUTPUT;

		// Here use the LED pattern to determine if a no-fault trip code is detected
		// If a new trip code is added, make sure the corresponding LED pattern is defined too
		if( latest_logcode_led_display.led_display_color == LED_UNASSIGNED_COLOR ||
	    	latest_logcode_led_display.output_code == 0)
	    {
	        //TODO determine if we should write a trip code or just send one to the cloud.
			//We either need to save the misc. fault or report all clear on startup.
			#if defined SAVE_RUNTIME_FAULTS
			trip_log_log_HAL_fault();
			//u8 trip_code = PRIMARY_HAL_TRIP_OR_OFF;
			//trip_log_write_tripcode(&trip_code, 1);
			#else
			queue_uart_message(pm2m_uart_comm, BREAKER_FAULT_STATE, PRIMARY_HAL_TRIP_OR_OFF, NO_PAYLOAD_2, NO_PAYLOAD_2);
			#endif
	        latest_logcode_led_display.output_code = 5;
	        latest_logcode_led_display.led_display_color = LED_CYAN;
	    }

	}
	else if ((led_get_key_status() == KEY_TAKEN_BY_BLINK_LATEST_LOG_CODE)
			&& (get_primary_switch_debounced_state() != SW_UNKNOWN)
			&& (get_primary_switch_debounced_state() != SW_FAILED)
			)
	{
		//TODO see if normal startup should be used to save fault cleared instead.
		#if defined SAVE_ALL_CLEAR
		trip_log_log_fault_cleared();
		#else
		queue_uart_message(pm2m_uart_comm, BREAKER_FAULT_STATE, NO_TRIP, NO_PAYLOAD_2, NO_PAYLOAD_2);
		#endif

		//clear out codes as we should not be in the triped state and thus not blinking.
		latest_logcode_led_display.count.times_to_blink = 0;
		latest_logcode_led_display.count.num_of_nonblink_between_code = 0;
		latest_logcode_led_display.count.total_half_cycle_per_blink = 0;
		latest_logcode_led_display.output_code = 0;
		latest_logcode_led_display.led_display_color = LED_WHITE;
		led_turn_off();
		//LED addigned to white because LED request below will not request an unasigned off.
		led_request(latest_logcode_led_display.led_display_color, LED_TURN_OFF);
		led_reset_key_status();
		//queue_uart_message(pm2m_uart_comm, BREAKER_FAULT_STATE, NO_TRIP, NO_PAYLOAD_2, NO_PAYLOAD_2);
	}
#endif

    if((led_get_key_status() == KEY_FREE) &&
       (latest_logcode_led_display.count.times_to_blink != 0)
       && (get_startup_indicated_flag() == TRUE)
	   && (get_startup_logged_flag() == TRUE)
       )
    {
        led_set_key_status(KEY_TAKEN_BY_BLINK_LATEST_LOG_CODE);
    }

    /* get quickblink flag to avoid overlapping of led blinks */
    if((quickblink_enable_flag == 0) && (led_get_key_status() == KEY_TAKEN_BY_BLINK_LATEST_LOG_CODE)
#if AUTO_MONITOR_CONFIGURATION == AUTO_MONITOR_ENABLED
       && (auto_monitor_get_result() != AUTO_MONITOR_TRIP_TEST_FAIL)
#endif
      )
    {
        /* check if tllt is requested */ //If so clear display and release key for tllt?
        if((trip_log_get_led_xfer_progress() == TRIP_LOG_LED_XFER_NOT_IN_PROGRESS))
        {

            if (( latest_logcode_led_display.count.times_to_blink == 0 ) &&
                ( latest_logcode_led_display.count.num_of_nonblink_between_code == 0 ) &&
                ( latest_logcode_led_display.count.total_half_cycle_per_blink == 0 ))
            {
               latest_logcode_led_display.output_code = 0;
               led_request(latest_logcode_led_display.led_display_color, LED_TURN_OFF);
               led_reset_key_status();

			   #if BOARD_CONFIGURATION == BOARD_SB2
               //When we are no longer blinking send the all clear.
               //Having it here makes it finish all its remaining blinks. I think I'd rather clear them right away.
               if((get_primary_switch_debounced_state() != SW_TRIP)
            		&& (get_primary_switch_debounced_state() != SW_UNKNOWN)
            		&& (get_primary_switch_debounced_state() != SW_FAILED)
            	  )
               {
				   #if defined SAVE_ALL_CLEAR
				   trip_log_log_fault_cleared();
				   #else
				   queue_uart_message(pm2m_uart_comm, BREAKER_FAULT_STATE, NO_TRIP, NO_PAYLOAD_2, NO_PAYLOAD_2);
				   #endif
            	   led_turn_off();//safety turn off
               }
			   #endif
            }
            else
            {
               if (( latest_logcode_led_display.count.num_of_nonblink_between_code == 0 ) && ( latest_logcode_led_display.count.total_half_cycle_per_blink == 0 ))
               {
                  latest_logcode_led_display.count.times_to_blink--;
                  latest_logcode_led_display.count.num_of_nonblink_between_code = (latest_logcode_led_display.output_code + TRIP_LOG_LED_DISPLAY_NUMBER_OF_NONBLINKS_BETWEEN_CODES - 1);
                  latest_logcode_led_display.count.total_half_cycle_per_blink = TRIP_LOG_LED_DISPLAY_TOTAL_HALF_CYCLES_PER_BLINK;

               }
               else if (latest_logcode_led_display.count.total_half_cycle_per_blink == 0)
               {
                  latest_logcode_led_display.count.num_of_nonblink_between_code--;
                  latest_logcode_led_display.count.total_half_cycle_per_blink = TRIP_LOG_LED_DISPLAY_TOTAL_HALF_CYCLES_PER_BLINK;
               }
               else
               {
                  latest_logcode_led_display.count.total_half_cycle_per_blink--;
               }

               if (( latest_logcode_led_display.count.num_of_nonblink_between_code >= latest_logcode_led_display.output_code ) ||
                   ( latest_logcode_led_display.count.total_half_cycle_per_blink > TRIP_LOG_LED_DISPLAY_HALF_CYCLES_PER_LED_BLINK_ON ))
               {
                   led_request(latest_logcode_led_display.led_display_color, LED_TURN_OFF);
               }
               else
               {
                   led_request(latest_logcode_led_display.led_display_color, LED_TURN_ON);
               }
            }
        }
        else
        {
            //latest_logcode_led_display.output_code = 0; //removed to allow trip to continue blinking after tllt finishes.
            led_request(latest_logcode_led_display.led_display_color, LED_TURN_OFF);
            led_reset_key_status();
        }
    }

#if AUTO_MONITOR_CONFIGURATION == AUTO_MONITOR_ENABLED
    /* if blink latest log code has started but automonitor has a trip circuit fault.
     * stop and return the key. so automonitor can light led. */
    else if ((led_get_key_status() == KEY_TAKEN_BY_BLINK_LATEST_LOG_CODE) &&
             (auto_monitor_get_result() == AUTO_MONITOR_TRIP_TEST_FAIL))
    {
        latest_logcode_led_display.output_code = 0;
        latest_logcode_led_display.count.times_to_blink = 0;
        led_request(latest_logcode_led_display.led_display_color, LED_TURN_OFF);
        led_reset_key_status();
    }
#endif

}

void trip_log_blink_latest_logcode_category()
{
    u8 most_recent_status_log_entry = trip_log_find_latest_fault_code_after_anchor();

    trip_log_map_logcode_value_to_logcode_category(most_recent_status_log_entry);

    if( !((most_recent_status_log_entry >= NORMAL_STARTUP) && (most_recent_status_log_entry <= RECALL_RESET_ANCHOR)) )
    {
        main_requeset_reset_anchor();
    }

    return;
}

#if BOARD_CONFIGURATION != BOARD_SB2 //Smart Breaker boards do not contain spi interface
void trip_log_output_trip_log_via_spi(void)
{
    u8* data_p = 0;
    u8  data_byte = 0;
    u16 counter = 0;
    u16 log_transmit_count = 0;
    u16 num_tripcodes = 0;
    u32 trip_log_output_start_addr;

    u8 trip_code_to_write = OUTPUT_STATUS_LOG_VIA_SPI;

  #if RATING_SHUNT_CAL_CONFIGURATION != RATING_SHUNT_CAL_NOT_ENABLED
    u8 breaker_handle_rating = (u8)shunt_cal_get_breaker_rating();
    u8 breaker_cal_factor = shunt_cal_get_breaker_cal_factor();
  #else
    u8 breaker_handle_rating = 0;
    u8 breaker_cal_factor = 0;
  #endif

    u8 breaker_rating_ones_complement = ~breaker_handle_rating;
    u8 breaker_cal_factor_ones_complement = ~breaker_cal_factor;

    iwdg_kick_watchdog();

    /* initialize spi component */
    spi_init_component();

    iwdg_kick_watchdog();

    /** get count of trip codes */
    for(data_p = (u8*)TRIP_LOG_FIRST_TRIPCODE_ADDRESS; data_p <= (u8*)TRIP_LOG_LAST_TRIPCODE_ADDRESS; data_p+=8)
    {
        /** find first empty trip code space */
        if((*data_p == 0x00) || (*data_p == 0xFF))
        {
            break;
        }
        else
        {
            num_tripcodes++;
        }

        iwdg_kick_watchdog();
    }

    /** The pen tool will only accept a fixed number of bytes. This number is less than what
     * the space reserved in flash for trip codes will allow us to write. This means we
     * could end up with more trip log entries than we would be able to send to the pen tool.
     * To account for this, in the situation where we have more trip codes than we can send we
     * change the starting point for the trip log transfer. This ensures that the most recent
     * log codes are sent to the pen tool.
     *      */
    trip_log_output_start_addr  = TRIP_LOG_FIRST_TRIPCODE_ADDRESS;

    if(num_tripcodes > TRIP_LOG_MAX_TRIPCODES_FOR_PEN_TOOL)
    {
        trip_log_output_start_addr += ((num_tripcodes - TRIP_LOG_MAX_TRIPCODES_FOR_PEN_TOOL) << 3);
        num_tripcodes = TRIP_LOG_MAX_TRIPCODES_FOR_PEN_TOOL;
    }

    /** turn on the LED as transmission is ongoing */
    LED_RED_ON;


    for(log_transmit_count = 0; log_transmit_count < NUM_TIMES_TO_OUTPUT_TRIP_LOG; log_transmit_count++)
    {

        iwdg_kick_watchdog();

        /** this points data_p at the upper 2 bytes of the checksum. */
        data_p = ((u8*)&__build_checksum + (sizeof(__build_checksum)-2));

        /** Sends only the first 2 bytes of the checksum 3 times (6 bytes) */
        for(counter = 0; counter < 3; counter++)
        {
            /** upper byte is index 1 because of the -2 used when assigning an address to data_p */
            spi_transmit(SPI1, &data_p[1], 1);
            spi_transmit(SPI1, data_p, 1);
        }

        iwdg_kick_watchdog();

        /** send the breaker handle rating and calibration factor (and their ones compliments). */

        data_p = (u8*)&breaker_handle_rating;
        spi_transmit(SPI1, data_p, 1);

        data_p = &breaker_rating_ones_complement;
        spi_transmit(SPI1, data_p, 1);


        iwdg_kick_watchdog();

        data_p = &breaker_cal_factor;
        spi_transmit(SPI1, data_p, 1);

        data_p = &breaker_cal_factor_ones_complement;
        spi_transmit(SPI1, data_p, 1);

        iwdg_kick_watchdog();

        /** send trip codes */
        data_p = (u8*)(trip_log_output_start_addr);

        for(counter = 0; counter < num_tripcodes; counter++)
        {
            spi_transmit(SPI1, data_p, 1);
            data_p+=8;

            iwdg_kick_watchdog();
        }

        /** The pen tool expects the full trip log and will timeout/fail if any bytes are missing from the
            received packet. The unwritten trip log bytes are sent separately here because they are
            0xFF in flash whereas the pen tool considers 0x00 to be the blank value. */
        if(num_tripcodes < TRIP_LOG_MAX_TRIPCODES_FOR_PEN_TOOL)
        {
            data_byte = NO_TRIP_CODE;
            data_p = &data_byte;

            for(counter = 0; counter < ((TRIP_LOG_MAX_TRIPCODES_FOR_PEN_TOOL) - num_tripcodes); counter++)
            {
                spi_transmit(SPI1, data_p, 1);

                iwdg_kick_watchdog();
            }
        }

        iwdg_kick_watchdog();

        /** these last 16 bytes of data are specific to the 32-bit breakers working with the 3rd
            iteration of the pen tool. This way 32-bit breakers can have the pen tool output the
            firmware version */

        data_byte = MSG_TOKEN;
        data_p = &data_byte;
        spi_transmit(SPI1, data_p, 1);

        data_byte = 0xFF;
        for(counter = 0; counter < RESERVED_BYTES_SIZE; counter++)
        {
            spi_transmit(SPI1, data_p, 1);
        }

        iwdg_kick_watchdog();

        /** send 32-bit checksum and 8 bytes of version info
           NOTE: order is reversed due to endianness */
        data_p = ((u8*)&__build_checksum + sizeof(__build_checksum)-1);

        for(counter = 0; counter < sizeof(__build_checksum); counter++)
        {
            iwdg_kick_watchdog();
            spi_transmit(SPI1, data_p, 1);
            data_p--;
        }

        iwdg_kick_watchdog();
        data_p = ((u8*)&__version + sizeof(__version)-1);

        for(counter = 0; counter < sizeof(__version); counter++)
        {
            iwdg_kick_watchdog();
            spi_transmit(SPI1, data_p, 1);
            data_p--;
        }

        iwdg_kick_watchdog();

        /** delay for a bit between transfers of trip log data */
        timer_delay(DELAY_BETWEEN_TRIP_LOG_OUTPUTS);

        iwdg_kick_watchdog();
    }

    /** write OUTPUT_STATUS_LOG_VIA_SPI tripcode after transfer */
    trip_log_write_tripcode(&trip_code_to_write, 1);

    iwdg_kick_watchdog();

    /** turn led off */
    LED_RED_OFF;

    /* stay in while loop to prevent watchdog reset, to prevent complete loss of zcd being logged */
    while(1)
    {
        iwdg_kick_watchdog();
    }

}

void trip_log_check_for_output_trip_log_via_spi(u32 it)
{
    /* check if MISO pin is being set by pin tool */
#warning the counter is set pretty low becasue the timer takes a long time to trigger when no zcds, this may need to be discussed
    if(gpio_read_input_pin(SPI_MISO_GPIO_Port, SPI_MISO_Pin) ==  GPIO_PIN_SET)
    {
        iwdg_kick_watchdog();
        output_status_log_counter++;

        /* once entered, it should never come back. */
        if(output_status_log_counter >= TRIP_LOG_OUTPUT_STATUS_LOG_COUNTER_THRESHOLD)
        {
            __disable_irq();
            trip_log_output_trip_log_via_spi();
        }
    }
    else
    {
        output_status_log_counter = 0;
    }

    return;
}

#endif

void* trip_log_get_next_available_location(void)
{
    u64* address_ptr;

    for(address_ptr = (u64*)TRIP_LOG_FIRST_TRIPCODE_ADDRESS;
        address_ptr <= (u64*)TRIP_LOG_LAST_TRIPCODE_ADDRESS;
        address_ptr++
       )
    {
        /* if full, return null */
        if((address_ptr == (u64*)TRIP_LOG_LAST_TRIPCODE_ADDRESS) &&
           (*address_ptr != TRIP_LOG_TRIPCODE_FACTORY_VALUE)
          )
        {
            address_ptr = NULL;
            break;
        }

        /* if equal factory value, then available location is found.*/
        if(*address_ptr == TRIP_LOG_TRIPCODE_FACTORY_VALUE)
        {
            break;
        }
    }

    return (void*)address_ptr;
}

u8 trip_log_find_latest_fault_code(void)
{
    u8* data_p = 0;

    /* Find the latest trip code that is an actual fault i.e not 0x01 or 0x02... */
    for(data_p = (u8*)TRIP_LOG_LAST_TRIPCODE_ADDRESS;
        data_p > (u8*)TRIP_LOG_FIRST_TRIPCODE_ADDRESS;
        data_p-=8
       )
    {
        /*USER_INITIATED_HW_FAULT_CODES_END-1 is because 0xFF is what is out of the factory, and there
        should be not trip code that is 0xFF anyway. */
        if ((( *data_p >= PROTECTION_FAULT_CODES_BEGIN ) &&
             ( *data_p <= PROTECTION_FAULT_CODES_END )) ||
            (( *data_p >= CONTINUOUS_HW_FAULT_CODES_BEGIN ) &&
             ( *data_p <= CONTINUOUS_HW_FAULT_CODES_END )) ||
            (( *data_p >= USER_INITIATED_HW_FAULT_CODES_BEGIN ) &&
             ( *data_p <= USER_INITIATED_HW_FAULT_CODES_END-1 )))
        {
            break;
        }
    }

    return *data_p;
}

u8 trip_log_find_latest_fault_code_after_anchor(void)
{
    u8* data_p = 0;

    /* Find the latest trip code that is an actual fault i.e not 0x01 or 0x02... */
    for(data_p = (u8*)TRIP_LOG_LAST_TRIPCODE_ADDRESS;
        data_p > (u8*)TRIP_LOG_FIRST_TRIPCODE_ADDRESS;
        data_p-=8
       )
    {
        /*USER_INITIATED_HW_FAULT_CODES_END-1 is because 0xFF is what is out of the factory, and there
        should be not trip code that is 0xFF anyway. */
        if ((( *data_p >= PROTECTION_FAULT_CODES_BEGIN ) &&
             ( *data_p <= PROTECTION_FAULT_CODES_END )) ||
            (( *data_p >= CONTINUOUS_HW_FAULT_CODES_BEGIN ) &&
             ( *data_p <= CONTINUOUS_HW_FAULT_CODES_END )) ||
            (( *data_p >= USER_INITIATED_HW_FAULT_CODES_BEGIN ) &&
             ( *data_p <= USER_INITIATED_HW_FAULT_CODES_END-1 )) ||
            (*data_p == END_OF_FACTORY_ANCHOR) ||
            (*data_p == RECALL_RESET_ANCHOR))
        {
            break;
        }
    }

    return *data_p;
}

bool trip_log_check_latest_three_codes(void)
{
    u8* data_p = 0, code_temp[3] = {0,1,0};

    /* Find the latest trip code that is an actual fault i.e not 0x01 or 0x02... */
    for(data_p = (u8*)TRIP_LOG_LAST_TRIPCODE_ADDRESS;
        data_p > (u8*)TRIP_LOG_FIRST_TRIPCODE_ADDRESS;
        data_p-=8
       )
    {
        /*USER_INITIATED_HW_FAULT_CODES_END-1 is because 0xFF is what is out of the factory, and there
        should be not trip code that is 0xFF anyway. */
        if ((( *data_p >= PROTECTION_FAULT_CODES_BEGIN ) &&
             ( *data_p <= PROTECTION_FAULT_CODES_END )) ||
            (( *data_p >= CONTINUOUS_HW_FAULT_CODES_BEGIN ) &&
             ( *data_p <= CONTINUOUS_HW_FAULT_CODES_END )) ||
            (( *data_p >= USER_INITIATED_HW_FAULT_CODES_BEGIN ) &&
             ( *data_p <= USER_INITIATED_HW_FAULT_CODES_END-1 )))
        {
            code_temp[0] = *data_p;
            if((data_p - 8) > (u8*)TRIP_LOG_FIRST_TRIPCODE_ADDRESS)
            {
                code_temp[1] = *(data_p -8);
            }
            else
            {
                return FALSE;
            }
            if((data_p - 16) > (u8*)TRIP_LOG_FIRST_TRIPCODE_ADDRESS)
            {
                code_temp[2] = *(data_p - 16);
            }
            else
            {
                return FALSE;
            }
            break;
        }
    }

    // check if three codes are the same
    if(code_temp[0] == code_temp[1] && code_temp[2] == code_temp[1])
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }

}

u8 trip_log_find_latest_log_code(void)
{
    u8* data_p = 0;

    /* Find the latest log code */
    for(data_p = (u8*)TRIP_LOG_FIRST_TRIPCODE_ADDRESS;
        data_p <= (u8*)TRIP_LOG_LAST_TRIPCODE_ADDRESS;
        data_p+=8
       )
    {
        if(*data_p == 0xFF)
        {
            break;
        }
    }

    if(data_p != (u8*)TRIP_LOG_FIRST_TRIPCODE_ADDRESS)
    {
        data_p-=8;
    }

    return *data_p;
}


#if OVERLOAD_CONFIGURATION == OVERLOAD_ENABLED

void trip_log_quickblink_init( void )
{
    quickblink_enable_flag = 1;

    /* get the key at initialization */
    if((led_get_key_status() == KEY_FREE))
    {
        led_set_key_status(KEY_TAKEN_BY_QUICk_BLINK);
    }

    return;

}

void trip_log_quickblink_indicate_overload_setting( void )
{
    static u8 quickblink_counter = 0;
    load_current_rating_t breaker_def= LOAD_CURRENT_RATING_NOT_DEFINED;

    if(quickblink_enable_flag == 1 && (led_get_key_status() == KEY_TAKEN_BY_QUICk_BLINK))
    {
      quickblink_counter ++;
#warning update voltage lockout threshold
      if((quickblink_counter > PRG_BEGIN_COUNTER_MAX) ||
         ((quickblink_counter >= QUICK_BLINK_01) &&
          (btn_get_button_state(BTN_1_ID) == BTN_PRESSED_SINCE_STARTUP) &&
          (btn_get_status(BTN_1_ID) == BTN_DOWN) &&
          (load_voltage_get_primary_peak() > LOAD_VOLTAGE_UNDERVOLTAGE_LOCKOUT)))
      {
        quickblink_counter = 0;
        quickblink_enable_flag = 0;
        led_reset_key_status();
      }
      /* get breaker definition  */
      breaker_def = shunt_cal_get_breaker_rating();
      /* first flash */
      if(quickblink_counter == QUICK_BLINK_01)
      {
          led_request(LED_RED, LED_TURN_ON);
      }
      else if(quickblink_counter == QUICK_BLINK_02)
      {
          led_request(LED_RED, LED_TURN_OFF);
      }
      /* the second flash */
      if((breaker_def == LOAD_CURRENT_RATING_20A)||(breaker_def == LOAD_CURRENT_RATING_15A)
#if RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_25_30A
      || (breaker_def == LOAD_CURRENT_RATING_25A)
#endif // OVERLOAD_25A
#if ((RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_25_30A) ||\
(RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_30A))
      || (breaker_def == LOAD_CURRENT_RATING_30A)
#endif // OVERLOAD_30A
      )
      {
        if(quickblink_counter == QUICK_BLINK_03)
        {
            led_request(LED_RED, LED_TURN_ON);
        }
        else if(quickblink_counter == QUICK_BLINK_04)
        {
            led_request(LED_RED, LED_TURN_OFF);
        }
      }

      /* the third flash */
      if((breaker_def == LOAD_CURRENT_RATING_20A)
#if RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_25_30A
        || (breaker_def == LOAD_CURRENT_RATING_25A)
#endif // OVERLOAD_25A
#if ((RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_25_30A) ||\
(RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_30A))
        || (breaker_def == LOAD_CURRENT_RATING_30A)
#endif // OVERLOAD_30A
      )
      {
        if(quickblink_counter == QUICK_BLINK_05)
        {
            led_request(LED_RED, LED_TURN_ON);
        }
        else if(quickblink_counter == QUICK_BLINK_06)
        {
            led_request(LED_RED, LED_TURN_OFF);
        }
      }

#if RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_25_30A
      /* the fourth flash*/
      if(breaker_def == LOAD_CURRENT_RATING_25A)
      {
        if(quickblink_counter == QUICK_BLINK_07)
        {
            led_request(LED_RED, LED_TURN_ON);
        }
        else if(quickblink_counter == QUICK_BLINK_08)
        {
            led_request(LED_RED, LED_TURN_OFF);
        }
      }
#endif // OVERLOAD_25A

#if ((RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_25_30A) ||\
(RATING_SHUNT_CAL_CONFIGURATION == RATING_SHUNT_CAL_15_20_30A))
      /*the fourth && fifth flash */
      if(breaker_def == LOAD_CURRENT_RATING_30A)
      {
        if(quickblink_counter == QUICK_BLINK_07)
        {
            led_request(LED_RED, LED_TURN_ON);
        }
        else if(quickblink_counter == QUICK_BLINK_08)
        {
            led_request(LED_RED, LED_TURN_OFF);
        }
        if(quickblink_counter == QUICK_BLINK_09)
        {
            led_request(LED_RED, LED_TURN_ON);
        }
        else if(quickblink_counter == QUICK_BLINK_10)
        {
            led_request(LED_RED, LED_TURN_OFF);
        }
      }
#endif // OVERLOAD_30A
    }
}

inline u8 trip_log_get_quickblink_flag(void)
{
    return quickblink_enable_flag;
}
#endif //OVERLOAD_PROTECTION




