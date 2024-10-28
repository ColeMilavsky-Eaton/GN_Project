/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 **************************************************************************************************
 *  Written by:
 *//**
 * @brief internal (to the component) interface for the main component.
 *
 * @file usart.c
 * @ingroup usart
 *
 *//*
 *
 **************************************************************************************************/

#include "usart.h"
#include "stdlib.h"
#include "string.h"

M2M_UART_COMMN *pm2m_uart_comm;
//CRC_HandleTypeDef hcrc;

//UART_HandleTypeDef huart2;

USART_TypeDef USARTx;

Device_Info DeviceInfo = {0};

st_rgb_led_blink rgb_led_blink = {0};

M2M_UART_QUEUE m2m_queue;

uint8_t msg_counter;
bool empty_led_rate_received_flag = FALSE;

extern bool led_idenfify_me_request_flag;
extern bool led_idenfify_me_on_going_flag;
extern u8 led_identify_me_blink_mode;
extern u32 hall_data_adc;
extern u32 total_gf_current;
extern bool startup_conf;
extern u64 __version;
extern s16 temperature_c_trending;

/**
* @brief Calculate CRC8 value
*
* @param data: Data buffer that used to calculate the CRC value
* @param length: Length of the data buffer
* @return CRC8 value
*/
unsigned char CRC8(const unsigned char *data,int length)
{
   unsigned char crc = 0x00;
   unsigned char extract;
   unsigned char sum;
   for(int i=0;i<length;i++)
   {
      extract = *data;
      for (unsigned char tempI = 8; tempI; tempI--)
      {
         sum = (crc ^ extract) & 0x01;
         crc >>= 1;
         if (sum)
            crc ^= 0x8C;
         extract >>= 1;
      }
      data++;
   }
   return crc;
}

/**
* @brief Process breaker open/close status M2M UART CMD from protection MCU
*
* @param M2M_UART_COMMN : pointer to the M2M protocol command format variable
* @return success - 0, failure - (-1)
*/
int process_open_close(M2M_UART_COMMN *pm2m_uart_comm)
{
	uint8_t open_close = pm2m_uart_comm->payload_1;
	//printf("process_open_close\n");
	switch(open_close)
	{
		case SECONDARY_CONTACTS_OPEN:
			//printf("SECONDARY_CONTACTS_OPEN\n");
			request_open_secondary_solenoid();
			break;
		case SECONDARY_CONTACTS_CLOSE:
			//printf("SECONDARY_CONTACTS_CLOSE\n");
			request_close_secondary_solenoid();
			break;
		case SECONDARY_CONTACTS_TOGGLE:
			//printf("SECONDARY_CONTACTS_TOGGLE\n");
			request_toggle_secondary_solenoid();
			break;
		default:
			break;

	}
	return 0;
}

/**
* @brief Process Comm MCU WiFi status and update LED
*
* @param M2M_UART_COMMN : pointer to the M2M protocol command format variable
* @return success - 0, failure - (-1)
*/
int process_wifi_status(M2M_UART_COMMN *pm2m_uart_comm)
{
	//uint8_t wifi_status = pm2m_uart_comm->payload_1;

	//printf("BREAKER_PROVISION_START, Update LED Color n Blink Rate\n");
	if(BREAKER_PROVISION_START == pm2m_uart_comm->payload_1)
	{
		//printf("BREAKER_PROVISION_START, Update LED Color n Blink Rate\n");
	}
//	button_1_status = 0;

	return 0;
}

void process_uart_ack(M2M_UART_COMMN *pm2m_uart_comm)
{
	if(pm2m_uart_comm->payload_1 == POSITIVE_ACK)
	{
		if((a_m2m_message_is_queued()) && (pm2m_uart_comm->msg_num == m2m_queue.message[m2m_queue.last_sent_index].msg_num))
		{
			m2m_queue.wait_for_ack = 0;
			m2m_queue.number_of_retries = MAX_NUMBER_OF_RETRIES;
			m2m_queue.last_sent_index++;
			if(m2m_queue.last_sent_index >= MAX_UART_QUEUE_SIZE)
			{
				m2m_queue.last_sent_index = 0;
			}
		}
	}
}

void send_uart_ack(M2M_UART_COMMN *pm2m_uart_comm)
{
	uint8_t len = 0;
	const uint8_t *pm2m_data = (const void*)pm2m_uart_comm;

	pm2m_uart_comm->start_flag      = START_BYTE;
	pm2m_uart_comm->cmd             = COMM_MCU_ACK;
	//pm2m_uart_comm->msg_num			= pm2m_uart_comm->msg_num; //leave message num as it was received
	pm2m_uart_comm->len             = sizeof(M2M_UART_COMMN);
	pm2m_uart_comm->payload_1       = POSITIVE_ACK;
	pm2m_uart_comm->payload_2       = NO_PAYLOAD_1;
	pm2m_uart_comm->payload_3       = NO_PAYLOAD_1;
	//subtract 2 to ignore crc and end byte.
	pm2m_uart_comm->crc             = CRC8((const uint8_t *)pm2m_uart_comm, (pm2m_uart_comm->len-2));
	pm2m_uart_comm->end_flag        = END_BYTE;

	//send uart command
	for (len = (sizeof(M2M_UART_COMMN)); len > 0; --len, ++pm2m_data)
	{
		LL_USART_TransmitData8(USART2, *pm2m_data);
		//printf("%x->", *pm2m_data);
		//need to check below call as this might cause deadlock
		while (!LL_USART_IsActiveFlag_TXE(USART2)) {}
	}
}


/**
* @brief Process command from communication MCU
*
* @param M2M_UART_COMMN : pointer to the M2M protocol command format variable
* @return success - 0, failure - (-1)
*/
int process_cmd_from_communication_mcu(M2M_UART_COMMN *pm2m_uart_comm)
{
	switch(pm2m_uart_comm->cmd)
	{
		case SWITCH2_PRESS_PROVISION:
		{
			if(UnProvisioned == pm2m_uart_comm->payload_1)
			{
				//printf("SWITCH2_PRESS_PROVISION, Update LED Color n Blink Rate\n");
				//add update after design final for button and LED
				DeviceInfo.provision_status = UnProvisioned;

			}
			else if(ProvisionStarted ==  pm2m_uart_comm->payload_1)
			{
				//printf("SWITCH2_PRESS_PROVISION, Update LED Color n Blink Rate\n");
				//add update after design final for button and LED
				DeviceInfo.provision_status = ProvisionStarted;
			}
			else
			{
				if(Provisioned ==  pm2m_uart_comm->payload_1)
				{
					//printf("SWITCH2_PRESS_PROVISION, Update LED Color n Blink Rate\n");
					//add update after design final for button and LED
					DeviceInfo.provision_status = Provisioned;
				}

			}
		}
		break;
//		case LED_COLOR:
//		{
////			printf("LED_COLOR 1::%d\n",pm2m_uart_comm->payload_1);
////			printf("LED_COLOR 2::%d\n",pm2m_uart_comm->payload_2);
//			//if(pm2m_uart_comm->payload_1)//payload_1 check with RGB color to blink
//			{
//				gRGB_Color = pm2m_uart_comm->payload_1;
//			}
		//}
//		break;
		case LED_COLOR_BLINK_RATE:
		{
//			printf("LED_COLOR_BLINK_RATE 1::%d\n",pm2m_uart_comm->payload_1);
//			printf("LED_COLOR_BLINK_RATE 2::%d\n",pm2m_uart_comm->payload_2);
			//if((pm2m_uart_comm->payload_1) )//payload_1 check with RGB color to blink
			{
				rgb_led_blink.rgb_led_color              = (led_color_t)pm2m_uart_comm->payload_1;
				rgb_led_blink.blinking_time_interval     = ((u16)pm2m_uart_comm->payload_2 * 6);
				rgb_led_blink.blinking_time_expire       = ((u16)pm2m_uart_comm->payload_3 * 6);
				//rgb_led_blink.blinking_time_interval     = pm2m_uart_comm->payload_2;
				//rgb_led_blink.blinking_time_expire       = pm2m_uart_comm->payload_3;
				rgb_led_blink.start_blinking_leds_flag = TRUE;
			}
		}
		break;
		default :
			break;
	}

	return 0;
}

/**
* @brief Process Identify Me
*
* @param M2M_UART_COMMN : pointer to the M2M protocol command format variable
* @return success - 0, failure - (-1)
*/
void process_identify_me(M2M_UART_COMMN *pm2m_uart_comm)
{
	// The LED blink mode corresponding to factory or normal setup.
	led_identify_me_blink_mode = pm2m_uart_comm->payload_1;

    // If the identify process is ongoing, ignore the command
    if (led_idenfify_me_on_going_flag != TRUE)
    {
        led_idenfify_me_request_flag = TRUE;
    }
}

/**
* @brief Process messages that set the startup configuration.
*
* @param M2M_UART_COMMN : pointer to the M2M protocol command format variable
* @return void
*
* @Note To properly function this should happen after the pin gets set as an output..
*/
void process_startup_config(M2M_UART_COMMN *pm2m_uart_comm)
{
	if(pm2m_uart_comm->payload_1 == 0x01)
	{
		LL_GPIO_SetOutputPin(STARTUP_CAP_CONF_GPIO_Port, STARTUP_CAP_CONF_Pin);
		startup_conf = 1;
	}
	else
	{
		LL_GPIO_ResetOutputPin(STARTUP_CAP_CONF_GPIO_Port, STARTUP_CAP_CONF_Pin);
		startup_conf = 0;
	}
}

/**
* @brief Process LED blink rate
*
* @param M2M_UART_COMMN : pointer to the M2M protocol command format variable
* @return success - 0, failure - (-1)
*/
int process_led_blink_rate(M2M_UART_COMMN *pm2m_uart_comm)
{
//	printf("process_led_blink_rate 1::%d\n",pm2m_uart_comm->payload_1);
//	printf("process_led_blink_rate 2::%d\n",pm2m_uart_comm->payload_2);
//	printf("process_led_blink_rate 3::%d\n",pm2m_uart_comm->payload_3);
	//if(pm2m_uart_comm->payload_1)//payload_1 check with RGB color to blink
	{
		rgb_led_blink.rgb_led_color              = (led_color_t)pm2m_uart_comm->payload_1;
		rgb_led_blink.blinking_time_interval     = ((u16)pm2m_uart_comm->payload_2 * 6);
		rgb_led_blink.blinking_time_expire       = ((u16)pm2m_uart_comm->payload_3 * 6);
		//rgb_led_blink.blinking_time_interval     = pm2m_uart_comm->payload_2;
		//rgb_led_blink.blinking_time_expire       = pm2m_uart_comm->payload_3;
		rgb_led_blink.start_blinking_leds_flag = TRUE;
		if(rgb_led_blink.blinking_time_interval == 0 && rgb_led_blink.blinking_time_expire == 0)
		{
		    empty_led_rate_received_flag = TRUE;
		}
		else
		{
		    empty_led_rate_received_flag = FALSE;
		}
	}
	return 0;
}

/**
* @brief Process M2M protocol UART command from protection MCU
*
* @param M2M_UART_COMMN : pointer to the M2M protocol command format variable
* @return success - 0, failure - (-1)
*/
int process_uart_packet(M2M_UART_COMMN *pm2m_uart_comm)
{
	switch(pm2m_uart_comm->cmd)
	{
		case BREAKER_OPEN_CLOSE:
			process_open_close(pm2m_uart_comm);
			send_uart_ack(pm2m_uart_comm);
			break;
		case READ_WIFI_STATUS:
			process_wifi_status(pm2m_uart_comm);
			send_uart_ack(pm2m_uart_comm);
			break;
		case PROTECTION_MCU_ACK:
			process_uart_ack(pm2m_uart_comm);
			break;
		case READ_COMMUNICATION_MCU_CMD:
			process_cmd_from_communication_mcu(pm2m_uart_comm);
			send_uart_ack(pm2m_uart_comm);
			break;
//
//		case READ_ESP_STATUS:
//			break;
//
		case READ_ESP_UART_STATUS:
		{
			if((!DeviceInfo.uart_status) && (pm2m_uart_comm->cmd == READ_ESP_UART_STATUS)
					&& (pm2m_uart_comm->payload_1 == ESP_UART_READY))
			{
				DeviceInfo.uart_status = ESP_UART_READY;
				//printf("\nSTM32 UART READY : M2M UP\n");
			}
		}
		break;
		case LED_COLOR_BLINK_RATE:
			process_led_blink_rate(pm2m_uart_comm);
			send_uart_ack(pm2m_uart_comm);
			break;
		case SET_STARTUP_CONFIG:
			process_startup_config(pm2m_uart_comm);
			send_uart_ack(pm2m_uart_comm);
			break;
        case IDENTIFY_ME:
            process_identify_me(pm2m_uart_comm);
            send_uart_ack(pm2m_uart_comm);
            break;
        case UPDATE_TEMPERATURE:
            send_to_communication_mcu(pm2m_uart_comm,
                    UPDATE_TEMPERATURE,
                    ((u16)temperature_c_trending >> 8) & 0xFF,
                    ((u16)temperature_c_trending >> 0) & 0xFF,
                    NO_PAYLOAD_1);
//            send_uart_ack(pm2m_uart_comm);
            break;
		case ERROR_RESPONSE:
			break;
		case BREAKER_PRTOTECTION_FW_VERSION:
		{
			update_protection_fw_version_to_communication();
		}
			break;
		default:
		{
			//Error response to user/cloud or if LED is finalized then use LED to show error
			//if error response received update a flag or use a queue to always send a command to
			//protection MCU
			//uart_recvd_err =  true;
			//prepare uart command for crc error response
			send_to_communication_mcu(pm2m_uart_comm, ERROR_RESPONSE, INVALID_CMD_ERROR_RESPONSE, NO_PAYLOAD_2, NO_PAYLOAD_2);

		}
			break;
	}
	return 0;
}


/**
* @brief Prepare UART Command
*
* @param pm2m_uart_comm: pointer to the uart communication protocol command
* @param uart_cmd: uart command
* @param payload : payload of uart command
* @return success - 0 / error - (-1)
*/
int prepare_uart_command(M2M_UART_COMMN *pm2m_uart_comm, UART_COMMANDS uart_cmd, uint8_t payload_1, uint8_t payload_2, uint8_t payload_3)
{
	msg_counter++;
	//uint8_t packet_data = (START_BYTE + uart_cmd + sizeof(M2M_UART_COMMN) + payload_1 + payload_2 + payload_3 + END_BYTE);
	pm2m_uart_comm->start_flag      = START_BYTE;
	pm2m_uart_comm->cmd             = uart_cmd;
	pm2m_uart_comm->msg_num			= msg_counter;
	pm2m_uart_comm->len             = sizeof(M2M_UART_COMMN);
	pm2m_uart_comm->payload_1       = payload_1;
	pm2m_uart_comm->payload_2       = payload_2;
	pm2m_uart_comm->payload_3       = payload_3;
	//subtract 2 to ignore crc and end byte.
	pm2m_uart_comm->crc             = CRC8((const uint8_t *)pm2m_uart_comm, (pm2m_uart_comm->len-2));
	pm2m_uart_comm->end_flag        = END_BYTE;

	return 0;
}


/**
* @brief Read UART Command
*
* @param  void
* @return void
*/
void read_uart_user_command(void)
{
	//uint8_t var = 0;
	//const uint8_t* pm2m_data = (const void*) pm2m_uart_comm;
	static uint16_t uart_tx_rx_count = 0;
	//uint8_t len = 0;
	uint8_t crc8 = 0;
	//uint8_t packet_data = 0;

	//First check if messages are queued and send if esp is ready.
	if(DeviceInfo.uart_status == ESP_UART_READY)
	{
		send_queued_uart_message();
	}

	//reset pointer to uart command buffer
	memset(pm2m_uart_comm, 0x00, sizeof(M2M_UART_COMMN));

	//use macro for not ready instead of negate flag
	if(DeviceInfo.uart_status == ESP_UART_NOT_READY)
	{
		if((uart_tx_rx_count == UART_CMD_TX_COUNT_RESET) || (uart_tx_rx_count == UART_CMD_TX_MAX_COUNT))
		{
			//read UART status from communication MCU
			send_to_communication_mcu(pm2m_uart_comm, READ_ESP_UART_STATUS, NO_PAYLOAD_1, NO_PAYLOAD_2, NO_PAYLOAD_2);
			if(uart_tx_rx_count == UART_CMD_TX_MAX_COUNT)
			{
				uart_tx_rx_count = UART_CMD_TX_COUNT_RESET;
			}
		}
	}
	else
	{
		//UART communication UP between STM32 and ESP32
		if(DeviceInfo.uart_status == ESP_UART_READY)
		{

			if((uart_tx_rx_count == UART_CMD_TX_COUNT_RESET) || (uart_tx_rx_count == UART_CMD_TX_MAX_COUNT))
			{
				{
					send_to_communication_mcu(pm2m_uart_comm, READ_COMMUNICATION_MCU_CMD, NO_PAYLOAD_1, NO_PAYLOAD_2, NO_PAYLOAD_2);
				}
			}
			if(uart_tx_rx_count == UART_CMD_TX_MAX_COUNT)
			{
				uart_tx_rx_count = UART_CMD_TX_COUNT_RESET;
			}
		}
	}

	//reset the M2M packet buffer
	memset(pm2m_uart_comm, 0x00, sizeof(M2M_UART_COMMN));

	//check UART for any M2M packet is received
	read_from_communication_mcu((uint8_t *)pm2m_uart_comm);

	//validate the M2M packet with start and end byte
	if((pm2m_uart_comm->start_flag == START_BYTE) && (pm2m_uart_comm->end_flag == END_BYTE))
	{
		static uint8_t last_msg_num = 0xFF;
		//need to add  payload 2 when led and blinking rate code support is added
		//packet_data = (pm2m_uart_comm->start_flag + pm2m_uart_comm->cmd + pm2m_uart_comm->len + pm2m_uart_comm->payload_1 + pm2m_uart_comm->end_flag);
		crc8 = CRC8((const uint8_t *)pm2m_uart_comm, (pm2m_uart_comm->len - 2));

		//Nitin -- commented to debug the blink rate crc getting failed
//		printf("\ncrc8:%x\n",crc8);
//		printf("\npm2m_uart_comm->crc:%x\n",pm2m_uart_comm->crc);
//        //crc check
        if(crc8 != pm2m_uart_comm->crc)
        {
			//send uart command for crc error response
			send_to_communication_mcu(pm2m_uart_comm, ERROR_RESPONSE, CRC_ERROR_RESPONSE, NO_PAYLOAD_2, NO_PAYLOAD_2);
			return;
        }

		//process uart cmd only if it is not a duplicate. (or it is an ack itself)
        if((last_msg_num != pm2m_uart_comm->msg_num) || (pm2m_uart_comm->cmd == PROTECTION_MCU_ACK))
        {
        	process_uart_packet(pm2m_uart_comm);
        }
        else
        {
        	//otherwise just send an ack
        	send_uart_ack(pm2m_uart_comm);
        }

		last_msg_num = pm2m_uart_comm->msg_num;

	}
	if(!a_m2m_message_is_queued() || (DeviceInfo.uart_status == ESP_UART_NOT_READY))
	{
		++uart_tx_rx_count;
	}

	//check if LED blinking flag for provision is initiated
	if (rgb_led_blink.start_blinking_leds_flag)
	{
		//process RGB LED state during provisioning and SBLCP LED control
		BlinkingLEDStateMachine();
	}
	else
	{
		if(led_get_key_status() == KEY_TAKEN_BY_PROVISIONING)
		{
			//Execute LED Turn Off request
			led_turn_off();
			//reset LED states to stop operation and blinking
			memset(&rgb_led_blink, 0x00, sizeof(st_rgb_led_blink));
			led_reset_key_status();
		}
	}
	if(DeviceInfo.uart_status == ESP_UART_READY && !a_m2m_message_is_queued())
	{
		update_breaker_status();
	}
}

/**
  * @brief Send data to communication UART
  * @param const uint8_t * - CMD+Data to be sent to comm MCU
  * @retval void
  */
void send_to_communication_mcu(M2M_UART_COMMN *pm2m_uart_comm, UART_COMMANDS uart_cmd, uint8_t payload_1, uint8_t payload_2, uint8_t payload_3)
{
	uint8_t len = 0;
	const uint8_t *pm2m_data = (const void*)pm2m_uart_comm;

	//prepare uart command with paylaod
	prepare_uart_command(pm2m_uart_comm, uart_cmd, payload_1, payload_2, payload_3);

	//send uart command
	for (len = (sizeof(M2M_UART_COMMN)); len > 0; --len, ++pm2m_data)
	{
		LL_USART_TransmitData8(USART2, *pm2m_data);
		//printf("%x->", *pm2m_data);
		//need to check below call as this might cause deadlock
		while (!LL_USART_IsActiveFlag_TXE(USART2)) {}
	}
}

void queue_uart_message(M2M_UART_COMMN *pm2m_uart_comm, UART_COMMANDS uart_cmd, uint8_t payload_1, uint8_t payload_2, uint8_t payload_3)
{
	prepare_uart_command(pm2m_uart_comm, uart_cmd, payload_1, payload_2, payload_3);

	memcpy(&m2m_queue.message[m2m_queue.to_queue_index],pm2m_uart_comm,sizeof(M2M_UART_COMMN));
	m2m_queue.to_queue_index++;
	if(m2m_queue.to_queue_index >= MAX_UART_QUEUE_SIZE)
	{
		m2m_queue.to_queue_index = 0;
	}
	if(m2m_queue.to_queue_index == m2m_queue.last_sent_index)
	{
		m2m_queue.last_sent_index++;
		if(m2m_queue.last_sent_index >= MAX_UART_QUEUE_SIZE)
		{
			m2m_queue.last_sent_index = 0;
		}
	}

	//if the uart is ready just send now. otherwise leave queued for later send.
	if(DeviceInfo.uart_status == ESP_UART_READY)
	{
		send_queued_uart_message();
	}
}

void send_queued_uart_message()
{
	//verify a message is queued.
	if(m2m_queue.wait_for_ack > 0)
	{
		m2m_queue.wait_for_ack--;
		return;
	}

	if(m2m_queue.to_queue_index != m2m_queue.last_sent_index)
	{
		uint8_t len = 0;
		const uint8_t* pm2m_data = (const void*) pm2m_uart_comm;

		memcpy(pm2m_uart_comm,&m2m_queue.message[m2m_queue.last_sent_index],sizeof(M2M_UART_COMMN));

		for (len = (sizeof(M2M_UART_COMMN)); len > 0; --len, ++pm2m_data)
		{
			LL_USART_TransmitData8(USART2, *pm2m_data);
			//printf("%x->", *pm2m_data);
			//need to check below call as this might cause deadlock
			while (!LL_USART_IsActiveFlag_TXE(USART2)) {}
		}

		m2m_queue.wait_for_ack = TIME_TO_WAIT_FOR_ACK;

		if(m2m_queue.number_of_retries > 0)
		{
			m2m_queue.number_of_retries--;
		}
		else
		{
			//out of retires so give up and go to the next one.
			m2m_queue.number_of_retries = MAX_NUMBER_OF_RETRIES;
			m2m_queue.last_sent_index++;
			if(m2m_queue.last_sent_index >= MAX_UART_QUEUE_SIZE)
			{
				m2m_queue.last_sent_index = 0;
			}
		}
	}
}

bool a_m2m_message_is_queued()
{
	if(m2m_queue.to_queue_index != m2m_queue.last_sent_index)
	{
		return TRUE;
	}
	return FALSE;
}

/**
  * @brief Read data to communication UART
  * @param const uint8_t * - CMD+Data to be read to comm MCU
  * @retval void
  */
void read_from_communication_mcu(uint8_t *pm2m_data)
{
	//use static variable for interrupt context switch,generic comment for UART read variables
	uint8_t num_rx = 0;
	uint8_t len = (sizeof(M2M_UART_COMMN));
	uint8_t recv = 0;
	uint8_t m2mdata_found = 0;

	//check if the USART Read Data Register Not Empty Flag is set or not
	if(!LL_USART_IsActiveFlag_RXNE(USART2))
	{
		return;
	}
	while ((len > 0) /*&&
		   LL_USART_IsActiveFlag_RXNE(USART2)*/)
	{
		/* Receive a character (8bit , parity none) */
		 recv = LL_USART_ReceiveData8(USART2);
		 if((recv == START_BYTE) && (!m2mdata_found))
		 {
			 m2mdata_found = 1;
			 len = (sizeof(M2M_UART_COMMN));
		 }
		 if(m2mdata_found)
		 {
			 pm2m_data[num_rx++] = recv;
		 }

		//printf("%x<-,",pm2m_data[num_rx-1]);
		/* Clear overrun error flag */
		if (LL_USART_IsActiveFlag_ORE(USART2))
		{
			LL_USART_ClearFlag_ORE(USART2);
		}
		len--;
	}
}

void update_breaker_status(void)
{
	static u16 call_timer = 0;
	static u8 last_primary_state = SW_UNKNOWN;
	static u8 last_secondary_state = SS_UNKNOWN;
	static u8 last_open_feedback_state = PATH_UNKNOWN;
	//static u8 update_stats_counter = 0;
	//u32 hall_data_adc;
	u32 gf_current;

	call_timer++;

	if( (last_primary_state != (u8)get_primary_switch_debounced_state())
		|| (last_secondary_state != (u8)secondary_solenoid_get_status())
		|| (last_open_feedback_state != (u8)get_path_status())
	  )
	{
		call_timer = 0xFFFF;
	}

	last_primary_state = (u8)get_primary_switch_debounced_state();
	last_secondary_state = (u8)secondary_solenoid_get_status();
	last_open_feedback_state = (u8)get_path_status();

	last_open_feedback_state |= (startup_conf<<4);

	if(call_timer >= UPDATE_BREAKER_STATUS_RATE)
	{
		call_timer = 0;
		send_to_communication_mcu(pm2m_uart_comm,
										  BREAKER_STATUS,
										  last_primary_state,
										  last_secondary_state,
										  last_open_feedback_state);
	}

	/*update_stats_counter++;
	if(update_stats_counter >= 120)
	{
		update_stats_counter = 0;
		//gf_data = (u32)adc_get_channel_data(GF_ADC_CHANNEL);
		gf_current = (total_gf_current / 1000);
		send_to_communication_mcu(pm2m_uart_comm, BREAKER_GF_RAW, (uint8_t)(gf_current >> 16), (uint8_t)(gf_current >> 8), (uint8_t)gf_current);
	}
	else if(update_stats_counter == 90)
	{
		//hall_data_adc = get_primary_switch_raw_adc();
		send_to_communication_mcu(pm2m_uart_comm, BREAKER_HAL_RAW, (uint8_t)(hall_data_adc >> 16), (uint8_t)(hall_data_adc >> 8), (uint8_t)hall_data_adc);
	}*/
}


/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void uart_init_component(void)
{
	//initialize the uart command buffer pointer
	pm2m_uart_comm = (M2M_UART_COMMN *) malloc(sizeof(M2M_UART_COMMN));

	msg_counter = 0;
	m2m_queue.last_sent_index = 0;
	m2m_queue.to_queue_index = 0;
	m2m_queue.wait_for_ack = 0;
	m2m_queue.number_of_retries = MAX_NUMBER_OF_RETRIES;
	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	LL_USART_InitTypeDef USART_InitStruct = {0};

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	//	  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	/**USART2 GPIO Configuration
	PA2   ------> USART2_TX -------> pin 6
	PA3   ------> USART2_RX -------> pin 34
	*/
	GPIO_InitStruct.Pin = USART2_TX_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(USART2_TX_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = USART2_RX_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(USART2_RX_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
	USART_InitStruct.BaudRate = 115200;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART2, &USART_InitStruct);
	LL_USART_SetTXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
	LL_USART_SetRXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
	//LL_USART_DisableFIFO(USART2);
	LL_USART_EnableFIFO(USART2);
	LL_USART_ConfigAsyncMode(USART2);

	/* USER CODE BEGIN WKUPType USART2 */

	/* USER CODE END WKUPType USART2 */

	LL_USART_Enable(USART2);

	/* Polling USART2 initialisation */
	while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
	{
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
  * @brief Update Protection FW Version to Communication
  *        (major.minor.patch versions)
  * @param const uint8_t gStm_Firmware_Version to be read to comm MCU
  * @retval void
  */
void update_protection_fw_version_to_communication(void)
{
	u8* data_p = 0;
	u16 counter = 0;
	uint8_t Uart_Data_Buffer[16] = {0};

	data_p = ((u8*)&__version + sizeof(__version)-1);
    for(counter = 0; counter < sizeof(__version); counter++)
    {
    	Uart_Data_Buffer[counter] = *data_p;
    	data_p--;
    	if((counter == 2) || (counter == 5) || (counter == 7))
    	{
    		if(counter == 7)
    		{
    			counter++;
    		}
    		queue_uart_message(pm2m_uart_comm, BREAKER_PRTOTECTION_FW_VERSION, Uart_Data_Buffer[counter-2], Uart_Data_Buffer[counter-1], Uart_Data_Buffer[counter]);
    	}
    }
}
