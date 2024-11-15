#ifndef SRC_USART_USART_H_
#define SRC_USART_USART_H_

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
 * @file usart.h
 * @ingroup usart
 *
 *//*
 *
 **************************************************************************************************/

#include "types.h"
//#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_usart.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx.h"
#include "main_internal.h"

#include "led_api.h"
#include "btn_api.h"
#include "primary_switch_api.h"
#include "secondary_solenoid_api.h"
#include "open_fdbk_api.h"

#define MAX_UART_QUEUE_SIZE 10
#define TIME_TO_WAIT_FOR_ACK 5
#define MAX_NUMBER_OF_RETRIES 5

//#include "stm32g0xx_ll_exti.h"
#define MCO_Pin LL_GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define USART2_TX_Pin LL_GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin LL_GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define LED_GREEN_Pin LL_GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOA
#define TMS_Pin LL_GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin LL_GPIO_PIN_14
#define TCK_GPIO_Port GPIOA

#define START_BYTE   0x7e
#define END_BYTE     0x7f

#define NO_PAYLOAD_2 0x00

#define NO_PAYLOAD_1 0x00
#define ESP_UART_READY 0x01
#define ESP_UART_NOT_READY 0x00
#define UART_CMD_TX_MAX_COUNT 500
#define UART_CMD_TX_COUNT_RESET 0x00

#define POSITIVE_ACK 	0x01
#define NEGATIVE_ACK	0xFE

typedef enum{
	RET_SUCCESS = 0,
	RET_ERROR   = -1,
}RETURN_STATUS;

typedef struct {
	uint8_t start_flag; //HEX start of packet end of packet flag
	uint8_t cmd;        //handler side what to do.
	uint8_t msg_num;	// Counter to ensure ack message is same as sent message.
	uint8_t len;        // payload length
	uint8_t payload_1;   //command data param 1
	uint8_t payload_2;   //command data param 2
	uint8_t payload_3;   //command data param 3
	uint8_t crc;        //crc function to implement for data validation
					 // (8/16 bitcrc check timing, for max bytes data)
	uint8_t end_flag;   //end of frame flag
}M2M_UART_COMMN;

typedef struct {
	M2M_UART_COMMN message[MAX_UART_QUEUE_SIZE];
	uint8_t to_queue_index;
	uint8_t last_sent_index;
	uint8_t wait_for_ack;
	uint8_t number_of_retries;
}M2M_UART_QUEUE;

typedef struct
{
	u16 blinking_counter;
	u16 blinking_time_interval;
	u16 blinking_time_expire;
	//u8 blinking_counter;
	//u8 blinking_time_interval;
	//u8 blinking_time_expire;
	u8 start_blinking_leds_flag;
    led_color_t rgb_led_color;
} st_rgb_led_blink;

typedef enum
{
    UnProvisioned =   0,
    ProvisionStarted,
    Provisioned,
}Provision_State_enum;


typedef struct{
	uint8_t uart_status;
	uint8_t provision_status;//rename according to button function
	uint8_t button_2_status;
}Device_Info;

typedef enum {
	//review
	//button events needs to be updated and also
	//communicated
	//LED Pattern : color, rate and duration of LED blinking pattern (Wifi status, provision)
	// when to start and stop the LED based on events
	BREAKER_FAULT_STATE       =    0x20,
	BREAKER_OPEN_CLOSE,
	COMM_MCU_ACK,
	PROTECTION_MCU_ACK,
	READ_ESP_UART_STATUS,
	BUTTON_PRESS_1_STATUS,
	SWITCH2_PRESS_PROVISION,//rename as per button funcitonality
	READ_COMMUNICATION_MCU_CMD,
	READ_WIFI_STATUS,//read wifi status connected to AP or open to connect or no AP found or no internet on AP
	READ_ESP_STATUS, //factory reset or first boot up
	LED_COLOR_BLINK_RATE,
	STM32_OTA_MODE,
	STM32_OTA_START,
	STM32_OTA_DATA,
	STM32_OTA_END,
	BREAKER_STATUS,
	IDENTIFY_ME,
	UPDATE_TEMPERATURE,
  SET_STARTUP_CONFIG,
	BREAKER_GF_RAW,
	BREAKER_HAL_RAW,
	BREAKER_PRTOTECTION_FW_VERSION,
	ESP_FACTORY_RESET = 0xFE,
	ERROR_RESPONSE            =    0xFF,
}UART_COMMANDS;

typedef enum {
	SECONDARY_CONTACTS_OPEN = 0x01,
	SECONDARY_CONTACTS_CLOSE,
	SECONDARY_CONTACTS_TOGGLE
}BREAKER_SECONDARY_CONTACT_STATE;

typedef enum {
	SWITCH_1_PRESS = 0x01,//change name according to button functionality
	BREAKER_PROVISION
}SWITCH_PRESS;

typedef enum{
	BREAKER_PROVISION_START = 0x61,

}WIFI_STATUS;

typedef enum{
	FACTORY_RESET = 0x01,
	STANDARD_RESET = 0x02,

}RESET_COMMAND_TYPE;

typedef enum {
	INVALID_CMD_ERROR_RESPONSE = 0x51,
	CRC_ERROR_RESPONSE,
	WIFI_CONNECT_AP_ERROR_RESPONSE, //for LED pattern

}ERROR_RESPONSE_TYPE;

int prepare_uart_command(M2M_UART_COMMN *pm2m_uart_comm, UART_COMMANDS uart_cmd, uint8_t payload_1, uint8_t payload_2, uint8_t payload_3);
unsigned char CRC8(const unsigned char *data,int length);
void send_to_communication_mcu(M2M_UART_COMMN *pm2m_uart_comm, UART_COMMANDS uart_cmd, uint8_t payload_1, uint8_t payload_2, uint8_t payload_3);
void read_from_communication_mcu(uint8_t *pm2m_data);
void uart_init_component(void);
void read_uart_user_command(void);
int process_open_close(M2M_UART_COMMN *pm2m_uart_comm);
int process_wifi_status(M2M_UART_COMMN *pm2m_uart_comm);
int process_cmd_from_communication_mcu(M2M_UART_COMMN *pm2m_uart_comm);
int process_led_blink_rate(M2M_UART_COMMN *pm2m_uart_comm);
void process_identify_me(M2M_UART_COMMN *pm2m_uart_comm);
void process_startup_config(M2M_UART_COMMN *pm2m_uart_comm);

void update_breaker_status(void);
void process_uart_ack(M2M_UART_COMMN *pm2m_uart_comm);
void send_uart_ack(M2M_UART_COMMN *pm2m_uart_comm);

void queue_uart_message(M2M_UART_COMMN *pm2m_uart_comm, UART_COMMANDS uart_cmd, uint8_t payload_1, uint8_t payload_2, uint8_t payload_3);
void send_queued_uart_message(void);
bool a_m2m_message_is_queued(void);
void update_protection_fw_version_to_communication(void);

#endif /* SRC_USART_USART_H_ */
