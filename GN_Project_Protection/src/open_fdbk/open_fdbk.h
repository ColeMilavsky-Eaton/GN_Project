#ifndef _OPEN_FDBK_H_
#define _OPEN_FDBK_H_
/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2022
 *                      All rights reserved
 *
 **************************************************************************************************
 *  Written by:         James Frances
 *                      Commercial & Residential Distribution Solutions (CRDS), Eaton
 *						1000 Cherrington Parkway
 *						Moon Township, PA 15108
 *						mobile: (724) 759-5500
 *//**
 * @brief interface for the secondary solenoid component.
 *
 * @file open_fdbk_api.h
 * @ingroup open_fdbk
 *
 *//*
 *
 **************************************************************************************************/
#include "open_fdbk_internal.h"

/* for diagnostics */
#define OPEN_FDBK_COMPONENT


#define FDBK1_STATUS		gpio_read_input_pin(OPEN_FDBK_1_GPIO_Port,OPEN_FDBK_1_Pin)
#define FDBK2_STATUS		gpio_read_input_pin(OPEN_FDBK_2_GPIO_Port,OPEN_FDBK_2_Pin)

/*Below defines based on the logic:
* 						(state_fdbk1_first <<3 )
*					 	| (state_fdbk2_first << 2)
*					 	| (state_fdbk1_second << 1)
*					 	| (state_fdbk2_second)
*/
#define PATH_OPEN_1P	1
#define PATH_CLOSED_1P	0


#define PATH_OPEN_2P					0x0F //0b1111
#define PATH_CLOSED_FDBK1_PRIMARY_2P	0x06 //0b0110
#define PATH_CLOSED_FDBK2_PRIMARY_2P	0x09 //0b1001
#define PATH_STUCK1_2P_LOAD				0x03 //0b0011 //The secondary pole is mimicking the primary
#define PATH_STUCK2_2P_LOAD				0x0C //0b1100 //The primary pole is mimicking the secondary
#define PATH_STUCK1_2P					0x07 //0b0111
#define PATH_STUCK2_2P					0x0E //0b1110
#define PATH_STUCK3_2P					0x0D //0b1101
#define PATH_STUCK4_2P					0x0B //0b1011


typedef struct
{
	//state of the open feedback during the first peak (it = 8)
	bool state_fdbk1_first;
	bool state_fdbk2_first;
	//state of the open feedback during the second peak (it = 19, 24, or 29)
	bool state_fdbk1_second;
	bool state_fdbk2_second;
	fdbk_status_t status;

} path_status_t;

static path_status_t path;
/**************************************************************************************************/
/**
 * @brief open feedback task.
 *
 * checks the status of the open feedback circuit.
 *
 * @return none.
 *
 * @exception none
 *
 **************************************************************************************************/
void open_fdbk_it_task(u32 it);

/**************************************************************************************************/
/**
 * @brief open feedback status updating task for 2 pole.
 *
 * checks the combination of feedback high/low status and makes a determination on the status
 * of the path as a whole. should not be used to directly determine state of the secondary contact.
 *
 * @return none.
 *
 * @exception none
 *
 **************************************************************************************************/
void set_path_status_2poles(void);

/**************************************************************************************************/
/**
 * @brief function to blink load feedback led.
 *
 * blinks the load feedback LED based on number of times called.
 *
 * @return none.
 *
 * @exception none
 *
 **************************************************************************************************/
void blink_load_powered_led(void);

#endif /* _OPEN_FDBK_H_ */
