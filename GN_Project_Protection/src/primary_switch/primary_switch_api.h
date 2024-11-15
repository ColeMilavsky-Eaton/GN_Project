#ifndef _PRIMARY_SWITCH_API_H_
#define _PRIMARY_SWITCH_API_H_
/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2023
 *                      All rights reserved
 *
 **************************************************************************************************
 *  Written by:         Ming Wu
 *                      Innovation
 *                      Commercial & Residential Distribution Solutions (CRDS), Eaton
 *                      1000 Cherrington Parkway
 *                      Moon Township, PA 15108
 *//**
 * @brief API for the primary switch monitoring component.
 *
 * @file primary_switch_api.h
 * @ingroup primary_switch
 *
 *//*
 *
 **************************************************************************************************/


#define PSW_DEBOUNCE_TIMER_COUNT            3
#define PSW_DEBOUNCE_INST_COUNT				10 //JF TODO determine value based on how long hardware actually takes
#define PSW_DEBOUNCE_INST_TIMEOUT			20

typedef enum
{
    SW_UNKNOWN = 0,
    SW_OPEN,
    SW_TRIP,
    SW_CLOSED,
    SW_FAILED,
} pswitch_status_t;


/**************************************************************************************************/
/**
 * @brief Initialize primary_switch component.
 *
 * This function initializes the primary_switch component and add task to task manager.
 *
 * @return none.
 *
 * @exception none
 *
 **************************************************************************************************/
void primary_switch_init(void);

/**************************************************************************************************/
/**
 * @brief Returns the current debounced state of the primary switch
 *
 * This function will return the last debounced status of the primary switch.
 *
 * @return pswitch_state_t.
 *
 * @exception none
 *
 **************************************************************************************************/
pswitch_status_t get_primary_switch_debounced_state(void);
u32 get_primary_switch_raw_adc(void);

/**************************************************************************************************/
/**
 * @brief Returns the current status of the primary switch after continually reading.
 *
 * This function will return the current status of the primary switch after directly
 * debouncing the signal. This function is blocking and should be avoided when timing is critical.
 *
 * @return pswitch_status_t.
 *
 * @exception none
 *
 **************************************************************************************************/
pswitch_status_t get_primary_switch_direct_debounced_status(void);

#endif /* PRIMARY_SWITCH_API_H_ */
