#ifndef _STM32G0xx_IT_API_H
#define _STM32G0xx_IT_API_H
/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 **************************************************************************************************
 *  Written by:         Aaron Joseph
 *                      Eaton Electrical
 *                      1000 Cherrington Parkway
 *                      Pittsburgh, PA  15108-4312
 *                      (412) 893-3300
 *//**
 * @brief External interface for the interrupt component.
 *
 * @file stm32g0xx_it_api.h
 * @ingroup interrupt
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"


/** primary zero crossing enumeration */
typedef enum
{
    PRIMARY_ZCD_NONE = 0,
    PRIMARY_ZCD_1,
    PRIMARY_ZCD_2,

} primary_zcd_t;

typedef enum
{
    SCENARIO_NOT_DEFINED            = 0,
    SCENARIO_1P                     = 1,
    SCENARIO_2P_PRIMARY_LEAD_180    = 2,
    SCENARIO_2P_PRIMARY_LEAD_120    = 3,
	SCENARIO_2P_PRIMARY_LEAD_240    = 4,
    SCENARIO_2P_ONLY_1P_CONNECTED   = 5,
	SCENARIO_2P_PRIMARY_SAME_PHASE  = 6,
    SCENARIO_3P                     = 7,
	SCENARIO_UNEXPECTED_PHASE     = 0xFE,

}input_voltage_scenario_t;

extern primary_zcd_t primary_zcd;
extern input_voltage_scenario_t input_voltage_scenario;


//NOTE SB2.0 P1 reverses the load_voltage when converting to 3.3. So negative peak is seen as positive.
//#define PRIMARY_PEAK_MIN	6
//#define PRIMARY_PEAK_MAX	10

//#define LEAD_120_PEAK_MIN	17
//#define LEAD_120_PEAK_MAX	21

//#define LEAD_180_PEAK_MIN	22
//#define LEAD_180_PEAK_MAX	26

//#define LEAD_240_PEAK_MIN	27
//#define LEAD_240_PEAK_MAX	31

//Using inverted load voltage detection.
#define PRIMARY_PEAK_MIN	22
#define PRIMARY_PEAK_MAX	26

#define LEAD_120_PEAK_MIN	1
#define LEAD_120_PEAK_MAX	5

#define LEAD_180_PEAK_MIN	6
#define LEAD_180_PEAK_MAX	10

#define LEAD_240_PEAK_MIN	11
#define LEAD_240_PEAK_MAX	15
/**************************************************************************************************/
/**
 * @brief get interrupt count
 *
 * return the current interrupt count
 *
 * @return interrupt count
 *
 * @requirement{xxx}
 * @requirement{xxx}
 *
 **************************************************************************************************/
u16 it_get_interrupt_count(void);

/**************************************************************************************************/
/**
 * @brief Return the input voltage scenario
 *
 *  @return  SCENARIO_NOT_DEFINED
 *          SCENARIO_1P
 *          SCENARIO_2P_PRIMARY_LEAD_180
 *          SCENARIO_2P_PRIMARY_LEAD_120
 *          SCENARIO_2P_PRIMARY_LEAD_240
 *          SCENARIO_2P_ONLY_1P_CONNECTED
 *          SCENARIO_3P
 *
 * @return none
 *
 *
 **************************************************************************************************/
input_voltage_scenario_t it_get_input_voltage_scenario(void);


/**************************************************************************************************/
/**
 * @brief Determines input voltage scenario
 *
 * This function determines input voltage scenario. This function compares the input voltage
 * at a specific time to determine the relationship of the input voltages.
 *
 * Due to the asymmetric ZCDs in two pole, this function has to be called on interrupt count 0
 * (right after zcd) and after all the adc data on interrupt count 0 has been collected.
 *
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void determine_input_scenario(u32 interrupt_count);


/**************************************************************************************************/
/**
 * @brief get previous half cycle interrupt count
 *
 * returns the interrupt count from the previous half cycle
 *
 * @return  previous_half_cycle_interrupt_cnt
 *
 * @requirement{xxx}
 * @requirement{xxx}
 *
 **************************************************************************************************/
inline u16 it_get_previous_half_cycle_interrupt_count(void);


/**************************************************************************************************/
/**
 * @brief Initialize and setup external interrupt
 *
 * Note that rising and falling flags are cleared at startup up due to the ZCD lines being pulled
 * up that results a false trigger.
 *
 *
 * @return none
 *
 *
 **************************************************************************************************/
void it_init_nvic(void);


/**************************************************************************************************/
/**
 * @brief initialize interrupt component
 *
 * Initialize all variables needed for the interrupt component
 *
 * @return none
 *
 *
 **************************************************************************************************/
void interrupt_init_component(void);

/**************************************************************************************************/
/**
 * @brief Return primary ZCD
 *
 * Return primary ZCD.
 *
 * @return none
 *
 *
 **************************************************************************************************/
primary_zcd_t it_get_primary_zcd(void);

/**************************************************************************************************/
/**
 * @brief return bias in range flag
 *
 * Return if bias in range flag.
 *
 * @return TRUE if in range, FALSE if not.
 *
 *
 **************************************************************************************************/
bool it_get_gf_bias_in_range_flag(void);

/**************************************************************************************************/
/**
 * @brief return frequency of the power
 *
 * Return the frequencyg.
 *
 * @return 1000 times the frequency in integer.
 *
 *
 **************************************************************************************************/
u32 it_get_measured_frequency(void);

#endif /* _STM32G0xx_IT_API_H */
