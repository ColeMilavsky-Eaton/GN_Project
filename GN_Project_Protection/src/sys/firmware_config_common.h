#ifndef SRC_SYS_FIRMWARE_CONFIG_COMMON_H_
#define SRC_SYS_FIRMWARE_CONFIG_COMMON_H_
/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2022
 *                      All rights reserved
 *
 ***************************************************************************************************
 *  Written by:         Aaron Joseph
 *                      Eaton Electrical
 *                      1000 Cherrington Parkway
 *                      Pittsburgh, PA  15108-4312
 *                      (412) 893-3300
 *//**
 *
 * @brief definition of firmware configuration options
 *
 * @file firmware_config_common.h
 * @ingroup firmware_config
 *
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "utils.h"

/** @name configuration_types this is used to guarantee unique values for various configuration settings.
 * @{ */

#define GF_CONFIG_TYPE                      0x0100
#define GN_CONFIG_TYPE                      0x0200
#define AFCI_CONFIG_TYPE                    0x0300
#define TRIP_CONFIG_TYPE                    0x0400
#define OVERLOAD_CONFIG_TYPE                0x0500
#define AUTO_MONITOR_CONFIG_TYPE            0x0600
#define BUTTON_LED_CONFIG_TYPE              0x0700
#define NO_POWER_TRIP_CIRCUIT_CONFIG_TYPE   0x0800
#define REGULATE_PIN_CONFIG_TYPE   			0x0800
#define SOLENOID_CONFIG_TYPE                0x0900
#define BOARD_CONFIG_TYPE                   0x0A00
#define AUTO_CONFIG_CAL_CONFIG_TYPE         0x0B00
#define NUM_POLES_CONFIGURATION_TYPE        0x0C00
#define SOFTWARE_ZCD_CONFIGURATION_TYPE     0x0D00
#define LED_CONFIG_TYPE                     0x0F00

/** @} */


/** @name configuration_options values given to individual configuration options within a single configuration type
 * @{ */

#define CONFIG_NONE     0
#define CONFIG_1    BIT_0
#define CONFIG_2    BIT_1
#define CONFIG_3    BIT_2
#define CONFIG_4    BIT_3
#define CONFIG_5    BIT_4
#define CONFIG_6    BIT_5
#define CONFIG_7    BIT_6
#define CONFIG_8    BIT_7

/** @} */


/** @name gf_configurations ground fault configuration options
 * @{ */

#define GF_NOT_ENABLED             (GF_CONFIG_TYPE + CONFIG_NONE)
#define GF_CONFIG_FIXED_TIMING      (GF_CONFIG_TYPE + CONFIG_1)
#define GF_CONFIG_INVERSE_TIMING    (GF_CONFIG_TYPE + CONFIG_2)

/** @} */


/** @name gn_configurations grounded neutral configuration options
 * @{ */

#define GN_NOT_ENABLED     (GN_CONFIG_TYPE + CONFIG_NONE)
#define GN_CONFIG_1_CT      (GN_CONFIG_TYPE + CONFIG_1)
#define GN_CONFIG_2_CT      (GN_CONFIG_TYPE + CONFIG_2)

/** @} */


/** @name af_configurations arc fault configuration options
 * @{ */

#define AF_NOT_ENABLED     (AFCI_CONFIG_TYPE + CONFIG_NONE)
#define AF_WITHOUT_LED_FIX  (AFCI_CONFIG_TYPE + CONFIG_1)
#define AF_WITH_LED_FIX     (AFCI_CONFIG_TYPE + CONFIG_2)

/** @} */


/** @name trip_configurations trip configuration options
 * @{ */

#define TRIP_CONFIG_INDIVIDUAL_FIRING (TRIP_CONFIG_TYPE + CONFIG_1)
#define TRIP_CONFIG_COMBINED_FIRING   (TRIP_CONFIG_TYPE + CONFIG_2)

/** @} */


/** @name overload_configurations electronic overload configurations configuration options
 * @{ */

#define OVERLOAD_NOT_ENABLED   (OVERLOAD_CONFIG_TYPE + CONFIG_NONE)
#define OVERLOAD_ENABLED        (OVERLOAD_CONFIG_TYPE + CONFIG_1)

/** @} */


/** @name auto_monitor configuration options
 * @{ */

#define AUTO_MONITOR_NOT_ENABLED   (AUTO_MONITOR_CONFIG_TYPE + CONFIG_NONE)
#define AUTO_MONITOR_ENABLED        (AUTO_MONITOR_CONFIG_TYPE + CONFIG_1)

/** @} */


/** @name button_led configuration options
 * @{ */

#define BUTTON_LED_SHARED_PIN           (AUTO_MONITOR_CONFIG_TYPE + CONFIG_1)
#define BUTTON_LED_NOT_SHARED_PIN       (AUTO_MONITOR_CONFIG_TYPE + CONFIG_2)

/** @} */


/** @name no_power_trip_ciruit configuration options
 * @{ */

#define NO_POWER_TRIP_CIRCUIT_NOT_ENABLED  (NO_POWER_TRIP_CIRCUIT_CONFIG_TYPE + CONFIG_NONE)
#define NO_POWER_TRIP_CIRCUIT_ENABLED       (NO_POWER_TRIP_CIRCUIT_CONFIG_TYPE + CONFIG_1)

/** @} */


/** @name power_supply_regulation_pin configuration options
 * @{ */

#define REGULATE_PIN_NOT_ENABLED   (REGULATE_PIN_CONFIG_TYPE + CONFIG_NONE)
#define REGULATE_PIN_ENABLED       (REGULATE_PIN_CONFIG_TYPE + CONFIG_1)

/** @} */


/** @name solenoid configuration options
 * @{ */

#define SOLENOID_SINGLE                    (SOLENOID_CONFIG_TYPE + CONFIG_1)
#define SOLENOID_DUAL                      (SOLENOID_CONFIG_TYPE + CONFIG_2)

/** @} */

/** @name board configuration options
 * @{ */

#define BOARD_CH                       (BOARD_CONFIG_TYPE + CONFIG_1)
#define BOARD_BR                       (BOARD_CONFIG_TYPE + CONFIG_2)
#define BOARD_SB2 					   (BOARD_CONFIG_TYPE + CONFIG_3)

/** @} */


/** @name rating shunt calibration configuration options
 * @{ */

#define RATING_SHUNT_CAL_NOT_ENABLED             (AUTO_CONFIG_CAL_CONFIG_TYPE + CONFIG_NONE)
#define RATING_SHUNT_CAL_15_20A                   (AUTO_CONFIG_CAL_CONFIG_TYPE + CONFIG_1)
#define RATING_SHUNT_CAL_15_20_25_30A             (AUTO_CONFIG_CAL_CONFIG_TYPE + CONFIG_2)
#define RATING_SHUNT_CAL_15_20_30A                (AUTO_CONFIG_CAL_CONFIG_TYPE + CONFIG_3)

/** @} */


/** @name number of poles configuration options
 * @{ */

#define SINGLE_POLE                 (NUM_POLES_CONFIGURATION_TYPE + CONFIG_1)
#define TWO_POLE                    (NUM_POLES_CONFIGURATION_TYPE + CONFIG_2)
#define THREE_POLE                  (NUM_POLES_CONFIGURATION_TYPE + CONFIG_3)

/** @} */

/** @name software zcd configuration options
 * @{ */

#define SOFTWARE_ZCD_NOT_ENABLED            (SOFTWARE_ZCD_CONFIGURATION_TYPE + CONFIG_NONE)
#define SOFTWARE_ZCD_ENABLED                (SOFTWARE_ZCD_CONFIGURATION_TYPE + CONFIG_1)

/** @} */

/** @name led configuration
 * @{ */

#define SINGLE_RED_LED                  (LED_CONFIG_TYPE + CONFIG_1)
#define RGB_LED                         (LED_CONFIG_TYPE + CONFIG_2)

/** @} */

#endif /* FIRMWARE_CONFIG_COMMON_H_ */




