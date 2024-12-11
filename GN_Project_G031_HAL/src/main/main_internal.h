#ifndef _MAIN_INTERNAL_H
#define _MAIN_INTERNAL_H
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
 * @brief internal (to CRDS breaker firmware) interface for the main component.
 *
 * @file main_internal.h
 * @ingroup main
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "utils.h"
#include "stm32g0xx_ll_adc.h"
#include "stm32g031xx.h"
//#include "stm32g0xx.h"
#include "stm32g0xx_hal.h"
#include "stm32g0xx_hal_conf.h"
#include "stm32g0xx_hal_spi.h"
#include "stm32g0xx_ll_iwdg.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_usart.h"

#if BOARD_CONFIGURATION != BOARD_SB2  //Smart Breaker boards do not contain spi interface
#include "stm32g0xx_ll_spi.h"
#endif
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_gpio.h"
#include "main_api.h"

#include "../../src/sys/hardware.pin_config"


#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

#define FLASH_BREAKER_CONFIG_START_ADDRESS 				(0x08019000)
#define FLASH_BREAKER_CONFIG_RATING_OFFSET              (0x10)
#define FLASH_BREAKER_CONFIG_CALIBRATION_OFFSET         (0x20)
#define FLASH_BREAKER_CONFIG_RATING_ADDRESS             (FLASH_BREAKER_CONFIG_START_ADDRESS + FLASH_BREAKER_CONFIG_RATING_OFFSET)
#define FLASH_BREAKER_CONFIG_CALIBRATION_ADDRESS        (FLASH_BREAKER_CONFIG_START_ADDRESS + FLASH_BREAKER_CONFIG_CALIBRATION_OFFSET)
#define FLASH_BREAKER_CONFIG_END_ADDRESS                (0x080197FF)
#define FLASH_BREAKER_CONFIG_CHECKSUM_BYTES             (6)
#define FLASH_BREAKER_CONFIG_HANDLE_RATING_BYTES        (2)
#define FLASH_BREAKER_CONFIG_SHUNT_CALI_BYTES           (2)
#define FLASH_BREAKER_CONFIG_BYTES                      (FLASH_BREAKER_CONFIG_CHECKSUM_BYTES\
                                                         +FLASH_BREAKER_CONFIG_HANDLE_RATING_BYTES\
                                                         +FLASH_BREAKER_CONFIG_SHUNT_CALI_BYTES)


/** @name Trip_code_defines
 * @{ */



/* trip codes */
#define NO_TRIP                                                (0)
#define NORMAL_STARTUP                                         (1)
#define SELF_TEST_SUCCESSFUL_TRIP                              (2)
#define OUTPUT_STATUS_LOG_VIA_SPI                              (3)
#define END_OF_FACTORY_ANCHOR                                  (4)
#define RECALL_RESET_ANCHOR                                    (5)

#define SB2_DIAGNOSTIC_BEGIN     							   (8)
#define END_SB2_DIAGNOSTIC							   		   (31)

#define PROTECTION_FAULT_CODES_BEGIN                           (32)
#define GENERAL_ARC_DETECTION_LOW_CURRENT_TRIP                 (PROTECTION_FAULT_CODES_BEGIN+0)
#define GENERAL_ARC_DETECTION_HIGH_CURRENT_TRIP                (PROTECTION_FAULT_CODES_BEGIN+1)
#define DIMMER_ARC_DETECTION_LOW_CURRENT_TRIP                  (PROTECTION_FAULT_CODES_BEGIN+2)
#define DIMMER_ARC_DETECTION_HIGH_CURRENT_TRIP                 (PROTECTION_FAULT_CODES_BEGIN+3)
#define GROUND_FAULT_OVERCURRENT_TRIP                          (PROTECTION_FAULT_CODES_BEGIN+4)
#define PARALLEL_ARC_FAULT_TRIP                                (PROTECTION_FAULT_CODES_BEGIN+5)
#define SHORT_DELAY_FAULT_OVERCURRENT_TRIP                     (PROTECTION_FAULT_CODES_BEGIN+6)
#define OVERVOLTAGE_TRIP                                       (PROTECTION_FAULT_CODES_BEGIN+7)
#define HRGF_COLDSTART_TRIP                                    (PROTECTION_FAULT_CODES_BEGIN+8)
#define HRGF_RUNNING_TRIP                                      (PROTECTION_FAULT_CODES_BEGIN+9)
#define GN_TRIP                                                (PROTECTION_FAULT_CODES_BEGIN+10)
#define OVERLOAD_TRIP                                          (PROTECTION_FAULT_CODES_BEGIN+11)
#define HIGH_GN_TRIP                                           (PROTECTION_FAULT_CODES_BEGIN+12)

#define PRIMARY_HAL_TRIP_OR_OFF							   	   (PROTECTION_FAULT_CODES_BEGIN+31)
#define PROTECTION_FAULT_CODES_END                             (PRIMARY_HAL_TRIP_OR_OFF)

#define CONTINUOUS_HW_FAULT_CODES_BEGIN                        (64)
#define FAILED_STARTUP_RAM_TEST                                (CONTINUOUS_HW_FAULT_CODES_BEGIN+0)
#define FAILED_STARTUP_ROM_TEST                                (CONTINUOUS_HW_FAULT_CODES_BEGIN+1)
#define FAILED_STARTUP_PROCESSOR_TEST                          (CONTINUOUS_HW_FAULT_CODES_BEGIN+2)
#define DATA_OVERRUN_TRIP                                      (CONTINUOUS_HW_FAULT_CODES_BEGIN+3)
#define INVALID_DATA_SEQUENCE_TRIP                             (CONTINUOUS_HW_FAULT_CODES_BEGIN+4)
#define INCORRECT_INTERRUPT_COUNT_TRIP                         (CONTINUOUS_HW_FAULT_CODES_BEGIN+5)
#define HF_SENSE_FAULT_TRIP                                    (CONTINUOUS_HW_FAULT_CODES_BEGIN+6)
#define LINE_CURRENT_BIAS_ERROR_TRIP                           (CONTINUOUS_HW_FAULT_CODES_BEGIN+7)
#define GROUND_FAULT_CURRENT_BIAS_ERROR_TRIP                   (CONTINUOUS_HW_FAULT_CODES_BEGIN+8)
#define LOG_HF_MIN_DETECTOR_STUCK_ERROR_TRIP                   (CONTINUOUS_HW_FAULT_CODES_BEGIN+9)
#define FAILED_CONTINUOUS_RAM_TEST_TRIP                        (CONTINUOUS_HW_FAULT_CODES_BEGIN+10)
#define FAILED_CONTINUOUS_ROM_TEST_TRIP                        (CONTINUOUS_HW_FAULT_CODES_BEGIN+11)
#define FAILED_CONTINUOUS_PROCESSOR_TEST_TRIP                  (CONTINUOUS_HW_FAULT_CODES_BEGIN+12)
#define NONHANDLED_INTERRUPT_TRIP                              (CONTINUOUS_HW_FAULT_CODES_BEGIN+13)
#define FAILED_TRIP_ATTEMPT                                    (CONTINUOUS_HW_FAULT_CODES_BEGIN+14)
//GF Self Check --Start--
#define FAILED_SELF_CHECK_GF_INPUT                             (CONTINUOUS_HW_FAULT_CODES_BEGIN+15)
#define FAILED_SELF_CHECK_OUTPUT                               (CONTINUOUS_HW_FAULT_CODES_BEGIN+16)
#define FAILED_SELF_CHECK_CT_DIRECTION                         (CONTINUOUS_HW_FAULT_CODES_BEGIN+17)
//GF Self Check --End--
#define COMPLETE_LOSS_OF_ZCD_TRIP                              (CONTINUOUS_HW_FAULT_CODES_BEGIN+18)
// Auto Monitor of trip solenoid
#define AUTO_MONITOR_TRIP_SOLENOID                             (CONTINUOUS_HW_FAULT_CODES_BEGIN+19)
// three message id for over temp
#define OVER_TEMP_MESSAGE                                      (CONTINUOUS_HW_FAULT_CODES_BEGIN+20)
#define OVER_TEMP_TRIP                                         (CONTINUOUS_HW_FAULT_CODES_BEGIN+21)
#define COOLED_DOWN_MESSAGE                                    (CONTINUOUS_HW_FAULT_CODES_BEGIN+22)
#define POWER_FREQUENCY_OUT_OF_RANGE_TRIP                      (CONTINUOUS_HW_FAULT_CODES_BEGIN+23)
#define CONTINUOUS_HW_FAULT_CODES_END                          (POWER_FREQUENCY_OUT_OF_RANGE_TRIP)

#define SECONDARY_SWITCH_DIAGNOSTICS_START					   (96)
#define SECONDARY_SWITCH_FAULT								   (SECONDARY_SWITCH_DIAGNOSTICS_START+0)
#define SECONDARY_SWITCH_STUCK_OPEN					   	   	   (SECONDARY_SWITCH_DIAGNOSTICS_START+1)
#define SECONDARY_SWITCH_STUCK_CLOSED				   	   	   (SECONDARY_SWITCH_DIAGNOSTICS_START+2)
#define SECONDARY_SWITCH_UNEXPECTED_OPEN			   	   	   (SECONDARY_SWITCH_DIAGNOSTICS_START+3)
#define SECONDARY_SWITCH_UNEXPECTED_CLOSED			   	   	   (SECONDARY_SWITCH_DIAGNOSTICS_START+4)
#define SECONDARY_SWITCH_UNEXPECTED_OVERLOAD		   	   	   (SECONDARY_SWITCH_DIAGNOSTICS_START+5)
#define SECONDARY_SWITCH_DEFAULT_TO_CLOSED			   	   	   (SECONDARY_SWITCH_DIAGNOSTICS_START+6)
//#define SECONDARY_SWITCH_BACKUP_TRIGGERED					   (SECONDARY_SWITCH_DIAGNOSTICS_START+7)

#define SECONDARY_SWITCH_DIAGNOSTICS_END					   (104)

#define USER_INITIATED_HW_FAULT_CODES_BEGIN                    (128)
#define USER_INITIATED_HW_FAULT_CODES_END                      (255)

/** @} */

#endif /* _MAIN_INTERNAL_H */
