#ifndef _STM32G0xx_IT__H
#define _STM32G0xx_IT__H
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
 * @brief internal (to the component) interface for the interrupt component.
 *
 * @file stm32g0xx_it_.h
 * @ingroup interrupt
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "stm32g0xx_it_internal.h"
#include "firmware.config"

#define IT_COMPONENT


#define COLLECTION_INTERVAL_SHIFT 4
#define ZCD_TIMEOUT_SHIFT 1


#warning adjust this number
#define COLLECTION_INTERVAL_FUDGE_FACTOR 3

#define MIN_HALF_CYCLE_PERIOD_CNT  ((COLLECTION_INTERVAL_FUDGE_FACTOR + 1) << \
                                   COLLECTION_INTERVAL_SHIFT)

// Values are after applying factor, since factor will scale raw  170V adc value to 120,000
// so the values are constant.
// TODO: Values are from simulation, adjust when have the actual hardware
#define LOAD_VOLTAGE_MINIMUM_NO_VOLTAGE_THRESHOLD (7059) // 10V peak
#define LOAD_VOLTAGE_MINIMUM_VOLTAGE_THRESHOLD (28235) //40V peak
//#define LOAD_VOLTAGE_MINIMUM_VOLTAGE_THRESHOLD (35295) //50V peak

//#define LOAD_VOLTAGE_MINIMUM_NO_VOLTAGE_THRESHOLD ((u32)(10.0) * LOAD_VOLTAGE_RATIO)
//#define LOAD_VOLTAGE_MINIMUM_VOLTAGE_THRESHOLD ((u32)(40.0) * LOAD_VOLTAGE_RATIO)
//#define LOAD_VOLTAGE_MINIMUM_VOLTAGE_THRESHOLD ((u32)(70.0) * LOAD_VOLTAGE_RATIO)

#if (ADV_GF_ONLY_30mA)
#define HRGF_UNDERVOLTAGE_LOCKOUT_THRESHOLD                  ((u32)(60.0) * LOAD_VOLTAGE_RATIO)
#else
#define HRGF_UNDERVOLTAGE_LOCKOUT_THRESHOLD                  ((u32)(70.0) * LOAD_VOLTAGE_RATIO)
#endif
#define FREQUENCY_MEASURE_CONSTANT  (250000000) // CPU_FREQUENCY/TIMER16_PRESCALE = 16000000/64, multiple by 100 to get up t0 0.01Hz
#define FREQUENCY_MEASURE_FUDGE_FACTOR (5)

#define COMPLETE_LOSS_OF_ZCD_THRESHOLD  (120) /* if zcd timeout is 1 cycle, then 300 counts should be arround 2 seconds */

#define HRGF_RUNNING_TRIP_THRESHOLD     (2)  /* trip threshold for hrgf running */

#define JUMP_BACK_TO_PRIMARY_ZCD1_THRESHOLD (5)

#define JUMP_BACK_TO_PRIMARY_ZCD1_TIMEOUT (15) /*  8.33 * (14-1) = around 116ms */

 #endif /* _STM32G0xx_IT__H */

