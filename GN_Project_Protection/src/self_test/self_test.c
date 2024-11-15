/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 ***************************************************************************************************
 *  Written by:         Neha Modi
 *                      EIIC Eaton Electrical
 *                      Magarpatta City, Hadapsar
 *                      Pune MH-411 013
 *//**
 * @defgroup self_test self_test component
 *
 * @brief self_test component handles breakers hardware self tests.
 *
 * # Overview
 * The self test component handles mainly three categories of self test. Push to test(PTT),
 * auto monitor and continuous hardware self checks. This C file mainly handles the initialization
 * of the component. PTT, Auto monitor and other continuous self checks will be described in other
 * C files.
 *
 *
 *
 * @file self_test.c
 * @ingroup self_test
 *
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "main_internal.h"
#include "self_test_.h"
#include "main_api.h"
#include "load_voltage_api.h"
#include "task_manager_api.h"
#include "stm32g0xx_it_api.h"
#include "main_api.h"
#include "firmware.config"
#include "gpio_api.h"


void self_test_init(self_test_init_t self_test_init_params)
{
    /* initialize hf_sense */
    hf_sense_init(self_test_init_params);

    /* initialize interrupt check */
    interrupt_check_init();

    frequency_check_init();
    /* initialize sequence check */
    sequence_number_check_init();

    /* initialize data overrun check */
    data_overrun_check_init();

  #if OVERLOAD_CONFIGURATION != OVERLOAD_NOT_ENABLED
    /* initialize line current bias check */
    line_current_bias_check_init(self_test_init_params);
  #endif
    
  #if GF_CONFIGURATION != GF_NOT_ENABLED
    /* initialize ground fault current bias check */
    ground_fault_bias_check_init(self_test_init_params);
  #endif

    /* initialize ptt */
    ptt_init(self_test_init_params);

    /* add hf sense zcd task */
#if AF_CONFIGURATION != AF_NOT_ENABLED
    task_man_add_task_to_schedule((task_t){.task_p = hf_sense_zcd_callback,
                                           .it_bits = TSK_MAN_ZCD});
#endif

    /* add hf sense task */
#if AF_CONFIGURATION != AF_NOT_ENABLED
    task_man_add_task_to_schedule((task_t){.task_p = hf_sense_data_collect_task,
                                           .it_bits = TSK_MAN_IT_ALL});
#endif

    return;
}

