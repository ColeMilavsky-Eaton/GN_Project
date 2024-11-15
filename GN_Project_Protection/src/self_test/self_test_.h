#ifndef _SELF_TEST__H
#define _SELF_TEST__H

/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 **************************************************************************************************
 *  Written by:         Neha Modi
 *                      EIIC Eaton Electrical
 *                      Magarpatta City, Hadapsar
 *                      Pune MH-411 013
 *//**
 * @brief Internal (to the component) interface for the self test component.
 *
 * @file self_test_.h
 * @ingroup self_test
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "utils.h"
#include "self_test_internal.h"
#include "firmware.config"
#include "gpio_api.h"


typedef struct self_test_error_t
{
    u16 error_counter;
    self_test_flag_t error_flag;

} self_test_error_t;

#define TEST_GF_SET                         gpio_set_output_pin(TEST_GF_GPIO_Port,TEST_GF_Pin)
#define TEST_GF_RESET                       gpio_reset_output_pin(TEST_GF_GPIO_Port,TEST_GF_Pin)

#define TEST_CURRENT_SET                    gpio_set_output_pin(TEST_CURRENT_GPIO_Port,TEST_CURRENT_Pin)
#define TEST_CURRENT_RESET                  gpio_reset_output_pin(TEST_CURRENT_GPIO_Port,TEST_CURRENT_Pin)

#warning this define needs to be moved to breaker core load voltage component
#define LOAD_VOLTAGE_UNDERVOLTAGE_LOCKOUT                       ((u32)70 * LOAD_VOLTAGE_RATIO)

#define SELF_TEST_LINE_CURRENT_OK_MIN                          ((u16)(4096.0 * (0.555/3.3)))
#define SELF_TEST_LINE_CURRENT_OK_MAX                          ((u16)(4096.0 * (0.945/3.3)))

#define SELF_TEST_LOG_HF_MIN_THRESH                            (2560)
#define SELF_TEST_LOG_HF_MIN_EXPECTED_COUNT                    (14)

#define SELF_TEST_HF_SENSE_FAULT_MINIMUM_TEST_VALUE            (800) // this needs more work -- corresponds to about 80 Vrms: 80*5=400, should also be coordinated with HF_SENSE_FAULT_MINIMUM_VOLTAGE_IN_CTS
#define SELF_TEST_HF_SENSE_FAULT_MAXIMUM_TEST_VALUE            (24000) // this needs more work -- corresponds to about 140 Vrms: 140*5=650
#define SELF_TEST_HF_SENSE_FAULT_TRIP_LIMIT                    (120)

#define INTERRUPT_ERROR_COUNTER_TRIP_LIMIT                       (10320)
#define FREQUENCY_OUT_OF_RANGE_COUNTER_TRIP_LIMIT                (7200)  // 7200 Half cycles is 1 min
#define INVALID_SEQUENCE_TRIP_LIMIT                              (120)
#define DATA_OVERRUN_ERROR_TRIP_LIMIT                            (120)
#define LINE_CURRENT_BIAS_ERROR_TRIP_LIMIT                       (120)
#define LINE_CURRENT_BIAS_TOLERANCE                              (200)
#define LINE_CURRENT_BIAS_MIN                                    (((1.65/3.3)*4096) - LINE_CURRENT_BIAS_TOLERANCE)
#define LINE_CURRENT_BIAS_MAX                                    (((1.65/3.3)*4096) + LINE_CURRENT_BIAS_TOLERANCE)

#define GROUND_FAULT_BIAS_ERROR_INCREMENT_COUNT                  (2)
#define GROUND_FAULT_BIAS_ERROR_DECREMENT_COUNT                  (1)
#define GROUND_FAULT_BIAS_ERROR_TRIP_THREHOLD_COUNT              (120)

#if AUTO_MONITOR_CONFIGURATION == AUTO_MONITOR_ENABLED
#define AUTO_MONITOR_ONE_SECOND_HALF_CYCLE_COUNTS          (120)
#define AUTO_MONITOR_INTERVAL_SECONDS                      (1200)

#define AUTO_MONITOR_SECONDS_AFTER_STARTUP                 (2)
#if NUM_POLES_CONFIGURATION == SINGLE_POLE
#define AUTO_MONITOR_GF_TEST_TRY_TIMES                     (4)
#elif NUM_POLES_CONFIGURATION == TWO_POLE
#define AUTO_MONITOR_GF_TEST_TRY_TIMES                     (6)
#else
  #error number of poles not configured
#endif

#define AUTO_MONITOR_TRIP_TEST_TRY_TIMES                   (4)

#define AUTO_MONITOR_GF_TEST_FIRST_TRY_CYCLE               (2)
#define AUTO_MONITOR_GF_TEST_SECOND_TRY_CYCLE              (AUTO_MONITOR_GF_TEST_FIRST_TRY_CYCLE + 6)
#define AUTO_MONITOR_GF_TEST_THIRD_TRY_CYCLE               (AUTO_MONITOR_GF_TEST_FIRST_TRY_CYCLE + 12)
#define AUTO_MONITOR_GF_TEST_FOURTH_TRY_CYCLE              (AUTO_MONITOR_GF_TEST_FIRST_TRY_CYCLE + 18)
#define AUTO_MONITOR_GF_TEST_FIFTH_TRY_CYCLE               (AUTO_MONITOR_GF_TEST_FIRST_TRY_CYCLE + 24)
#define AUTO_MONITOR_GF_TEST_SIXTH_TRY_CYCLE               (AUTO_MONITOR_GF_TEST_FIRST_TRY_CYCLE + 30)


#define AUTO_MONITOR_TRIP_TEST_FIRST_TRY_CYCLE             (AUTO_MONITOR_GF_TEST_SIXTH_TRY_CYCLE + 6)
#define AUTO_MONITOR_TRIP_TEST_SECOND_TRY_CYCLE            (AUTO_MONITOR_TRIP_TEST_FIRST_TRY_CYCLE + 6)
#define AUTO_MONITOR_TRIP_TEST_THIRD_TRY_CYCLE             (AUTO_MONITOR_TRIP_TEST_FIRST_TRY_CYCLE + 12)
#define AUTO_MONITOR_TRIP_TEST_FOURTH_TRY_CYCLE            (AUTO_MONITOR_TRIP_TEST_FIRST_TRY_CYCLE + 18)
#endif

// The Measured accuracy is about 0.1Hz in the range of 57-63Hz
#define LOW_LIMIT_OF_FREQUENCY_CHECK_RANGE                 56900 // 56.9 Hz
#define HIGH_LIMIT_OF_FREQUENCY_CHECK_RANGE                63100 // 63.1 Hz
/**************************************************************************************************/
/**
 * @brief Initialize global variables for self test componet.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void self_test_initialize_globals(void);

/**************************************************************************************************/
/**
 * @brief Call back function for self test zcd task.
 *
 * This function should run at every ZCD event.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void ptt_zcd_callback(u32 it);

/**************************************************************************************************/
/**
 * @brief task function for user initiated self test.
 *
 * This function should run on every interrupt count. which goes through every step of the user
 * initiated self test process.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void ptt(void);

/**************************************************************************************************/
/**
 * @brief Initialize 8MHz pwm signal.
 *
 *  This function will setup a 8MHz pwm signal primarily used for AFCI user self test
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void self_test_pwm_init(void);

/**************************************************************************************************/
/**
 * @brief starts pwm signal
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void self_test_start_pwm(void);

/**************************************************************************************************/
/**
 * @brief stops pwm signal
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void self_test_stop_pwm(void);

/**************************************************************************************************/
/**
 * @brief task to run ptt
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void ptt_task(u32 it);

/**************************************************************************************************/
/**
 * @brief sets the test HF line low.
 *
 * configures the test HF line to gpio output LOW. this is needed after turning off pwm on the test
 * HF line.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void self_test_set_test_HF_line_low(void);

/**************************************************************************************************/
/**
 * @brief initialize hf sense
 *
 * initialize globals and assign parameters.
 *
 * @param[in] self_test_init_params - passing the hf sense variable address.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void hf_sense_init(self_test_init_t self_test_init_params);

/**************************************************************************************************/
/**
 * @brief this task collects hf sense data.
 *
 * Collects hf sense data and saves the integral value through a half cycle. At the same time
 * compare the current value from previous average and accumulate the difference.
 *
 *
 * @param[in] it - interrupt to run the task.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void hf_sense_data_collect_task(u32 it);

/**************************************************************************************************/
/**
 * @brief ZCD task for hf sense.
 *
 * calculate an average for the previous half cycle and save a copy of the previous accumulated
 * test value.
 *
 *
 * @param[in] it - interrupt to run the task. (ZCD)
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void hf_sense_zcd_callback(u32 it);

/**************************************************************************************************/
/**
 * @brief get hf sense fault flag.
 *
 * @return TEST_PASS
 *         TEST_FAIL
 *
 * @exception none
 *
 **************************************************************************************************/
inline self_test_flag_t hf_sense_get_fault_flag(void);

/**************************************************************************************************/
/**
 * @brief set hf sense fault flag.
 *
 * @param[in] flag - TEST_PASS or TEST_FAIL
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
inline void set_hf_sense_fault_flag(self_test_flag_t flag);


/**************************************************************************************************/
/**
 * @brief initialize interrupt check
 *
 * initialize globals.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void interrupt_check_init(void);

/**************************************************************************************************/
/**
 * @brief get interrupt check fault flag.
 *
 * @return TEST_PASS
 *         TEST_FAIL
 *
 * @exception none
 *
 **************************************************************************************************/
inline self_test_flag_t get_interrupt_check_error_flag(void);


/**************************************************************************************************/
/**
 * @brief set interrupt check fault flag.
 *
 * @param[in] flag - TEST_PASS or TEST_FAIL
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
inline void reset_interrupt_check_error_flag(void);

/**************************************************************************************************/
/**
 * @brief initialize sequence number check
 *
 * initialize globals.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void sequence_number_check_init(void);

/**************************************************************************************************/
/**
 * @brief reset sequence number error flag.
 *
 * @param[in] flag - TEST_PASS or TEST_FAIL
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
inline void reset_sequence_number_error_flag(void);

/**************************************************************************************************/
/**
 * @brief get sequence number fault flag.
 *
 * @return TEST_PASS
 *         TEST_FAIL
 *
 * @exception none
 *
 **************************************************************************************************/
inline self_test_flag_t get_sequence_number_error_flag(void);


/**************************************************************************************************/
/**
 * @brief updates background sequence number.
 *
 * @param[in] it - interrupt number.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void update_background_sequence_number(u32 it);


/**************************************************************************************************/
/**
 * @brief initialize data overrun check
 *
 * initialize globals.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void data_overrun_check_init(void);

/**************************************************************************************************/
/**
 * @brief get data overrun fault flag.
 *
 * @return TEST_PASS
 *         TEST_FAIL
 *
 * @exception none
 *
 **************************************************************************************************/
inline self_test_flag_t get_data_overrun_error_flag(void);

/**************************************************************************************************/
/**
 * @brief set data overrun error flag.
 *
 * @param[in] flag - TEST_PASS or TEST_FAIL
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
inline void reset_data_overrun_error_flag(void);

/**************************************************************************************************/
/**
 * @brief initialize line current bias check
 *
 * initialize globals and assign parameters.
 *
 * @param[in] self_test_init_params - passing the line current bias variable address.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void line_current_bias_check_init(self_test_init_t self_test_init_params);

/**************************************************************************************************/
/**
 * @brief get line current bias fault flag.
 *
 * @return TEST_PASS
 *         TEST_FAIL
 *
 * @exception none
 *
 **************************************************************************************************/
inline self_test_flag_t get_line_current_bias_error_flag(void);

/**************************************************************************************************/
/**
 * @brief set line current bias error flag.
 *
 * @param[in] flag - TEST_PASS or TEST_FAIL
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
inline void set_line_current_bias_error_flag(self_test_flag_t flag);

#if GF_CONFIGURATION != GF_NOT_ENABLED
/**************************************************************************************************/
/**
 * @brief initialize ground fault bias check
 *
 * initialize globals and assign parameters.
 *
 * @param[in] self_test_init_params - passing the ground fault bias variable address.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void ground_fault_bias_check_init(self_test_init_t self_test_init_params);
#endif

/**************************************************************************************************/
/**
 * @brief initialize push to test (PTT)
 *
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void ptt_init(self_test_init_t self_test_init_params);

/**************************************************************************************************/
/**
 * @brief initialize globals used for push to test (PTT)
 *
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void ptt_initialize_globals(void);

#if AUTO_MONITOR_CONFIGURATION == AUTO_MONITOR_ENABLED
/**************************************************************************************************/
/**
 * @brief initialize auto monitor
 *
 * initialize globals.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void auto_monitor_initialize_globals(void);

/**************************************************************************************************/
/**
 * @brief ZCD task for auto monitor.
 *
 * This task counts up and determine the right time to run automonitor.
 *
 *
 * @param[in] it - interrupt to run the task. (ZCD)
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void auto_monitor_zcd_callback(u32 it);

/**************************************************************************************************/
/**
 * @brief automonitor task
 *
 * This task runs the whole automonitor sequence that checks gf CT and solenoid.
 *
 *
 * @param[in] it - interrupt to run the task.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void auto_monitor(u32 it);
#endif

#endif /* _SELF_TEST__H */
