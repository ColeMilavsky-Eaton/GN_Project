#ifndef _SELF_TEST_API_H
#define _SELF_TEST_API_H

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
 * @brief External interface for the self test component.
 *
 * @file self_test_api.h
 * @ingroup self_test
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "firmware.config"

/* should still be able to measure up to 20 ma if this threshold is reached. */
#define GROUND_FAUL_BIAS_TOLERANCE                               (512)
/* Base on hardware, bias should be around half of supplied VDD */
#define GROUND_FAULT_BIAS_MIN                                    ((u32)((1.65/3.3)*MAX_ADC_VALUE) - GROUND_FAUL_BIAS_TOLERANCE)
#define GROUND_FAULT_BIAS_MAX                                    ((u32)((1.65/3.3)*MAX_ADC_VALUE) + GROUND_FAUL_BIAS_TOLERANCE)


typedef enum
{
    USER_NOT_INITIATED = 0,
    USER_INITIATED_SELF_TEST_PASS = 1,
    USER_INITIATED_SELF_TEST_FAIL = 2,
    USER_INITIATED_SELF_TEST_LOGGED = 3

}result_t;

typedef struct
{
    result_t result;
    union
    {
        bitfield8 failed_trip_log_bits;
        u8 failed_trip_log;
    };

}user_self_test_result_t;

typedef struct
{
    u32* af_data_p;
    u32* hf_sense_data_p;
    u32* line_current_bias_data_p;
    u32* ground_fault_bias_data_p;
    u32* ground_fault_data_p;

}self_test_init_t;

typedef enum
{
    PTT_IDLE_STATE = 0,
    PTT_COUNTER_KICKOFF_VALUE = 13, // value should be set so 60 Hz self-test is applied during correct line voltage polarity!
    PTT_COUNTER_12 = 12,
    PTT_COUNTER_11 = 11,
    PTT_COUNTER_10 = 10,
    PTT_COUNTER_9 = 9,
    PTT_COUNTER_8 = 8,
    PTT_COUNTER_7 = 7,
    PTT_COUNTER_6 = 6,
    PTT_COUNTER_5 = 5,
    PTT_COUNTER_4 = 4,
    PTT_COUNTER_3 = 3,
    PTT_COUNTER_2 = 2,
    PTT_COUNTER_1 = 1,

} ptt_state;

typedef enum
{
    TEST_FAIL = 0,
    TEST_PASS = 1,

}self_test_flag_t;

#if AUTO_MONITOR_CONFIGURATION == AUTO_MONITOR_ENABLED
typedef enum
{
    AUTO_MONITOR_WAIT = 0,
    AUTO_MONITOR_RESET_VAR = 1,
    AUTO_MONITOR_GF_TEST = 2,
    AUTO_MONITOR_TRIP_TEST = 3,

} auto_monitor_state_t;

typedef enum
{
    AUTO_MONITOR_NO_FAULT = 0,
    AUTO_MONITOR_GF_TEST_FAIL = 1,
    AUTO_MONITOR_TRIP_TEST_FAIL = 2,

}auto_monitor_result_t;
#endif

/**************************************************************************************************/
/**
 * @brief Initiate self test.
 *
 * This function sets the state of the user initiated self test to a kick off value and will trigger
 * user initiated self test to run.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void initiate_ptt(void);

/**************************************************************************************************/
/**
 * @brief initialize self test component.
 *
 * Initialize self test component and assign tasks.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void self_test_init(self_test_init_t self_test_init_params);

/**************************************************************************************************/
/**
 * @brief Gets user initiated self test result.
 *
 *
 * @return user_self_test_result_t that includes the result and failed trip log data.
 *
 * @exception none
 *
 **************************************************************************************************/
user_self_test_result_t self_test_get_user_initiated_test_result(void);\

/**************************************************************************************************/
/**
 * @brief sets the result of user initiated self test.
 *
 * The purpose of this function is for main component to update the result to
 * USER_INITIATED_SELF_TEST_LOGGED after writing the trip log in a failed user self test situation.
 *
 * This prevents continuously  writing the log code and eventually fill up the trip code area.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void ptt_set_result_to_logged(void);

/**************************************************************************************************/
/**
 * @brief resets the result of ptt.
 *
 * Resets the result of ptt to get ready if user wants to run ptt again.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void ptt_reset_result(void);

/**************************************************************************************************/
/**
 * @brief returns the state(counter) of the user initiated self test.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
ptt_state self_test_get_user_initiated_test_state(void);


/**************************************************************************************************/
/**
 * @brief checks for hf sense connection
 *
 * check if the test value is within range. And if out of range for a certain amount of time.
 * Return fault detected.
 *
 *
 * @return STATUS_NO_FAULT_DETECTED if no fault
 *         STATUS_FAULT_DETECTED if fault exists.
 *
 * @exception none
 *
 **************************************************************************************************/
status_t check_for_hf_sense_connection(void);

/**************************************************************************************************/
/**
 * @brief Check for proper number of interrupts.
 *
 *  This function is called after ZCD. The correct interrupt number should be the number before ZCD
 *  hits. At any time interrupt number doesn't match what is expected, a flag is raised. And when
 *  the number of times the number doesn't match reaches the trip limit, fault detected is returned.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
status_t check_for_proper_number_of_interrupts(void);

/**************************************************************************************************/
/**
 * @brief Check for if the power frequency is out of the range (57-63Hz).
 *
 *  This function is called after ZCD. The frequency is checked every cycle; a flag is raised when
 *  the frequecy is out of the range for 86 Seconds, fault detected is returned.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
status_t check_for_proper_frequency(void);
/**************************************************************************************************/
/**
 * @brief get sequence number error counter.
 *
 * @return sequence_number.error_counter
 *
 * @exception none
 *
 **************************************************************************************************/
u16 get_interrupt_check_error_counter(void);


/**************************************************************************************************/
/**
 * @brief Check for proper data sequence.
 *
 *  Checks if the sequence number is incrementing correctly. If fails consecutively for 120 half
 *  cycles, breaker will trip.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
status_t check_for_proper_data_sequence(void);


/**************************************************************************************************/
/**
 * @brief get foreground sequence number
 *
 * @return sequence_number.foreground
 *
 * @exception none
 *
 **************************************************************************************************/
u32 get_foreground_sequence_number(void);


/**************************************************************************************************/
/**
 * @brief get background sequence number
 *
 * @return sequence_number.background
 *
 * @exception none
 *
 **************************************************************************************************/
u32 get_background_sequence_number(void);


/**************************************************************************************************/
/**
 * @brief get sequence number error counter.
 *
 * @return sequence_number.error_counter
 *
 * @exception none
 *
 **************************************************************************************************/
u16 get_sequence_number_error_counter(void);


/**************************************************************************************************/
/**
 * @brief checks for data overrun
 *
 * Using the sequence number to check if data has been over written before being processed.
 *
 * If the sequence number don't match for a certain amount of time, return fault detected.
 *
 *
 * @return STATUS_NO_FAULT_DETECTED if no fault
 *         STATUS_FAULT_DETECTED if fault exists.
 *
 * @exception none
 *
 **************************************************************************************************/
status_t check_for_data_overrun(void);

/**************************************************************************************************/
/**
 * @brief checks for line bias.
 *
 * checks if line bias is within range, if out of range for a certain amount of time, return fault
 * detected.
 *
 *
 * @return STATUS_NO_FAULT_DETECTED if no fault
 *         STATUS_FAULT_DETECTED if fault exists.
 *
 * @exception none
 *
 **************************************************************************************************/
status_t check_for_line_current_bias_out_of_range(void);


/**************************************************************************************************/
/**
 * @brief get line current bias error counter.
 *
 * @return error counter for line current bias check
 *
 * @exception none
 *
 **************************************************************************************************/
u16 get_line_current_bias_error_counter(void);


/**************************************************************************************************/
/**
 * @brief get hf sense connection error counter.
 *
 * @return error counter for hf sense connection check
 *
 * @exception none
 *
 **************************************************************************************************/
u16 get_hf_sense_error_counter(void);

/**************************************************************************************************/
/**
 * @brief resets sequence numbers
 *
 *  Resets sequence numbers, this is called when there's an error and needs to reset.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void reset_sequence_numbers(void);

#if GF_CONFIGURATION != GF_NOT_ENABLED
/**************************************************************************************************/
/**
 * @brief checks for ground fault bias.
 *
 *
 * checks if ground fault bias is within range. An error counter increases if bias is out of range,
 * and when error counter reach a threshold, fault active is returned.
 *
 *
 * @return STATUS_NO_FAULT_DETECTED if no fault
 *         STATUS_FAULT_DETECTED if fault exists.
 *
 * @exception none
 *
 **************************************************************************************************/
status_t check_for_ground_fault_bias_out_of_range(void);

/**************************************************************************************************/
/**
 * @brief get ground fault bias error flag.
 *
 * @return error flag for ground fault bias
 *
 * @exception none
 *
 **************************************************************************************************/
self_test_flag_t get_ground_fault_bias_error_flag(void);

/**************************************************************************************************/
/**
 * @brief Resets the ground fault error flag.
 *
 * Resets the ground fault error flag to TEST_PASS
 *
 * @return None
 *
 * @exception none
 *
 **************************************************************************************************/
void reset_ground_fault_bias_error_flag();
#endif

#if AUTO_MONITOR_CONFIGURATION == AUTO_MONITOR_ENABLED
/**************************************************************************************************/
/**
 * @brief initialize auto monitor.
 *
 * Initializes global variables and assign tasks to task manager for the auto monitor feature.
 *
 * @param[in]  self_test_init_params - parameters to pass in.
 *
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void auto_monitor_init(self_test_init_t self_test_init_params);

/**************************************************************************************************/
/**
 * @brief get auto monitor result.
 *
 * @return auto monitor result.
 *
 * @exception none
 *
 **************************************************************************************************/
inline auto_monitor_result_t auto_monitor_get_result(void);

/**************************************************************************************************/
/**
 * @brief task to turn off trip signal
 *
 * Purpose of this task is to turn off the scr (trip). Note that turn on the scr should be in the
 * beginning of the task list and this task (turn off) needs to be close to end of the task list
 * because scr will not respond is turn off to quickly.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void auto_monitor_turn_off_trip_task(u32 it);

/**************************************************************************************************/
/**
 * @brief get auto monitor state.
 *
 * @return auto monitor state.
 *
 * @exception none
 *
 **************************************************************************************************/
auto_monitor_state_t auto_monitor_get_state(void);
/**************************************************************************************************/
/**
 * @brief get auto monitor trip test flag.
 *
 * If this flag returns true means that auto monitor is currently testing the trip circuit
 *
 * @return auto monitor trip test flag.
 *
 * @exception none
 *
 **************************************************************************************************/
bool auto_monitor_get_trip_test_flag(void);

/**************************************************************************************************/
/**
 * @brief get auto monitor cycle
 *
 * This is needed for dual solenoid configuration in single pole breakers to pass information to
 * interrupt component to know which solenoid to turn on.
 *
 * @return auto monitor trip test flag.
 *
 * @exception none
 *
 **************************************************************************************************/
u32 auto_monitor_get_cycle(void);

#endif

#endif /* _SELF_TEST_API_H */

