/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 ***************************************************************************************************
 *  Written by:         Hank Sun
 *                      Eaton Electrical
 *                      1000 Cherrington Parkway
 *                      Pittsburgh, PA  15108-4312
 *                      (412) 893-3300
 *//**
 *
 * @brief Runs a series of tests when requested.
 *
 *
 * # Description
 *
 * PTT task runs on every interrupt count and checked if a PTT is requested. When PTT is requested,
 * PTT task then runs a series of tests. A counter determines which tests to run and when PTT is
 * requested, the counter is set to a kickoff value. The counter also counts down on every ZCD
 * interrupt. The last step for PTT is to conclude the above and hardware self checks results. Each
 * tests has a flag(bit) that keeps track of pass or fail. The final results will be stored.
 *
 * ##Tests
 *
 * ### Ground Fault Coil Check
 * A test signal is injected to the ground fault circuit and the result(output) is read back to the
 * microcontroller. The microcontroller checks if the value is within thresholds and stores the
 * result in a flag.
 *
 * ### Line Current Circuit Check
 * A test signal is injected in both the positive and negative half cycle. The output is then read
 * back to the microcontroller and the result is stored in a flag.
 *
 * ###Arc Fault Circuit Check
 * A PWM signal is injected to the arc fault circuit for half a cycle and the output of the circuit
 * is read back to the microcontroller. The result is stored in a flag.
 *
 * @file ptt.c
 * @ingroup self_test
 *
 *
 *//*
 *
 **************************************************************************************************/
#include "self_test_.h"
#include "types.h"
#include "firmware.config"
#include "task_manager_api.h"
#include "main_internal.h"
#include "load_voltage_api.h"
#include "stm32g0xx_it_api.h"



#if AF_CONFIGURATION != AF_NOT_ENABLED
self_test_flag_t log_hf_min_detector_stuck_error_flag;  // this flag is not used anymore. its existence is keep the same format as 8 bit
u32 hf_min_test_counter;
u32* self_test_af_data_p;
#endif

#if GF_CONFIGURATION != GF_NOT_ENABLED
u32 ptt_gf_buffer[NUM_INTERRUPTS_PER_HALFCYCLE * 2]; // stores gf data in buffer
u32 *ptt_ground_fault_data_p;
u32 ptt_ground_fault_cycle_diff; // difference between current gf data and one cycle ago.
u32 ptt_ground_fault_integrator;
self_test_flag_t ptt_ground_fault_coil_test;
#endif

ptt_state user_initiated_self_test_state;
user_self_test_result_t user_initiated_self_test_result;



void ptt_init(self_test_init_t self_test_init_params)
{
    ptt_initialize_globals();

  #if AF_CONFIGURATION != AF_NOT_ENABLED
    self_test_af_data_p = self_test_init_params.af_data_p;
  #endif

  #if GF_CONFIGURATION != GF_NOT_ENABLED
    ptt_ground_fault_data_p = self_test_init_params.ground_fault_data_p;
  #endif

    /* add zcd task */
    task_man_add_task_to_schedule((task_t){.task_p = ptt_zcd_callback,
                                           .it_bits = TSK_MAN_ZCD});

    /* add ptt task */
    task_man_add_task_to_schedule((task_t){.task_p = ptt_task,
                                           .it_bits = TSK_MAN_IT_ALL});

    return;
}

void ptt_initialize_globals(void)
{
    /* initializing globals */
  #if AF_CONFIGURATION != AF_NOT_ENABLED
    log_hf_min_detector_stuck_error_flag = TEST_PASS;
    hf_min_test_counter = 0;
    self_test_af_data_p = NULL;
  #endif

  #if GF_CONFIGURATION != GF_NOT_ENABLED
    for(u32 i = 0; i < (NUM_INTERRUPTS_PER_HALFCYCLE * 2); i++)
    {
        ptt_gf_buffer[i] = 0;
    }
    ptt_ground_fault_cycle_diff = 0;
    ptt_ground_fault_integrator = 0;
    ptt_ground_fault_coil_test = TEST_PASS;
  #endif

    user_initiated_self_test_state = PTT_IDLE_STATE;
    user_initiated_self_test_result.result = USER_NOT_INITIATED;
    user_initiated_self_test_result.failed_trip_log = 0;



    return;
}

void ptt_task(u32 it)
{
    ptt();

    return;
}

void ptt_zcd_callback(u32 it)
{
    /* decrement user self test state(counter)*/
    if((user_initiated_self_test_state <= PTT_COUNTER_KICKOFF_VALUE) &&
       (user_initiated_self_test_state > 0))
    {
        user_initiated_self_test_state--;
    }

    /* this is added in case interrupt count 15 is somehow skipped */
    TEST_GF_RESET;

    return;
}

void ptt(void)
{
  #if OVERLOAD_CONFIGURATION != OVERLOAD_NOT_ENABLED
    static self_test_flag_t line_current_neg_OK = TEST_FAIL;
    static self_test_flag_t line_current_pos_OK = TEST_FAIL;
    static s16 line_current_pre_stimulus = 0;
    s16 line_current_neg_difference = 0;
    s16 line_current_pos_difference = 0;
  #endif

  #if AF_CONFIGURATION != AF_NOT_ENABLED
    static self_test_flag_t log_HF_min_OK = TEST_FAIL;
  #endif

    u16 current_interrupt_cnt = it_get_interrupt_count();

  #if GF_CONFIGURATION != GF_NOT_ENABLED
    /* calcluate the difference between gf data one cycle ago and current */
    ptt_ground_fault_cycle_diff = ABS((s32)ptt_gf_buffer[current_interrupt_cnt] - (s32)(*ptt_ground_fault_data_p));

    /* update buffer */
    ptt_gf_buffer[current_interrupt_cnt] = *ptt_ground_fault_data_p;
  #endif

    switch(user_initiated_self_test_state)
    {
        case PTT_COUNTER_11:
#if GF_CONFIGURATION != GF_NOT_ENABLED
            /* turn on test GF signal */
            TEST_GF_SET;

            /* integrate GF output*/
            if( ((current_interrupt_cnt & 0x0F) >= 10) && ((current_interrupt_cnt & 0x0F) <= 15) )
            {
                /* add up the difference */
                ptt_ground_fault_integrator += ptt_ground_fault_cycle_diff;

                /* check if integrated GF output is within range */
                if((current_interrupt_cnt & 0x0F) == 15)
                {
                    if(ptt_ground_fault_integrator > SELF_TEST_GF_COIL_OK_MIN)
                    {
                        ptt_ground_fault_coil_test = TEST_PASS;
                    }
                    else
                    {
                        ptt_ground_fault_coil_test = TEST_FAIL;
                    }
                    TEST_GF_RESET;
                }
            }



#endif
            break;
        case PTT_COUNTER_6:
          #if NUM_POLES_CONFIGURATION == TWO_POLE
            /* This part of the test is added for 2 pole. When both poles powered and around 3.5mA
             * of ground fault current on pole 2. Since the timing is all based on ZCD1, the
             * injection of the ground fault test signal pushes the ground fault current to the
             * other side(polarity). ptt compares the ground fault data (ABS data) with the
             * previous full cycle and see no difference and causes ptt to fail gf self test.
             * So on PTT_COUNTER_6, the test will run again if failed on PTT_COUNTER_11.
             * Note that PTT_COUNTER_6 will be on a different polarity with PTT_COUNTER_11.
             *  */
            #if GF_CONFIGURATION != GF_NOT_ENABLED
            if(ptt_ground_fault_coil_test == TEST_FAIL)
            {
                /* turn on test GF signal */
                TEST_GF_SET;

                /* integrate GF output*/
                if( ((current_interrupt_cnt & 0x0F) >= 10) && ((current_interrupt_cnt & 0x0F) <= 15) )
                {
                    /* add up the difference */
                    ptt_ground_fault_integrator += ptt_ground_fault_cycle_diff;

                    /* check if integrated GF output is within range */
                    if((current_interrupt_cnt & 0x0F) == 15)
                    {
                        if(ptt_ground_fault_integrator > SELF_TEST_GF_COIL_OK_MIN)
                        {
                            ptt_ground_fault_coil_test = TEST_PASS;
                        }
                        else
                        {
                            ptt_ground_fault_coil_test = TEST_FAIL;
                        }
                        TEST_GF_RESET;
                    }
                }
            }


            #endif
          #endif
            break;

        case PTT_COUNTER_5:
          #if OVERLOAD_CONFIGURATION == OVERLOAD_ENABLED
            TEST_CURRENT_SET;

            if ((current_interrupt_cnt & 0x0F) == 0)
            {
               line_current_pre_stimulus = (s16)main_get_current_plus_bias();
            }
            if ((current_interrupt_cnt & 0x0F) == 1)
            {
               line_current_neg_difference = line_current_pre_stimulus - (s16)main_get_current_plus_bias();
               if (( line_current_neg_difference > SELF_TEST_LINE_CURRENT_OK_MIN ) &&
                   ( line_current_neg_difference < SELF_TEST_LINE_CURRENT_OK_MAX ))
               {
                  line_current_neg_OK = TEST_PASS;
               }
            }
          #endif
            break;

        case PTT_COUNTER_4:
          #if OVERLOAD_CONFIGURATION == OVERLOAD_ENABLED
            TEST_CURRENT_RESET;
            if ((current_interrupt_cnt & 0x0F) == 0)
            {
                line_current_pre_stimulus = (s16)main_get_current_plus_bias();
            }
            if ((current_interrupt_cnt & 0x0F) == 1)
            {
                line_current_pos_difference = (s16)main_get_current_plus_bias() - line_current_pre_stimulus;
                if (( line_current_pos_difference > SELF_TEST_LINE_CURRENT_OK_MIN ) &&
                    ( line_current_pos_difference < SELF_TEST_LINE_CURRENT_OK_MAX ))
                {
                    line_current_pos_OK = TEST_PASS;
                }
            }
          #endif
            break;

        case PTT_COUNTER_3:
#if AF_CONFIGURATION != AF_NOT_ENABLED
            if((current_interrupt_cnt & 0x0F) == 0)
            {
                self_test_pwm_init();
                self_test_start_pwm();
            }
            if (*self_test_af_data_p > SELF_TEST_LOG_HF_MIN_THRESH)
            {
                hf_min_test_counter++;
            }
#endif
            break;

        case PTT_COUNTER_2:
#if AF_CONFIGURATION != AF_NOT_ENABLED
            if((current_interrupt_cnt & 0x0F) == 0)
            {
                self_test_stop_pwm();
                self_test_set_test_HF_line_low();
            }
            if (current_interrupt_cnt == 1)
            {
               if (hf_min_test_counter >= SELF_TEST_LOG_HF_MIN_EXPECTED_COUNT)
               {
                  log_HF_min_OK = TEST_PASS;
               }
            }
#endif
            break;

        case PTT_COUNTER_1:
            /* summarize result */
             if ((current_interrupt_cnt & 0x0F) == 1)
             {
                 /* if all test pass */
                 if(
                  #if OVERLOAD_CONFIGURATION != OVERLOAD_NOT_ENABLED
                    ( line_current_neg_OK == TEST_PASS ) &&
                    ( line_current_pos_OK == TEST_PASS ) &&
                    ( get_line_current_bias_error_flag() == TEST_PASS) &&
                  #endif
                  #if AF_CONFIGURATION != AF_NOT_ENABLED
                    ( log_HF_min_OK == TEST_PASS ) &&
                    ( hf_sense_get_fault_flag() == TEST_PASS ) &&
                    ( log_hf_min_detector_stuck_error_flag == TEST_PASS ) &&
                  #endif
                  #if GF_CONFIGURATION != GF_NOT_ENABLED
                     ( ptt_ground_fault_coil_test == TEST_PASS ) &&
                     ( get_ground_fault_bias_error_flag() == TEST_PASS) &&
                  #endif
                     ( get_sequence_number_error_flag() == TEST_PASS ) &&
                     ( get_interrupt_check_error_flag() == TEST_PASS ) &&
                     ( get_data_overrun_error_flag() == TEST_PASS )
                    )
                 {
                     user_initiated_self_test_result.result = USER_INITIATED_SELF_TEST_PASS;

                     /* jump to idle state since there's no need to log any errors */
                     user_initiated_self_test_state = PTT_IDLE_STATE;
                 }
             }

             if ((current_interrupt_cnt & 0x0F) == 3)
             {
                 user_initiated_self_test_result.result = USER_INITIATED_SELF_TEST_FAIL;

                 if (
                       #if OVERLOAD_CONFIGURATION != OVERLOAD_NOT_ENABLED
                         ( line_current_pos_OK == TEST_FAIL ) ||
                         ( line_current_neg_OK == TEST_FAIL ) ||
                       #else
                         0 ||
                       #endif
                       #if AF_CONFIGURATION != AF_NOT_ENABLED
                         ( log_HF_min_OK == TEST_FAIL ) ||
                         ( hf_sense_get_fault_flag() == TEST_FAIL ) ||
                         ( log_hf_min_detector_stuck_error_flag == TEST_FAIL )
                       #else
                         0
                       #endif
                     )
                 {
                     user_initiated_self_test_result.failed_trip_log_bits.b7 = 1;

                     user_initiated_self_test_result.failed_trip_log_bits.b6 = 0;

                   #if OVERLOAD_CONFIGURATION != OVERLOAD_NOT_ENABLED
                     user_initiated_self_test_result.failed_trip_log_bits.b5= line_current_pos_OK;

                     user_initiated_self_test_result.failed_trip_log_bits.b4 = line_current_neg_OK;

                     user_initiated_self_test_result.failed_trip_log_bits.b3 = get_line_current_bias_error_flag();
                   #else
                     user_initiated_self_test_result.failed_trip_log_bits.b5= TEST_PASS;

                     user_initiated_self_test_result.failed_trip_log_bits.b4 = TEST_PASS;

                     user_initiated_self_test_result.failed_trip_log_bits.b3 = TEST_PASS;
                   #endif

                   #if AF_CONFIGURATION != AF_NOT_ENABLED
                     user_initiated_self_test_result.failed_trip_log_bits.b2 = log_HF_min_OK;

                     user_initiated_self_test_result.failed_trip_log_bits.b1 = hf_sense_get_fault_flag();

                     user_initiated_self_test_result.failed_trip_log_bits.b0 = log_hf_min_detector_stuck_error_flag;
                   #else
                     user_initiated_self_test_result.failed_trip_log_bits.b2 = TEST_PASS;

                     user_initiated_self_test_result.failed_trip_log_bits.b1 = TEST_PASS;

                     user_initiated_self_test_result.failed_trip_log_bits.b0 = TEST_PASS;
                   #endif
                 }
                 else
                 {
                     user_initiated_self_test_result.failed_trip_log_bits.b7 = 1;

                     user_initiated_self_test_result.failed_trip_log_bits.b6 = 1;

                   #if GF_CONFIGURATION != GF_NOT_ENABLED
                     user_initiated_self_test_result.failed_trip_log_bits.b5 = ptt_ground_fault_coil_test;

                     user_initiated_self_test_result.failed_trip_log_bits.b4 = get_ground_fault_bias_error_flag();

                   #else
                     user_initiated_self_test_result.failed_trip_log_bits.b5= TEST_PASS;

                     user_initiated_self_test_result.failed_trip_log_bits.b4 = TEST_PASS;
                   #endif
                     user_initiated_self_test_result.failed_trip_log_bits.b3 = get_sequence_number_error_flag();

                     user_initiated_self_test_result.failed_trip_log_bits.b2 = get_interrupt_check_error_flag();

                     user_initiated_self_test_result.failed_trip_log_bits.b1 = get_data_overrun_error_flag();

                     user_initiated_self_test_result.failed_trip_log_bits.b0 = 0;
                 }
             }
             break;

        case PTT_COUNTER_KICKOFF_VALUE:
        case PTT_COUNTER_12:
        case PTT_COUNTER_10:
        case PTT_COUNTER_9:
        case PTT_COUNTER_8:
        case PTT_COUNTER_7:
        case PTT_IDLE_STATE:

            break;

        default:
            user_initiated_self_test_state = PTT_IDLE_STATE;
            break;
    }

    return;
}


void initiate_ptt(void)
{
    if( user_initiated_self_test_state == PTT_IDLE_STATE)
    {
        user_initiated_self_test_state = PTT_COUNTER_KICKOFF_VALUE;
      #if AF_CONFIGURATION != AF_NOT_ENABLED
        hf_min_test_counter = 0;
        set_hf_sense_fault_flag(TEST_PASS);
        log_hf_min_detector_stuck_error_flag = TEST_PASS;
      #endif

      #if OVERLOAD_CONFIGURATION != OVERLOAD_NOT_ENABLED
        set_line_current_bias_error_flag(TEST_PASS);
      #endif

        #if GF_CONFIGURATION != GF_NOT_ENABLED
          ptt_ground_fault_integrator = 0;
          reset_ground_fault_bias_error_flag();
        #endif

        reset_sequence_number_error_flag();
        reset_data_overrun_error_flag();
        reset_interrupt_check_error_flag();
    }

    return;
}

#if AF_CONFIGURATION != AF_NOT_ENABLED
void self_test_pwm_init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* TIM1 interrupt Init */
  NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 0);
  NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
  NVIC_SetPriority(TIM1_CC_IRQn, 0);
  NVIC_EnableIRQ(TIM1_CC_IRQn);

  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 1;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
  TIM_BDTRInitStruct.BreakAFMode = LL_TIM_BREAK_AFMODE_INPUT;
  TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_DISABLE;
  TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
  TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
  TIM_BDTRInitStruct.Break2AFMode = LL_TIM_BREAK_AFMODE_INPUT;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_ENABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**TIM1 GPIO Configuration
  PA11 [PA9]   ------> TIM1_CH4
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void self_test_start_pwm(void)
{
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_EnableCounter(TIM1);
}

void self_test_stop_pwm(void)
{
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_DisableCounter(TIM1);
}

void self_test_set_test_HF_line_low(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = TEST_HF_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(TEST_HF_GPIO_Port, &GPIO_InitStruct);

    LL_GPIO_ResetOutputPin(TEST_HF_GPIO_Port, TEST_HF_Pin);
}
#endif

inline user_self_test_result_t self_test_get_user_initiated_test_result(void)
{
    return user_initiated_self_test_result;
}

inline void ptt_set_result_to_logged(void)
{
    user_initiated_self_test_result.result = USER_INITIATED_SELF_TEST_LOGGED;
}

inline ptt_state self_test_get_user_initiated_test_state(void)
{
    return user_initiated_self_test_state;
}

inline void ptt_reset_result(void)
{
    user_initiated_self_test_result.result = USER_NOT_INITIATED;

    return;
}

