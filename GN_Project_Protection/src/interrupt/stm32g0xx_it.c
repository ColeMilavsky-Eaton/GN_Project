/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 ***************************************************************************************************
 *  Written by:         Aaron Joseph
 *                      Eaton Electrical
 *                      1000 Cherrington Parkway
 *                      Pittsburgh, PA  15108-4312
 *                      (412) 893-3300
 *//**
 * @defgroup interrupt Interrupt controller
 *
 * @brief The interrupt component contains the interrupt service routines for the system
 *
 * # Overview
 * The interrupt component performs the following functions:
 *  - Provides functions for handling external interrupt events
 *  - Provides functions for handling timer interrupt events
 *  - handles zero crossing events
 *
 * # Description
 * For a list of the available peripheral interrupt handler names, refer to the startup file
 * (startup_stm32g0xx.s)
 *
 * @file stm32g0xx_it.c
 * @ingroup interrupt
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "utils.h"
#include "firmware_config_common.h"
#include "firmware.config"
#include "main_internal.h"
#include "stm32g0xx_it_.h"
#include "timer_internal.h"
#include "main_api.h"
#include "gpio_api.h"
#include "adc_api.h"
#include "task_manager_api.h"
#include "load_voltage_api.h"
#include "power_loss_detection_api.h"
#include "led_api.h"
#include "trip_api.h"
#include "self_test_api.h"
#include "dataAcq.h"
#include "secondary_solenoid_api.h"
#include "iwdg_api.h"
#include "gfci_api.h"
#include "firmware.config"


#if SOFTWARE_ZCD_CONFIGURATION == SOFTWARE_ZCD_ENABLED
volatile bool software_ZCD_interrupt_flag = FALSE;
#endif

primary_zcd_t primary_zcd;

u16 previous_half_cycle_interrupt_cnt;
u16 interrupt_cnt;
bool gf_bias_in_range_flag;
u32 half_cycle_period_count, previous_half_cycle_period_count = SI_TIMER_DEFAULT;
u32 measured_frequency;

extern bool startup_conf;

input_voltage_scenario_t input_voltage_scenario;

#if (!COMPLETE_LOSS_OF_ZCD_DISALBED)
u32 loss_of_zcd_counter;
#endif

#warning this external variable will eventually get removed once ioman is further developed. Only used now so gf could be tested
/*extern */u32 gf_data;
/*extern */u32 main_load_voltage[MAX_NUM_POLES];
//extern u32 af_data;
/*extern */u32 gn_data;
//extern u32 hf_sense_data;
/*extern */bool gf_polarity_positive;
//extern u32 gf_sample_integral;
/*extern */u32 gf_bias;
extern u32 hall_data_adc;
/*extern */bool trip_class_1_flag;
#if defined POWER_LOSS_DETECTION
//extern volatile u32 power_loss_detection_possibility_level;
#endif

#if GF_CONFIGURATION != GF_NOT_ENABLED
u32 hrgf_running_fault_counter = 0;
#endif

/*extern */u32 raw_sol_adc_value;
#if OVERLOAD_CONFIGURATION == OVERLOAD_ENABLED
extern u32 main_load_current_plus_bias;
extern u32 main_load_current_bias;
extern u32 main_load_current_data;
extern u32 main_load_current_positive_data;
extern u32 main_load_current_negative_data;
extern u32 main_thermistor_resistance;
#endif

#warning check all functions for empty param list
extern inline u16 it_get_previous_half_cycle_interrupt_count()
{
    return previous_half_cycle_interrupt_cnt;
}

extern u16 it_get_interrupt_count()
{
    return interrupt_cnt;
}

extern inline u32 it_get_measured_frequency()
{
    return measured_frequency;
}
#warning move to separate zero crossing component
void set_primary_zero_crossing()
{
    LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

    /* if the interrupt source was ZCD_1 */
    if(primary_zcd == PRIMARY_ZCD_1)
    {
        /* pass information to core */
        load_voltage_set_primary_pole(POLE_1);

#ifdef ZCD_2
        /* disable zcd 2 interrupt */
        EXTI_InitStruct.Line_0_31 = ZCD_2_EXTI_LINE;
        EXTI_InitStruct.LineCommand = DISABLE;
        EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
      #if SOFTWARE_ZCD_CONFIGURATION == SOFTWARE_ZCD_ENABLED
        EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
      #else
        EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
      #endif
        LL_EXTI_Init(&EXTI_InitStruct);
#endif
    }
    else
    {
        /* pass information to core */
        load_voltage_set_primary_pole(POLE_2);

        /* keep ZCD1 interrupt enabled so that if ZCD1 comes back can catch it.*/
        EXTI_InitStruct.Line_0_31 = ZCD_1_EXTI_LINE;
        EXTI_InitStruct.LineCommand = ENABLE;
        EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
      #if SOFTWARE_ZCD_CONFIGURATION == SOFTWARE_ZCD_ENABLED
        EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
      #else
        EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
      #endif
        LL_EXTI_Init(&EXTI_InitStruct);
    }

}
void zero_crossing_callback(void)
{
    half_cycle_period_count = LL_TIM_GetCounter(ZCD_TIMER);
    u32 data_collection_interval = 0;

    measured_frequency = FREQUENCY_MEASURE_CONSTANT/(half_cycle_period_count + previous_half_cycle_period_count + FREQUENCY_MEASURE_FUDGE_FACTOR);

    /* clear zcd timer */
    LL_TIM_SetCounter(ZCD_TIMER, 0);

    /* restart zcd timer */
    LL_TIM_EnableCounter(ZCD_TIMER);

  #if SOFTWARE_ZCD_CONFIGURATION == SOFTWARE_ZCD_ENABLED
    if(!software_ZCD_interrupt_flag)
    {
        LL_TIM_DisableCounter(SI_TIMER);

        // set a timer to a half cycle
        LL_TIM_SetAutoReload(SI_TIMER, (half_cycle_period_count + previous_half_cycle_period_count) >> 1);
        LL_TIM_SetCounter(SI_TIMER, 0);
        LL_TIM_EnableCounter(SI_TIMER);
    }
  #endif

    /** stop data collection timer */
    LL_TIM_DisableCounter(DATA_COLLECTION_TIMER);

    /* update data collection interval length */
    data_collection_interval = ((half_cycle_period_count >> COLLECTION_INTERVAL_SHIFT) -
            COLLECTION_INTERVAL_FUDGE_FACTOR);

    /* check min max */
    if(data_collection_interval > DATA_COLLECTION_TIMER_UPPER_LIMIT)
    {
        data_collection_interval = DATA_COLLECTION_TIMER_UPPER_LIMIT;
    }
    if (data_collection_interval > 0xFF)
    {
        data_collection_interval = 0xFF;
    }
    else if (data_collection_interval < DATA_COLLECTION_TIMER_LOWER_LIMIT)
    {
        data_collection_interval = DATA_COLLECTION_TIMER_LOWER_LIMIT;
    }

    /* set new period value for data collection timer */
    LL_TIM_SetAutoReload(DATA_COLLECTION_TIMER, data_collection_interval);

    /* force collection interrupt to occur after this interrupt returns */
    LL_TIM_SetCounter(DATA_COLLECTION_TIMER, (data_collection_interval - 1));

    /* re-enable data collection interrupt */
    LL_TIM_EnableCounter(DATA_COLLECTION_TIMER);

    /* save interrupt count from the half cycle that just ended */
    previous_half_cycle_interrupt_cnt = (interrupt_cnt & 0x000F);

    /* reset collection variables */
    if(load_voltage_get_primary_polarity() == POSITIVE)
    {
        // will roll over to 0 once first data collection interrupt
        interrupt_cnt = U16_MAX;
    }
    else
    {
        interrupt_cnt = 15;
    }

  #if SOFTWARE_ZCD_CONFIGURATION == SOFTWARE_ZCD_ENABLED
    /* update background sequence */
    previous_half_cycle_period_count = half_cycle_period_count;
  #endif

    /* run zcd loop */
    task_man_run_zcd_loop(load_voltage_get_primary_polarity());

    return;
}

void NMI_Handler(void)
{
  #warning nonmaskable interrupts need to be handled
    while (1)
    {
      #if LED_CONFIGURATION == SINGLE_RED_LED

        LED_RED_ON;
        timer_delay(200);

        LED_RED_OFF;
        timer_delay(200);
      #endif

      #if LED_CONFIGURATION == RGB_LED
        LED_RED_ON;
        LED_BLUE_OFF;
        LED_GREEN_OFF;
        timer_delay(200);

        LED_RED_OFF;
        LED_BLUE_OFF;
        LED_GREEN_OFF;
        timer_delay(200);
      #endif
    }
}


void HardFault_Handler(void)
{
    while (1)
    {
    }
}


void SysTick_Handler(void)
{

}

void EXTI4_15_IRQHandler(void)
{
  #if DEBUG_SYSTEM_TIMING
    gpio_set_output_pin(ZCD_TIMING_GPIO_Port, ZCD_TIMING_Pin);
  #endif

#ifdef ZCD_2
  static bool jump_back_to_primary_zcd1_flag = FALSE;
  static u32 jump_back_to_primary_zcd1_counter = 0;
  static u32 jump_back_to_primary_zcd1_timeout = 0;
#endif
    /* stop zcd timer */
    LL_TIM_DisableCounter(ZCD_TIMER);

#if defined POWER_LOSS_DETECTION
    LL_TIM_DisableCounter(PLD_TIMER);
    LL_TIM_SetAutoReload(PLD_TIMER, PLD_DEFAULT_TIMEOUT);
    LL_TIM_SetCounter(PLD_TIMER, 0);
    LL_TIM_EnableCounter(PLD_TIMER);
#endif

    /* if runt zcd */
    if(LL_TIM_GetCounter(ZCD_TIMER) < MIN_HALF_CYCLE_PERIOD_CNT)
    {
        /* restart zcd timer */
        LL_TIM_EnableCounter(ZCD_TIMER);
    }
    else
    {
    #ifdef ZCD_2
      /* check if ZCD1 is back when primary is ZCD2*/
      if((primary_zcd == PRIMARY_ZCD_2) && (LL_EXTI_IsActiveFallingFlag_0_31(ZCD_1_EXTI_LINE) != RESET) )
      {
          /* enable flag to start counter for timeout. */
          jump_back_to_primary_zcd1_flag = TRUE;

          /* if threshold reached */
          if(jump_back_to_primary_zcd1_counter >= JUMP_BACK_TO_PRIMARY_ZCD1_THRESHOLD)
          {
              /* reset primary ZCD */
              primary_zcd = PRIMARY_ZCD_NONE;
              jump_back_to_primary_zcd1_flag = FALSE;
          }
          else
          {
              /* increase counter */
              jump_back_to_primary_zcd1_counter++;

              /* reset flags before return, other wise will re-enter right away */
              LL_EXTI_ClearRisingFlag_0_31(ZCD_1_EXTI_LINE);
              LL_EXTI_ClearFallingFlag_0_31(ZCD_1_EXTI_LINE);

              return;
          }
      }
      /* check timeout */
      if(jump_back_to_primary_zcd1_flag)
      {
          jump_back_to_primary_zcd1_timeout++;
          if(jump_back_to_primary_zcd1_timeout >= 60)
          {
              jump_back_to_primary_zcd1_counter = 0;
              jump_back_to_primary_zcd1_flag = FALSE;
          }
      }

    #endif

      #if SOFTWARE_ZCD_CONFIGURATION == SOFTWARE_ZCD_ENABLED

        /* If source is software */
        if (software_ZCD_interrupt_flag)
        {
          #ifdef ZCD_2
            load_voltage_set_primary_polarity(NEGATIVE);
          #else
            load_voltage_set_primary_polarity(POSITIVE);
          #endif
        }
        else
        {
          #ifdef ZCD_2
            load_voltage_set_primary_polarity(POSITIVE);
          #else
            load_voltage_set_primary_polarity(NEGATIVE);
          #endif

            /* if no primary ZCD is set */
            if(primary_zcd == PRIMARY_ZCD_NONE)
            {
                primary_zcd = (LL_EXTI_IsActiveFallingFlag_0_31(ZCD_1_EXTI_LINE) != RESET) ?
                                                                      (PRIMARY_ZCD_1):(PRIMARY_ZCD_2);
                /* set primary zcd */
                set_primary_zero_crossing();
            }
        }
        /* execute zcd callback */
        zero_crossing_callback();
        software_ZCD_interrupt_flag = FALSE;

      #else
        /* Determine interrupt source. */
        /* If source is ZCD_1 or ZCD_2 and it was on a rising edge. */
        if ((LL_EXTI_IsActiveRisingFlag_0_31(ZCD_1_EXTI_LINE) != RESET) ||
            (LL_EXTI_IsActiveRisingFlag_0_31(ZCD_2_EXTI_LINE) != RESET))
        {
            /* set polarity to positive */
          #ifdef ZCD_2
            load_voltage_set_primary_polarity(NEGATIVE);
          #else
            load_voltage_set_primary_polarity(POSITIVE);
          #endif


            /* if no primary ZCD is set */
            if(primary_zcd == PRIMARY_ZCD_NONE)
            {
#warning rework how the setting of primary ZCD is done
                primary_zcd = (LL_EXTI_IsActiveRisingFlag_0_31(ZCD_1_EXTI_LINE) != RESET) ?
                                                                    (PRIMARY_ZCD_1):(PRIMARY_ZCD_2);
                /* set primary zcd */
                set_primary_zero_crossing();
            }
        }

        /* If source is ZCD_1 or ZCD_2 and it was on a falling edge*/
        else if ((LL_EXTI_IsActiveFallingFlag_0_31(ZCD_1_EXTI_LINE) != RESET) ||
                 (LL_EXTI_IsActiveFallingFlag_0_31(ZCD_2_EXTI_LINE) != RESET))
        {
            /* set polarity to negative */
          #ifdef ZCD_2
            load_voltage_set_primary_polarity(POSITIVE);
          #else
            load_voltage_set_primary_polarity(NEGATIVE);
          #endif

            /* if no primary ZCD is set */
            if(primary_zcd == PRIMARY_ZCD_NONE)
            {
                primary_zcd = (LL_EXTI_IsActiveFallingFlag_0_31(ZCD_1_EXTI_LINE) != RESET) ?
                                                                  (PRIMARY_ZCD_1):(PRIMARY_ZCD_2);
                /* set primary zcd */
                set_primary_zero_crossing();
            }
        }

        /* execute zcd callback */
        zero_crossing_callback();
      #endif
    }

  #if (!COMPLETE_LOSS_OF_ZCD_DISALBED)
    /* clear loss zcd counter */
    loss_of_zcd_counter = 0;
  #endif

    /* clear all interrupt flags, Even if only zcd is setup to only detect Falling, Rising still
     * needs to be cleared because software triggered interrupt will raise a rising edge detect
     * see STM32G0x0_Reference_Manual */
    LL_EXTI_ClearRisingFlag_0_31(ZCD_1_EXTI_LINE);
    LL_EXTI_ClearFallingFlag_0_31(ZCD_1_EXTI_LINE);

    LL_EXTI_ClearRisingFlag_0_31(ZCD_2_EXTI_LINE);
    LL_EXTI_ClearFallingFlag_0_31(ZCD_2_EXTI_LINE);

  #if DEBUG_SYSTEM_TIMING
    gpio_reset_output_pin(ZCD_TIMING_GPIO_Port, ZCD_TIMING_Pin);
  #endif

    return;
}

void zero_crossing_timer_irq_handler (void)
{
  #if (!COMPLETE_LOSS_OF_ZCD_DISALBED)
    u8 trip_code = 0;
    u32 loss_of_zcd_above_voltage_threshold_flag = FALSE;
  #endif
    LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

    /* stop zcd timer */
    LL_TIM_DisableCounter(ZCD_TIMER);

    /* disable software interrupt timer */
    LL_TIM_DisableCounter(SI_TIMER);

    /* enable both zero crossing interrupts */
    EXTI_InitStruct.Line_0_31 = ZCD_1_EXTI_LINE;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  #if SOFTWARE_ZCD_CONFIGURATION == SOFTWARE_ZCD_ENABLED
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  #else
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  #endif
    LL_EXTI_Init(&EXTI_InitStruct);
  #ifdef ZCD_2
    EXTI_InitStruct.Line_0_31 = ZCD_2_EXTI_LINE;
    LL_EXTI_Init(&EXTI_InitStruct);
  #endif


    /* clear primary zcd */
    primary_zcd = PRIMARY_ZCD_NONE;

    /* pass information to core */
    load_voltage_set_primary_pole(PRIMARY_POLE_NOT_DEFINED);

    /* polarity is unknown */
    load_voltage_set_primary_polarity(POLARITY_UNKNOWN);

    /* update zcd timeout length to default value */
    LL_TIM_SetAutoReload(ZCD_TIMER, ZCD_DEFAULT_TIMEOUT);

    /* clear zcd timer */
    LL_TIM_SetCounter(ZCD_TIMER, 0);

    /* restart zcd timer */
    LL_TIM_EnableCounter(ZCD_TIMER);

    /* clear interrupt flag */
    LL_TIM_ClearFlag_UPDATE(ZCD_TIMER);

#if (!COMPLETE_LOSS_OF_ZCD_DISALBED)

#if (BOARD_CONFIGURATION != BOARD_SB2)
    /* check if pen tool is connected */
    if(gpio_read_input_pin(SPI_MISO_GPIO_Port, SPI_MISO_Pin) !=  GPIO_PIN_SET)
#endif
    {
        loss_of_zcd_counter++;

        if(loss_of_zcd_counter >= COMPLETE_LOSS_OF_ZCD_THRESHOLD)
        {
            trip_code = COMPLETE_LOSS_OF_ZCD_TRIP;
            trip(&trip_code, 1);
        }
    }

#endif

    /* update watchdog so that watchdog doesn't timeout and clear loss_of_zcd_counter*/
    iwdg_kick_watchdog();

    return;
}

void io_man_collect_data(u32 interrupt_count)
{
  #if GF_CONFIGURATION != GF_NOT_ENABLED
    static u32 gf_bias_previous_half_cycle = 2048;
    static u32 gf_bias_current_half_cycle = 2048;
    u32 gf_bias_temp = 0; // this temporary value is used to store temporary gf bias value, check if in range or not before storing in the global gf_bias
    s16 signed_gf = 0;
    u8 trip_code = 0;
  #endif

#if GF_CONFIGURATION != GF_NOT_ENABLED

    /* read gf data plus bias*/
    (void)adc_read_sequence(GF_ADC_CHANNEL);
    signed_gf = (s16)adc_get_channel_data(GF_ADC_CHANNEL);

    /* check for high resistance ground fault condition */
    if(gfci_check_for_hrgf((u32)signed_gf) == STATUS_FAULT_DETECTED)
    {
        /*increase fault counter*/
        hrgf_running_fault_counter++;
    }
    else if( hrgf_running_fault_counter > 0)
    {
        /*decrease fault counter*/
        hrgf_running_fault_counter--;
    }

    /*trip if counter reaches the max value*/
    if(hrgf_running_fault_counter >= HRGF_RUNNING_TRIP_THRESHOLD)
    {
        trip_code = HRGF_RUNNING_TRIP;
        trip_class_1_flag = TRUE;
        #if NUM_POLES_CONFIGURATION == SINGLE_POLE
            trip_instant(&trip_code, 1);
        #elif NUM_POLES_CONFIGURATION == TWO_POLE
            trip(&trip_code, 1);
        #else
            #error number of poles not configure
        #endif
    }

    /* set read bias pin */
    if(((self_test_get_user_initiated_test_state() == PTT_IDLE_STATE) &&
      #if AUTO_MONITOR_CONFIGURATION == AUTO_MONITOR_ENABLED
        (auto_monitor_get_state() == AUTO_MONITOR_WAIT) &&
      #endif
       ((interrupt_count == 15) || (interrupt_count == 31)) ))
    {
        /* set read bias pin */
        gpio_set_output_pin(GF_OFFSET_GPIO_Port, GF_OFFSET_Pin);
    }

#endif

  #if OVERLOAD_CONFIGURATION == OVERLOAD_ENABLED
    u32 load_current_bias_raw = 0;
    s32 signed_load_current = 0;

  #endif

  #if AF_CONFIGURATION != AF_NOT_ENABLED
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  #endif

  #if OVERLOAD_CONFIGURATION == OVERLOAD_ENABLED
    (void)adc_read_sequence(LOAD_CURRENT_CHANNEL);
    main_load_current_plus_bias = (u32)adc_get_channel_data(LOAD_CURRENT_CHANNEL);

    /* set current  bias measurement circuit */
    if(self_test_get_user_initiated_test_state() == IDLE_STATE)
    {
        gpio_set_output_pin(LOAD_CURRENT_BIAS_GPIO_Port, LOAD_CURRENT_BIAS_Pin);
    }

  #endif



  #if AF_CONFIGURATION != AF_NOT_ENABLED
    /** read arc fault and hf_sense input  */
    (void)adc_read_sequence(HF_MIN_ADC_CHANNEL | HF_SENSE_CHANNEL);

    /** flip the input by subtracting read value from the max adc counts */
    af_data = MAX_ADC_VALUE - adc_get_channel_data(HF_MIN_ADC_CHANNEL);

  #ifdef ADD_HF_NOISE_FOR_BENCHTOP_AF_TESTING
  #error ADDING NOISE TO AF INPUT IN ORDER TO BE ABLE TO TEST AFCI ON BENCHTOP TESTER
    af_data += main_load_current_plus_bias & 0xFF;
  #endif

    hf_sense_data = (u32)adc_get_channel_data(HF_SENSE_CHANNEL);
  #endif

    /* collect load voltage data*/
#ifdef ZCD_2
    (void)adc_read_sequence(V_PEAK_1_ADC_CHANNEL | V_PEAK_2_ADC_CHANNEL);
    #if (BOARD_CONFIGURATION == BOARD_SB2)
        // For SB2.0, P1 board the load voltage ADC detects the negative half cycle with 3.3V as reference
        // so in FW, the ADC values are reversed to use the original algorithm
        main_load_voltage[0] = (u32)((~adc_get_channel_data(V_PEAK_1_ADC_CHANNEL) & 0x0FFF) * LOAD_VOLTAGE_FACTOR);
        main_load_voltage[1] = (u32)((~adc_get_channel_data(V_PEAK_2_ADC_CHANNEL) & 0x0FFF) * LOAD_VOLTAGE_FACTOR);
    #else
        main_load_voltage[0] = (u32)(adc_get_channel_data(V_PEAK_1_ADC_CHANNEL) * LOAD_VOLTAGE_FACTOR);
        main_load_voltage[1] = (u32)(adc_get_channel_data(V_PEAK_2_ADC_CHANNEL) * LOAD_VOLTAGE_FACTOR);
    #endif
#else
    (void)adc_read_sequence(V_PEAK_1_ADC_CHANNEL);
    main_load_voltage[0] = (u32)(adc_get_channel_data(V_PEAK_1_ADC_CHANNEL) * LOAD_VOLTAGE_FACTOR);
#endif

  #if AF_CONFIGURATION != AF_NOT_ENABLED
    /** set the HF_MIN pin to output a 0. This drains the min detect circuit; resetting the value it holds */
    GPIO_InitStruct.Pin = HF_MIN_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(HF_MIN_GPIO_Port, &GPIO_InitStruct);

    LL_GPIO_ResetOutputPin(HF_MIN_GPIO_Port, HF_MIN_Pin);

  #endif

  #if OVERLOAD_CONFIGURATION == OVERLOAD_ENABLED

    if(self_test_get_user_initiated_test_state() == IDLE_STATE)
    {
        /* if measuring shunt resistor */
        if((interrupt_cnt == 12) || (interrupt_cnt == 28))
        {
            (void)adc_read_sequence(SHUNT_TEMPERATURE_CHANNEL);
            main_thermistor_resistance = (u32)adc_get_channel_data(SHUNT_TEMPERATURE_CHANNEL);
        }

        #if GN_CONFIGURATION != GN_CONFIG_1_CT
        /* add delay time for circuit to settle, If GN 1CT is enabled. this delay is not needed  */
        timer_delay(65);
       #endif

        (void)adc_read_sequence(LOAD_CURRENT_CHANNEL);
        load_current_bias_raw = (u32)adc_get_channel_data(LOAD_CURRENT_CHANNEL);

        gpio_reset_output_pin(LOAD_CURRENT_BIAS_GPIO_Port, LOAD_CURRENT_BIAS_Pin);

        #warning this bias adjustment will be different on CH breakers
        load_current_bias_raw = load_current_bias_raw + (load_current_bias_raw >> 7) +(load_current_bias_raw >> 8);
        load_current_bias_raw = load_current_bias_raw - (load_current_bias_raw >> 4);

        main_load_current_bias = load_current_bias_raw;
    }

    /* calculate signed load current */
    signed_load_current = (s32)main_load_current_plus_bias - (s32)main_load_current_bias;

    /* prepare data */
    main_load_current_data = ABS(signed_load_current);

    if(signed_load_current > 0)
    {
        main_load_current_positive_data = signed_load_current;
        main_load_current_negative_data = 0;
    }
    else
    {
        main_load_current_negative_data = (-signed_load_current);
        main_load_current_positive_data = 0;
    }

  #endif

  #if AF_CONFIGURATION != AF_NOT_ENABLED
    /** set HF_MIN input pin back to analog input so that the next iteration of min detect can start */
    GPIO_InitStruct.Pin = HF_MIN_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(HF_MIN_GPIO_Port, &GPIO_InitStruct);
#endif

#if GF_CONFIGURATION != GF_NOT_ENABLED

    /* read bias */
    if(((self_test_get_user_initiated_test_state() == PTT_IDLE_STATE) &&
      #if AUTO_MONITOR_CONFIGURATION == AUTO_MONITOR_ENABLED
        (auto_monitor_get_state() == AUTO_MONITOR_WAIT) &&
      #endif
       ((interrupt_count == 15) || (interrupt_count == 31)) ))
    {
        (void)adc_read_sequence(GF_ADC_CHANNEL);

        /* reset the offset pin after getting the bias */
        gpio_reset_output_pin(GF_OFFSET_GPIO_Port, GF_OFFSET_Pin);

        /* store values */
        gf_bias_previous_half_cycle = gf_bias_current_half_cycle;
        gf_bias_current_half_cycle = adc_get_channel_data(GF_ADC_CHANNEL);

        /* Do average */
        gf_bias_temp = (gf_bias_previous_half_cycle + gf_bias_current_half_cycle) >> 1;

        if( (gf_bias_temp > GROUND_FAULT_BIAS_MAX) ||
            (gf_bias_temp < GROUND_FAULT_BIAS_MIN))
        {
            gf_bias_in_range_flag = FALSE;
        }
        else
        {
            gf_bias_in_range_flag = TRUE;

            /* update gf_bias if value is in range */
            gf_bias = gf_bias_temp;
        }
    }

    /* remove bias to get only gf data*/
    signed_gf -= (s16)gf_bias;

    /* get polarity information */
    if(signed_gf > 0)
    {
        gf_polarity_positive = 1;
        gf_data = (u32)(signed_gf);
    }
    else
    {
        gf_polarity_positive = 0;
        gf_data = (u32)(-signed_gf);
    }

    /* update gf data with factor before sending to breaker core */
    gf_data *= GROUND_FAULT_FACTOR;

#endif
#if BOARD_CONFIGURATION == BOARD_SB2
    // read HALL input value before ZCD
    if((interrupt_count == 15) || (interrupt_count == 31))
    {
        (void)adc_read_sequence(HALL_IN_ADC_CHANNEL);
        hall_data_adc = (u32)adc_get_channel_data(HALL_IN_ADC_CHANNEL);
    }
#endif
    return;
}

void data_collection_callback()
{

}


void data_collection_irq_handler (void)
{
  #if DEBUG_SYSTEM_TIMING
    gpio_set_output_pin(DATA_COLLECT_TIMING_GPIO_Port, DATA_COLLECT_TIMING_Pin);
  #endif
    //u8 trip_code = 0;
    /* clear collection interrupt flag */
    LL_TIM_ClearFlag_UPDATE(DATA_COLLECTION_TIMER);

    interrupt_cnt++;

    /* if at the last interrupt of the halfcycle*/
    if((interrupt_cnt & 0x0F) == 0x0F)
    {
        /* extend timeout to keep next interrupt from coming close to zero crossing */
        LL_TIM_DisableCounter(DATA_COLLECTION_TIMER);
        LL_TIM_SetCounter(DATA_COLLECTION_TIMER, 0);
        LL_TIM_SetAutoReload(DATA_COLLECTION_TIMER, DATA_COLLECTION_DEFAULT_TIMEOUT);
        LL_TIM_EnableCounter(DATA_COLLECTION_TIMER);
    }
    /* check if interrupt count is within the expected range (0 to 31)*/
    if(interrupt_cnt < (NUM_INTERRUPTS_PER_HALFCYCLE << 1) )
    {
#if BOARD_CONFIGURATION != BOARD_SB2
    /* for auto monitor to turn on solenoid. The reason it is placed here and not inside self test
     * (auto monitor) itself is because we want to turn on the solenoid in the very beginning of
     * interrupt count 15 and not wait till task_man is running.  */
        if((auto_monitor_get_trip_test_flag() == TRUE) && (interrupt_cnt & 0x0F) == 15)
        {
          #if NUM_POLES_CONFIGURATION == SINGLE_POLE
            #if SOLENOID_CONFIGURATION == SOLENOID_DUAL
              if((auto_monitor_get_cycle() == AUTO_MONITOR_TRIP_TEST_FIRST_TRY_CYCLE) ||
                 (auto_monitor_get_cycle() == AUTO_MONITOR_TRIP_TEST_SECOND_TRY_CYCLE) )
              {
                  TRIP_1_ON;
              }
              else if ((auto_monitor_get_cycle() == AUTO_MONITOR_TRIP_TEST_THIRD_TRY_CYCLE) ||
                       (auto_monitor_get_cycle() == AUTO_MONITOR_TRIP_TEST_FOURTH_TRY_CYCLE) )
              {
                  TRIP_2_ON;
              }
            #else
              TRIP_1_ON;
              TRIP_2_ON;
            #endif
          #elif NUM_POLES_CONFIGURATION == TWO_POLE
                 TRIP_1_ON;
         #endif
        }
#else
    #ifdef SB20_PUL
         // SB2.0, collect the signal at SOL_ADC
          if((auto_monitor_get_trip_test_flag() == TRUE) && interrupt_cnt < 15)
          {
              (void)adc_read_sequence(SOL_ADC_CHANNEL);
               raw_sol_adc_value = (s16)adc_get_channel_data(SOL_ADC_CHANNEL);
          }
    #endif
#endif
        io_man_collect_data(interrupt_cnt);

        #if BOARD_CONFIGURATION == BOARD_SB2
        determine_input_scenario(interrupt_cnt);
        #endif

        /* trigger task manager to run component task loop */
        task_man_run_task_loop(interrupt_cnt);

    #if GN_CONFIGURATION == GN_CONFIG_2_CT
       if( (interrupt_cnt & 0x0F) == 8 || (interrupt_cnt & 0x0F) == 10 )
        {
            (void)adc_read_sequence(GF_ADC_CHANNEL);
            gn_data = (u32)adc_get_channel_data(GF_ADC_CHANNEL);
        }
    #endif

      #if (STM32CUBE_MONITOR_SNAPSHOT_MODE)
        #warning DEBUG SNAPSHOT MODE ENABLED
        /* send snapshot data to stm32cube_monitor. */
        DumpTrace();
      #endif
      #if DEBUG_SYSTEM_TIMING
        gpio_reset_output_pin(DATA_COLLECT_TIMING_GPIO_Port, DATA_COLLECT_TIMING_Pin);
      #endif
    }
  return;
}


void interrupt_init_component(void)
{
    primary_zcd = PRIMARY_ZCD_NONE;
    interrupt_cnt = 0;
    gf_bias_in_range_flag = TRUE;
  #if (!COMPLETE_LOSS_OF_ZCD_DISALBED)
    loss_of_zcd_counter = 0;
  #endif

    return;

}

void determine_input_scenario(u32 interrupt_count)
{
    static u32 max_cycle_voltage[MAX_NUM_POLES] = {0,0,0};
    static u8 peak_int_location[MAX_NUM_POLES] = {0,0,0}; //moved to global for ST_MON debugging.
    static input_voltage_scenario_t potential_input_voltage_scenario = SCENARIO_NOT_DEFINED;
    static u8 consistent_scenario_counter = 0;
    u8 primary_peak = 0;
    u8 secondary_peak = 0;
    // check if scenario is defined
    if( ((input_voltage_scenario == SCENARIO_NOT_DEFINED) || (input_voltage_scenario == SCENARIO_UNEXPECTED_PHASE))
    	&& (primary_zcd != PRIMARY_ZCD_NONE)
	  )
    {
    	//find_voltage_peak_loc(interrupt_count);
        // if single pole breaker

    	for (u8 i = 0; i< MAX_NUM_POLES; i++)
    	{
    		if(main_load_voltage[i] > max_cycle_voltage[i])
    		{
    			max_cycle_voltage[i] = main_load_voltage[i];
    			peak_int_location[i] = interrupt_count;
    		}
    	}

    	if(interrupt_count == 31)
    	{

	#if NUM_POLES_CONFIGURATION == SINGLE_POLE

			consistent_scenario_counter = INPUT_SCENARIO_DEBOUNCER;
			potential_input_voltage_scenario = SCENARIO_1P;

	#elif NUM_POLES_CONFIGURATION == TWO_POLE

			if(primary_zcd == PRIMARY_ZCD_1)
			{
				primary_peak = 0;
				secondary_peak = 1;
			}
			else
			{
				primary_peak = 1;
				secondary_peak = 0;
			}

			//first verify primary peak is within expected primary position threshold. and at a high enough voltage.
			if( ( (peak_int_location[primary_peak] < PRIMARY_PEAK_MIN) || (peak_int_location[primary_peak] > PRIMARY_PEAK_MAX) )
				|| (max_cycle_voltage[primary_peak] < LOAD_VOLTAGE_MINIMUM_VOLTAGE_THRESHOLD)
			  )
			{
				consistent_scenario_counter = 0;
				potential_input_voltage_scenario = SCENARIO_NOT_DEFINED;
			}
			//after verifying primary ensure secondary has the minimum load voltage to be detected.
			else if(max_cycle_voltage[secondary_peak] < LOAD_VOLTAGE_MINIMUM_NO_VOLTAGE_THRESHOLD)
			{
				consistent_scenario_counter = (potential_input_voltage_scenario == SCENARIO_2P_ONLY_1P_CONNECTED) ? ++consistent_scenario_counter:0;
				potential_input_voltage_scenario = SCENARIO_2P_ONLY_1P_CONNECTED;
			}
			// check if 180 out of phase scenario
			else if((peak_int_location[secondary_peak] >= LEAD_180_PEAK_MIN) && (peak_int_location[secondary_peak] <= LEAD_180_PEAK_MAX) )
			{
				consistent_scenario_counter = (potential_input_voltage_scenario == SCENARIO_2P_PRIMARY_LEAD_180) ? ++consistent_scenario_counter:0;
				potential_input_voltage_scenario = SCENARIO_2P_PRIMARY_LEAD_180;
			}
			else if((peak_int_location[secondary_peak] >= LEAD_120_PEAK_MIN) && (peak_int_location[secondary_peak] <= LEAD_120_PEAK_MAX) )
			{
				consistent_scenario_counter = (potential_input_voltage_scenario == SCENARIO_2P_PRIMARY_LEAD_120) ? ++consistent_scenario_counter:0;
				potential_input_voltage_scenario = SCENARIO_2P_PRIMARY_LEAD_120;
			}
			else if((peak_int_location[secondary_peak] >= LEAD_240_PEAK_MIN) && (peak_int_location[secondary_peak] <= LEAD_240_PEAK_MAX) )
			{
				consistent_scenario_counter = (potential_input_voltage_scenario == SCENARIO_2P_PRIMARY_LEAD_240) ? ++consistent_scenario_counter:0;
				potential_input_voltage_scenario = SCENARIO_2P_PRIMARY_LEAD_240;
			}
			else if((peak_int_location[secondary_peak] >= PRIMARY_PEAK_MIN) && (peak_int_location[secondary_peak] <= PRIMARY_PEAK_MAX) )
			{
				consistent_scenario_counter = (potential_input_voltage_scenario == SCENARIO_2P_PRIMARY_SAME_PHASE) ? ++consistent_scenario_counter:0;
				potential_input_voltage_scenario = SCENARIO_2P_PRIMARY_SAME_PHASE;
			}
			//secondary being not present or aligned with primary are already handled.
			//This would mean the secondary was high enough to be detected but is not in an expected phase.
			else
			{
				consistent_scenario_counter = (potential_input_voltage_scenario == SCENARIO_UNEXPECTED_PHASE) ? ++consistent_scenario_counter:0;
				potential_input_voltage_scenario = SCENARIO_UNEXPECTED_PHASE;
			}

	#elif NUM_POLES_CONFIGURATION == THREE_POLE
	#error this has not been tested. its just a pace holder for future development on 3 pole.
				consistent_scenario_flag = TRUE;
				potential_input_voltage_scenario = SCENARIO_3P;

	#endif
			if(consistent_scenario_counter >= INPUT_SCENARIO_DEBOUNCER)
			{
				input_voltage_scenario = potential_input_voltage_scenario;
			}

			//clear values for next int.
			for (u8 i = 0; i< MAX_NUM_POLES; i++)
			{
				max_cycle_voltage[i] = 0;
			    peak_int_location[i] = 0;
			}
		}
    }

    return;
}

inline input_voltage_scenario_t it_get_input_voltage_scenario(void)
{
    return input_voltage_scenario;
}

void it_init_nvic(void)
{
    /* clear flag at starup */
    LL_EXTI_ClearRisingFlag_0_31(ZCD_1_EXTI_LINE);
    LL_EXTI_ClearFallingFlag_0_31(ZCD_1_EXTI_LINE);

#ifdef ZCD_2

    LL_EXTI_ClearRisingFlag_0_31(ZCD_2_EXTI_LINE);
    LL_EXTI_ClearFallingFlag_0_31(ZCD_2_EXTI_LINE);
#endif

    /* clear pending interrupt request */
    __NVIC_ClearPendingIRQ(EXTI4_15_IRQn);

    /* EXTI4_15_IRQn interrupt configuration */
    NVIC_SetPriority(EXTI4_15_IRQn, 0);
    NVIC_EnableIRQ(EXTI4_15_IRQn);

  return ;
}

inline primary_zcd_t it_get_primary_zcd(void)
{
    return primary_zcd;
}

inline bool it_get_gf_bias_in_range_flag(void)
{
    return gf_bias_in_range_flag;
}

#if SOFTWARE_ZCD_CONFIGURATION == SOFTWARE_ZCD_ENABLED
void software_interrupt_timer_irq_handler (void)
{
    software_ZCD_interrupt_flag = TRUE;
    /* write "1" to EXTI_SWIER*/
    if(primary_zcd == PRIMARY_ZCD_1)
    {
        LL_EXTI_GenerateSWI_0_31(ZCD_1_EXTI_LINE);
    }
#ifdef ZCD_2
    else
    {
        LL_EXTI_GenerateSWI_0_31(ZCD_2_EXTI_LINE);
    }
#endif
    LL_TIM_DisableCounter(SI_TIMER);
    LL_TIM_SetAutoReload(SI_TIMER, SOFTWARE_ZCD_DEFAULT_TIMEOUT);

    LL_TIM_SetCounter(SI_TIMER, 0);
    LL_TIM_EnableCounter(SI_TIMER);
    LL_TIM_ClearFlag_UPDATE(SI_TIMER);
}
#endif

#if defined POWER_LOSS_DETECTION
void power_loss_detection_timer_irq_handler (void)
{
    LL_TIM_DisableCounter(PLD_TIMER);
    LL_TIM_SetAutoReload(PLD_TIMER, PLD_DEFAULT_TIMEOUT);

    gpio_set_output_pin(PWRSENSE_GPIO_Port, PWRSENSE_Pin);
    LL_TIM_SetCounter(PLD_TIMER, 0);
    LL_TIM_EnableCounter(PLD_TIMER);
    LL_TIM_ClearFlag_UPDATE(PLD_TIMER);

    // JF TODO: We need to determine if we should add the startup config to this as well
	// That way if we detect a power loss but somehow do not loose power we could still
	// default the breaker to open. This may require further testing however as it will
	// also cause us to die quicker if we are loosing power.
	if(!startup_conf)
	{
		request_open_secondary_solenoid();
	}
}
#endif
