/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 **************************************************************************************************
 *  Written by:         Ming Wu
 *                      Innovation
 *                      Commercial & Residential Distribution Solutions (CRDS), Eaton
 *                      1000 Cherrington Parkway
 *                      Moon Township, PA 15108
 *//**
 * @defgroup power_loss_detection power_loss_detection component
 *
 * @brief power_loss_detection component monitors the load voltage and ZCD and
 * will report if a signal of power loss is present.

  * NOTE: 1.
  *
  * @file power_loss_detection.c
  * @ingroup power_loss_detection
  *
  *
  *//*
  *
  **************************************************************************************************/
#include "types.h"
#include "main_internal.h"
#include "gpio_api.h"
#include "power_loss_detection.h"
#include "stm32g0xx_it_api.h"
#include "firmware.config"
#include "string.h"

volatile u32 power_loss_detection_possibility_level;

u32 voltage_buffer1[MAX_NUM_POLES][NUM_SAMPLES_PER_CYCLE];
u32 voltage_buffer2[MAX_NUM_POLES][NUM_SAMPLES_PER_CYCLE];
u32* voltage_reference;
u32* voltage_live;

static bool pld_initialized_flag = FALSE;
u32 num_of_poles;
u32* voltage_p[MAX_NUM_POLES];

inline void set_power_loss_detection_level(u32 level)
{
    power_loss_detection_possibility_level = level;
}

inline u32 get_power_loss_detection_level ()
{
    return power_loss_detection_possibility_level;
}

void power_loss_detection_init(power_loss_detection_init_t power_loss_detection_init_params)
{
    if(pld_initialized_flag == FALSE)
    {
        voltage_reference = &voltage_buffer1[0][0];
        voltage_live = &voltage_buffer2[0][0];
        /* task type to assign tasks */
        task_t t = (task_t){.task_p = NULL, .it_bits = 0};

        num_of_poles = power_loss_detection_init_params.number_of_poles;
        voltage_p[0] = power_loss_detection_init_params.load_voltage_data_source1;
        voltage_p[1] = power_loss_detection_init_params.load_voltage_data_source2;
        voltage_p[2] = power_loss_detection_init_params.load_voltage_data_source3;
        /* assign task */
        t.task_p = power_loss_detection_it_task;
        t.it_bits = TSK_MAN_IT_ALL;
        task_man_add_task_to_schedule(t);

        pld_initialized_flag = TRUE;
    }
    return;
}

void power_loss_detection_zcd_callback()
{

}

void power_loss_detection_it_task(u32 it)
{
    u8 i;
    u32* temp, voltage_defactor, reference_it;

    // Here using the last cycle's voltage data as the reference
    // It could be some constant values (based on 120V RMS), but that need to
    // know the power scenario

    if(it == 0)
    {
        temp = voltage_reference;
        voltage_reference = voltage_live;
        voltage_live = temp;
    }

    // voltage samples near zero crossing are smaller and may be less accurate
    for(i = 0; i < num_of_poles; i++)
    {
        // remove the factor to use the raw ADC value since a big factor would increase the variation of ADC detection.
        voltage_defactor = *voltage_p[i] / LOAD_VOLTAGE_FACTOR;
        *(voltage_live + (i*NUM_SAMPLES_PER_CYCLE + it)) = voltage_defactor;
        // if a live voltage is less than a reference voltage significantly (1/2 of the reference)
        // increase the possibility level.
        // For the first cycle, the reference is zero
        reference_it = *(voltage_reference +(i*NUM_SAMPLES_PER_CYCLE + it));

        // Based on test on P1 board, ADC values lower than the BAR will not be compared
        if(reference_it > POWER_LOSS_DETECTION_ADC_VARIANCE_BAR)
        {
            if(voltage_defactor < (reference_it / 2))
            {
                power_loss_detection_possibility_level ++;
            }
            // when live is over 7/8 of the reference, decrease the possibility level
            // This is to handle occasional disturbances and prevent the level accumulate over time
            else if(voltage_defactor > (reference_it * 7 / 8))
            {
                power_loss_detection_possibility_level =  power_loss_detection_possibility_level > 0? (power_loss_detection_possibility_level-1):0;
            }
        }
    }

    if(power_loss_detection_possibility_level > POWER_LOSS_DETECTION_LEVEL_THRESHOLD)
    {
        // considered a power loss ,toggle a pin
        gpio_set_output_pin(PWRSENSE_GPIO_Port, PWRSENSE_Pin);
        power_loss_detection_possibility_level = 0;
    }

}
