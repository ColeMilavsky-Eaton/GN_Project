#ifndef _TIMER_INTERNAL_H
#define _TIMER_INTERNAL_H
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
 * @brief internal (to CRDS breaker firmware) interface for the timer component.
 *
 * @file timer_internal.h
 * @ingroup timer
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "timer_api.h"
#include "task_manager_api.h"

#define CPU_FREQUENCY       (16000000)
#define TIMER16_PRESCALE    (64)
#define LINE_FREQUENCY_LOWER_LIMIT  (57)
#define LINE_FREQUENCY_UPPER_LIMIT  (63)
#define LINE_FREQUENCY_DEFAULT      (60)


#define NUMBER_OF_INTERRUPTS

#define ZCD_TIMER TIM14
#define zero_crossing_timer_irq_handler TIM14_IRQHandler

#define SS_BACKUP_TIMER TIM7
#define ss_backup_timer_irq_handler TIM7_LPTIM2_IRQHandler

#define DATA_COLLECTION_TIMER TIM16
#define data_collection_irq_handler TIM16_IRQHandler
#define DATA_COLLECTION_TIMER_UPPER_LIMIT ((u32)(CPU_FREQUENCY/(((float)TIMER16_PRESCALE) * NUM_INTERRUPTS_PER_HALFCYCLE * LINE_FREQUENCY_LOWER_LIMIT * 2.0)))
#define DATA_COLLECTION_TIMER_LOWER_LIMIT ((u32)(CPU_FREQUENCY/(((float)TIMER16_PRESCALE) * NUM_INTERRUPTS_PER_HALFCYCLE * LINE_FREQUENCY_UPPER_LIMIT * 2.0)))

#if SOFTWARE_ZCD_CONFIGURATION == SOFTWARE_ZCD_ENABLED
#define SI_TIMER TIM17
#define software_interrupt_timer_irq_handler TIM17_IRQHandler
#define SI_TIMER_UPPER_LIMIT ((u32)(CPU_FREQUENCY/(((float)TIMER16_PRESCALE) * LINE_FREQUENCY_LOWER_LIMIT * 2.0)))
#define SI_TIMER_DEFAULT     ((u32)(CPU_FREQUENCY/(((float)TIMER16_PRESCALE) * LINE_FREQUENCY_DEFAULT * 2.0)))
#define SI_TIMER_LOWER_LIMIT ((u32)(CPU_FREQUENCY/(((float)TIMER16_PRESCALE) * LINE_FREQUENCY_UPPER_LIMIT * 2.0)))
#endif

#if defined POWER_LOSS_DETECTION
#define PLD_TIMER TIM15
#define power_loss_detection_timer_irq_handler TIM15_IRQHandler
#endif
 #endif /* _TIMER_INTERNAL_H */
