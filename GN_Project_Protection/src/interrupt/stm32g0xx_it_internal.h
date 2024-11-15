#ifndef _STM32G0xx_IT_INTERNAL_H
#define _STM32G0xx_IT_INTERNAL_H
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
 * @brief internal (to CRDS breaker firmware) interface for the interrupt component.
 *
 * @file interrupt_internal.h
 * @ingroup interrupt
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "stm32g0xx_it_api.h"

void NMI_Handler(void); /* non- maskable interrupt handler */
void HardFault_Handler(void); /* hardfault interrupt handler */
void SysTick_Handler(void); /* system tick timer interrupt handler */

/**************************************************************************************************/
/**
 * @brief Interrupt handler for zero crossing input
 *
 * There is a primary and secondary Zero Crossing. The primary is used to control algorithm
   timing. The secondary is ignored. Whichever zero crossing is triggered first after power on
   becomes the primary. If the primary ever drops out, then pick it again
 *
 * * Two versions:
 * 1. software ZCD for 2p breaker
 *  In 2p breaker design, ZCD1 is the input from one phase while ZCD2 is from the other phase.
 *  Due to the hardware designs, in non split-phase condition the interrupts generated from ZCD1 or ZCD2
 *  are not aligned with the corresponding real zero crossings, so using the same approach as the one
 *  in 1p design is not valid anymore.
 *   @image html 120degree.png "Figure 1 - 120 degree of phase shift between two poles " height=500
 *
 *  However, the time between two same edges detected still equals one whole cycle. So we can insert a
 *  software zero crossing in the middle of the whole cycle to still use the design of 1p breaker.
 *
 *  To implement this, the firmware only detects one edge on ZCD1/ZCD2, which gives a correct reference of
 *  one whole cycle. Meanwhile, a separate timer called software interrupt timer is used to simulate a ZCD
 *  between two hardware ZCDs. The timer's auto load value is calculated and set at every hardware
 *  ZCD interrupt, the value is the average of last two half cycles, ie, one half cycle.
 *
 *  @image html softwareZCD.png "Figure 2 -Software ZCD implementation " height=500
 *
 *  The firmware starts with a setting that can only detect rising edges from ZCD1 and ZCD2,
 *  which is designed to pick up the first positive zero crossing from the 2 phases. When the
 *  first positive zero crossing is detected, it remembers the interrupt source (ZCD1 or ZCD2) as the primary ZCD
 *  and will use the setting thereafter. It will call set_primary_zero_crossing() to
 *  apply the setting.
 *
 *  2. Hardware ZCD for 1p breaker
 *  In 1p design, ZCD1 and ZCD2 are from the same phase and the interrupts aligned well with the real
 *  zero crossings. To pick the first zero crossing, the firmware starts with a setting that can detect
 *  both rising and falling edges from ZCD1 and ZCD2, when the first zero crossing is detected, it
 *  remembers the interrupt source and disable the other in set_primary_zero_crossing()
 *
 * @return none
 *
 * @requirement{xxx}
 *
 **************************************************************************************************/
void EXTI4_15_IRQHandler(void);

/**************************************************************************************************/
/**
 * @brief Disabled the unused input between ZCD1 and ZCD2 as the primary interrupt source is detected
 *
 * Two versions:
 * 1. software ZCD for 2p
 *    Depends on first zero crossing detected, set the detection edge (rising or falling) for hardware
 *    zero crossings. It also disable the ZCD2 if ZCD1 was picked, or ZCD1 if ZCD2 is chosen.
 *
 * 2. Hardware ZCD for 1p
 *   It simply disables the ZCD2 if ZCD1 was picked, or ZCD1 if ZCD2 is picked
 *
 * @return none
 *
 * @requirement{xxx}
 *
 **************************************************************************************************/
void set_primary_zero_crossing( void );

/**************************************************************************************************/
/**
 * @brief call back function in the zero crossing isr
 *
 * Two versions:
 * 1. software ZCD for 2p
 *     Get the counter of ZCD timer for the last half cycle, then average it with the counter collected
 *     from the half cycle before the last half cycle to set the software interrupt timer auto reload value.
 *     Also the ZCD timer counter is used to calculate and set data collection timer auto reload value.
 *     The ZCD timer auto reload value is also set to multiples of last half cycle.
 *     Also, during the software interrupt, check if the primary pole has voltage. If yes, then its a
 *     120/240 degree situation. In addition, check voltage peak1 and voltage peak2, to determine the phase shift
 *     If primary peak is close to 0v, primary is leading/lagging 180. If not then
 *       1.  If primary peak (voltage) is smaller than secondary peak (voltage). => primary is lagging 120
 *       2.  If primary peak (voltage) is larger than secondary peak (voltage). => primary is leading 120
 *
 *
 * 2. Hardware ZCD for 1p
 *    Get the counter of ZCD timer for the last half cycle, then use it to calculate and set data collection
 *    timer auto reload value. Also reset the ZCD timer auto reload value to multiples of last half cycle
 *
 * @return none
 *
 * @requirement{xxx}
 *
 **************************************************************************************************/
void zero_crossing_callback(void);

/**************************************************************************************************/
/**
 * @brief isr for software interrupt timer
 *
 * The isr generate a software interrupt in ZCD1 and ZCD2 then set the timer auto reload value to a
 * default value. The auto reload value will be adjusted at the next hardware interrupt handler.
 *
 *
 * @requirement{xxx}
 *
 **************************************************************************************************/
void software_interrupt_timer_irq_handler (void);

/**************************************************************************************************/
/**
 * @brief reset the zero crossing timer
 *
 * The isr will reset the ZCD selection and let the firmware select the primary ZCD again.
 * It should not be triggered in normal situation as the timer's counter is cleared at ZCD interrupt,
 * it will be only activated when hardware ZCD interrupts are missed two times in a row
 *
 *
 * @requirement{xxx}
 *
 **************************************************************************************************/
void zero_crossing_timer_irq_handler( void );
#endif /* _STM32G0xx_IT_INTERNAL_H */
