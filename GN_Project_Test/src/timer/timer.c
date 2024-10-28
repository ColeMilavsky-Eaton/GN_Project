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
 * @defgroup timer Timer Component
 *
 * @brief The timer component handles the setup of the timers used by the system
 *
 * # Overview
 * The timer component performs the following functions:
 *  - initialization of timers
 *
 * # Description
 * The timer component contains code related to the timers used by the system. This component only
 * handles the initialization of the timers. The callbacks associated with a timer event are
 * contained elsewhere.
 *
 * @file timer.c
 * @ingroup timer
 *
 *//*
 *
 **************************************************************************************************/
#include "timer_.h"
#include "main_internal.h"
#include "iwdg_api.h"

void timer_zcd_timer_init (void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM14);

    /* TIM14 interrupt Init */
    NVIC_SetPriority(TIM14_IRQn, 0);
    NVIC_EnableIRQ(TIM14_IRQn);

    /* set update source to over flow only. */
    LL_TIM_SetUpdateSource(TIM14, LL_TIM_UPDATESOURCE_COUNTER);

    /* set timer configuration */
    TIM_InitStruct.Prescaler = 64-1;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = ZCD_DEFAULT_TIMEOUT;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;

    LL_TIM_Init(TIM14, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM14);

    /* enable update interrupt. */
    LL_TIM_EnableIT_UPDATE(TIM14);

    /* enable counter */
    LL_TIM_EnableCounter(TIM14);

    return;
}

void timer_data_collection_timer_init (void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);

  /* TIM16 interrupt Init */
  NVIC_SetPriority(TIM16_IRQn, 0);
  NVIC_EnableIRQ(TIM16_IRQn);

  /* set update source to over flow only. */
  LL_TIM_SetUpdateSource(TIM16, LL_TIM_UPDATESOURCE_COUNTER);

  TIM_InitStruct.Prescaler = 64-1;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = DATA_COLLECTION_DEFAULT_TIMEOUT;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM16, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM16);

  /* enable update interrupt. */
  LL_TIM_EnableIT_UPDATE(TIM16);

  /* enable counter */
  LL_TIM_EnableCounter(TIM16);

  return;
}

#if SOFTWARE_ZCD_CONFIGURATION == SOFTWARE_ZCD_ENABLED
void software_ZCD_timer_init (void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM17);

    /* TIM17 interrupt Init */
    NVIC_SetPriority(TIM17_IRQn, 0);
    NVIC_EnableIRQ(TIM17_IRQn);

    /* set update source to over flow only. */
    LL_TIM_SetUpdateSource(TIM17, LL_TIM_UPDATESOURCE_COUNTER);

    /* set timer configuration */
    TIM_InitStruct.Prescaler = 64-1;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = SOFTWARE_ZCD_DEFAULT_TIMEOUT;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;

    LL_TIM_Init(TIM17, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM17);

    /* enable update interrupt. */
    LL_TIM_EnableIT_UPDATE(TIM17);

    return;
}
#endif

#if defined POWER_LOSS_DETECTION
void power_loss_detection_timer_init (void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM15);

    /* TIM17 interrupt Init */
    NVIC_SetPriority(TIM15_IRQn, 0);
    NVIC_EnableIRQ(TIM15_IRQn);

    /* set update source to over flow only. */
    LL_TIM_SetUpdateSource(TIM15, LL_TIM_UPDATESOURCE_COUNTER);

    /* set timer configuration */
    TIM_InitStruct.Prescaler = 64-1;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = PLD_DEFAULT_TIMEOUT;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;

    LL_TIM_Init(TIM15, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM15);

    /* enable update interrupt. */
    LL_TIM_EnableIT_UPDATE(TIM15);

    return;
}
#endif

void ss_backup_timer_init (void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);

    /* TIM7 interrupt Init */
    NVIC_SetPriority(TIM7_LPTIM2_IRQn, 0);
    NVIC_EnableIRQ(TIM7_LPTIM2_IRQn);

    /* set update source to over flow only. */
    LL_TIM_SetUpdateSource(TIM7, LL_TIM_UPDATESOURCE_COUNTER);

    /* set timer configuration */
    TIM_InitStruct.Prescaler = 64-1;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = SS_DEFAULT_TIMEOUT;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;

    LL_TIM_Init(TIM7, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM7);

    /* enable update interrupt. */
    LL_TIM_EnableIT_UPDATE(TIM7);

    return;
}

void timer_delay(u64 delay_counts)
{
    while(delay_counts--)
    {
        /* watchdog refresh */
        iwdg_kick_watchdog();
    }

    return;
}

