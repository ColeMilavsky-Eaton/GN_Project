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
 * @defgroup iwdg Independent watch dog component
 *
 * @brief iwdg component handles the initialization and refreshing (kicking) of the independent
 * watchdog.
 *
 * The independent watchdog is configured to NOT use the window mode. System will reset and
 * iwdg reset flag (in RCC_CSR register) will set when iwdg counter counts to 0.
 *
 * Once iwdg starts running. the iwdg CANNOT be stopped.
 *
 * @file iwdg.c
 * @ingroup iwdg
 *
 *
 *//*
 *
 **************************************************************************************************/

#include "iwdg_.h"
#include "main_internal.h"

void iwdg_init_component(void)
{

  LL_IWDG_Enable(IWDG);
  LL_IWDG_EnableWriteAccess(IWDG);
  LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_4);
  LL_IWDG_SetReloadCounter(IWDG, 1725);
  while (LL_IWDG_IsReady(IWDG) != 1)
  {
  }

  /* not using window feature */
  LL_IWDG_SetWindow(IWDG, 4095);

  LL_IWDG_ReloadCounter(IWDG);

  return;
}

void iwdg_kick_watchdog(void)
{
    LL_IWDG_ReloadCounter(IWDG);

    return;
}

bool iwdg_is_iwdgrstf_active(void)
{
    return(LL_RCC_IsActiveFlag_IWDGRST() == 1);
}

#warning this function clears ALL reset flags, should this function move somewhere else?
void iwdg_clear_rst_flags(void)
{
    LL_RCC_ClearResetFlags();

    return;
}
