#ifndef _IWDG_API_H
#define _IWDG_API_H
/**************************************************************************************************/
/*
 *                      Eaton Electrical
 *
 *                      Proprietary Information
 *                      (C) Copyright 2020
 *                      All rights reserved
 *
 **************************************************************************************************
 *  Written by:         Hank Sun
 *                      Eaton Electrical
 *                      1000 Cherrington Parkway
 *                      Pittsburgh, PA  15108-4312
 *                      (412) 893-3300
 *//**
 * @brief External interface for the iwdg component.
 *
 * @file iwdg_api.h
 * @ingroup iwdg
 *
 *//*
 *
 **************************************************************************************************/

#include "types.h"

/**************************************************************************************************/
/**
 * @brief Initialize the iwdg component.
 *
 * Initialize the iwdg component. Iwdg starts running after this function is called. Once iwdg
 * starts running, iwdg CANNOT be stopped.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
void iwdg_init_component(void);

/**************************************************************************************************/
/**
 * @brief Refresh (kick) the independent watchdog.
 *
 * Writes 0xAAAA to IWDG_KR register. If iwdg is initialized. This function should be called
 * periodically to prevent iwdg reset.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
inline void iwdg_kick_watchdog(void);

/**************************************************************************************************/
/**
 * @brief Check if system has reset due to iwdg.
 *
 * This function checks if IWDGRSTF (iwdg reset flag) in RCC_CSR register is set.
 *
 * @return TRUE If IWDGRSTF is set.
 *         FALSE If IWDGRSTF is not set.
 *
 * @exception none
 *
 **************************************************************************************************/
inline bool iwdg_is_iwdgrstf_active(void);

/**************************************************************************************************/
/**
 * @brief Clears all reset flags in RCC_CSR register.
 *
 * This function clears all reset flags in the RCC_CSR register.
 *
 * @return none
 *
 * @exception none
 *
 **************************************************************************************************/
inline void iwdg_clear_rst_flags(void);

#endif /*_IWDG_API_H */

