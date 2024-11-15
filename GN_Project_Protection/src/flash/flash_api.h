#ifndef _FLASH_API_H
#define _FLASH_API_H
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
 * @brief External interface for the flash component.
 *
 * @file flash_api.h
 * @ingroup flash
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"

#define FLASH_PAGE_(x)          (x)
#define FLASH_STANDARD_WRITE_BYTES  (8)

/**************************************************************************************************/
/**
 * @brief Unlock flash memory's CR register.
 *
 * @return STATUS_OK if unlock successful.
 *         STATUS_FAIL if unlock unsuccessful.
 *
 **************************************************************************************************/
inline status_t flash_unlock(void);

/**************************************************************************************************/
/**
 * @brief Lock flash memory's CR register.
 *
 * @return STATUS_OK if Lock successful.
 *         STATUS_FAIL if lock unsuccessful.
 *
 **************************************************************************************************/
inline status_t flash_lock(void);

/**************************************************************************************************/
/**
 * @brief Write 8 bytes of data on flash memory.
 *
 * @param[in] src - The address that stores the data to write.
 * @param[out] dest - The address in the flash memory to write to. NOTE: This address must be
 *                          a value that is divisible by 8.
 *
 * @return STATUS_OK if write successful.
 *         STATUS_NULL_P if parameters are null pointers.
 *         STATUS_FAIL if address_from is not divisible by 8. Or write unsuccessful. Or timeout.
 *
 * @requirement{2010190904}
 *
 **************************************************************************************************/
status_t flash_write64(void* src, void* dest);

/**************************************************************************************************/
/**
 * @brief Erase a page of flash memory.
 *
 * @return STATUS_OK if erase successful.
 *         STATUS_FAIL if erase fail, or timeout.
 *
 * @requirement{2010190905}
 *
 **************************************************************************************************/
status_t flash_erase_page(u16 page_num);

#endif /*_FLASH_API_H*/

