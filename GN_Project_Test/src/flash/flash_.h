#ifndef _FLASH__H
#define _FLASH__H
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
 * @brief Internal (to the component) interface for the flash component.
 *
 * @file flash_.h
 * @ingroup flash
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "flash_internal.h"

#define FLASH_CR_LOCK_BIT       (0x80000000)
#define FLASH_CR_PG_BIT         (0x00000001)
#define FLASH_CR_PER_BIT        (0x00000002)
#define FLASH_CR_PNB_BITS       (0x00001FF8)
#define FLASH_CR_START_BIT      (0x00010000)
#define FLASH_SR_BSY1_BIT       (0x00010000)
#define FLASH_SR_OPERR_BIT      (0x00000002)
#define FLASH_SR_PROGERR_BIT    (0x00000008)
#define FLASH_SR_WRPERR_BIT     (0x00000010)
#define FLASH_SR_PGAERR_BIT     (0x00000020)
#define FLASH_SR_SIZERR_BIT     (0x00000040)
#define FLASH_SR_PGSERR_BIT     (0x00000080)
#define FLASH_SR_MISSERR_BIT    (0x00000100)
#define FLASH_SR_FASTERR_BIT    (0x00000200)
#define FLASH_SR_ERROR_BITS     (FLASH_SR_OPERR_BIT|FLASH_SR_PROGERR_BIT|\
                                 FLASH_SR_WRPERR_BIT|FLASH_SR_PGAERR_BIT|\
                                 FLASH_SR_SIZERR_BIT|FLASH_SR_PGSERR_BIT|\
                                 FLASH_SR_MISSERR_BIT|FLASH_SR_FASTERR_BIT)
#define FLASH_SR_CLEAR_VALUE    (0xFFFFFFFF)

/* Flash unlock keys */
#define FLASH_UNLOCK_KEY_1      (0x45670123)
#define FLASH_UNLOCK_KEY_2      (0xCDEF89AB)

#define FLASH_LOCKED                          (READ_BIT(FLASH -> CR, FLASH_CR_LOCK_BIT) == FLASH_CR_LOCK_BIT)
#define FLASH_UNLOCKED                        (READ_BIT(FLASH -> CR, FLASH_CR_LOCK_BIT) != FLASH_CR_LOCK_BIT)
#define FLASH_BUSY                            (READ_BIT(FLASH -> SR, FLASH_SR_BSY1_BIT) == FLASH_SR_BSY1_BIT)
#define FLASH_WAIT_WHILE_FLASH_IS_BUSY()      while(FLASH_BUSY)
#define FLASH_WRITE_UNLOCK_KEY_1              (WRITE_REG(FLASH->KEYR, FLASH_UNLOCK_KEY_1))
#define FLASH_WRITE_UNLOCK_KEY_2              (WRITE_REG(FLASH->KEYR, FLASH_UNLOCK_KEY_2))
#define FLASH_SET_LOCK_BIT                    (SET_BIT(FLASH -> CR, FLASH_CR_LOCK_BIT))
#define FLASH_CLEAR_ERROR_FLAGS               (WRITE_REG(FLASH -> SR, FLASH_SR_CLEAR_VALUE))
#define FLASH_SET_PROGRAMMING_ENABLE_BIT      (SET_BIT(FLASH -> CR, FLASH_CR_PG_BIT))
#define FLASH_CLEAR_PROGRAMMING_ENABLE_BIT    (CLEAR_BIT(FLASH -> CR, FLASH_CR_PG_BIT))
#define FLASH_SET_PAGE_ERASE_BIT              (SET_BIT(FLASH -> CR, FLASH_CR_PER_BIT))
#define FLASH_CLEAR_PAGE_ERASE_BIT            (CLEAR_BIT(FLASH -> CR, FLASH_CR_PER_BIT))
#define FLASH_ERROR_EXISTS                    ((READ_BIT(FLASH -> SR, FLASH_SR_ERROR_BITS) & FLASH_SR_ERROR_BITS) != 0)
#define FLASH_START_ERASE                     (SET_BIT(FLASH -> CR, FLASH_CR_START_BIT))
#define FLASH_CLEAR_CR_PNB_BITS               (WRITE_REG(FLASH -> CR, READ_REG(FLASH -> CR) & (~FLASH_CR_PNB_BITS)))
#define FLASH_SET_CR_PNB_BITS_(_X)            (WRITE_REG(FLASH -> CR, READ_REG(FLASH -> CR) | ( _X << 3)))


#endif /*_FLASH__H*/

