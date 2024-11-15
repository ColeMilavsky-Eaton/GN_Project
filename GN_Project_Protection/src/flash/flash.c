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
 * @defgroup flash Flash component
 *
 * @brief The flash component performs read/write operations on flash memory.
 *
 * # Overview
 * The flash component performs the following functions:
 *  - provides a function to unlock flash memory .
 *  - provides a function to lock flash memory .
 *  - provides a function to write to flash memory .
 *  - provides a function to erase a page of flash memory .
 *
 *
 * @file flash.c
 * @ingroup flash
 *
 *
 *//*
 *
 **************************************************************************************************/
#include "types.h"
#include "main_internal.h"
#include "flash_.h"


status_t flash_unlock(void)
{
    status_t status = STATUS_OK;

    /* check if already unlocked. */
    if(!(FLASH_UNLOCKED))
    {
        /* write unlock sequence to flash key register */
        FLASH_WRITE_UNLOCK_KEY_1;
        FLASH_WRITE_UNLOCK_KEY_2;

        /* check again */
        if(!(FLASH_UNLOCKED))
        {
            status = STATUS_FAIL;
        }
    }

    return status;
}

status_t flash_lock(void)
{
    status_t status = STATUS_OK;

    /* set lock bit. */
    FLASH_SET_LOCK_BIT;

    /* check if lock is successful */
    if(!(FLASH_LOCKED))
    {
        status = STATUS_FAIL;
    }

    return status;
}

status_t flash_write64(void* src, void* dest)
{
    status_t status = STATUS_OK ;

    /* check null pointer */
    if((src == NULL) || (dest == NULL))
    {
        return STATUS_NULL_P;
    }

    /* check address alignment */
    if(( (u32)dest & (u32)0x07) != 0)
    {
        return  STATUS_FAIL;
    }

    /* unlock flash*/
    (void)flash_unlock();

    /* check if flash is busy. */
    FLASH_WAIT_WHILE_FLASH_IS_BUSY();

    /* clear error flags. */
    FLASH_CLEAR_ERROR_FLAGS;

    /* set PG bit and clear PER bit */
    FLASH_SET_PROGRAMMING_ENABLE_BIT;
    FLASH_CLEAR_PAGE_ERASE_BIT;

    /* write data */
    *(u64*)dest = *(u64*)src;

    /* check busy */
    FLASH_WAIT_WHILE_FLASH_IS_BUSY();

    /* lock flash */
    (void)flash_lock();

    /* check error */
    if(FLASH_ERROR_EXISTS)
    {
        status =  STATUS_FAIL;
    }

    return status;
}

status_t flash_erase_page(u16 page_num)
{
    status_t status = STATUS_OK ;

    /* unlock flash*/
    (void)flash_unlock();

    /* check if flash is busy. */
    FLASH_WAIT_WHILE_FLASH_IS_BUSY();

    /* clear error flags. */
    FLASH_CLEAR_ERROR_FLAGS;

    /* set PER bit and clear PG bit */
    FLASH_SET_PAGE_ERASE_BIT;
    FLASH_CLEAR_PROGRAMMING_ENABLE_BIT;

    /* set page to erase.*/
    FLASH_CLEAR_CR_PNB_BITS;
    FLASH_SET_CR_PNB_BITS_(page_num);

    /* start erase */
    FLASH_START_ERASE;

    /* lock flash */
    (void)flash_lock();

    /* check if flash is busy. */
    FLASH_WAIT_WHILE_FLASH_IS_BUSY();

    /* check error */
    if(FLASH_ERROR_EXISTS)
    {
        status = STATUS_FAIL;
    }

    return status;
}

