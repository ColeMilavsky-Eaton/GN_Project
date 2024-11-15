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
 * @defgroup micro_test micro_test component
 *
 * @brief This component handles the microcontroller self tests such as cpu test, flash test and
 * ram test.
 *
 *
 * @file micro_test.c
 * @ingroup micro_test
 *
 *
 *//*
 *
 **************************************************************************************************/

#include "types.h"
#include "utils.h"
#include "micro_test_.h"
#include "task_manager_api.h"
#include "stm32g0xx.h"
#include "stm32g0xx_it_api.h"
#include "crc_api.h"

/* For continuous CPU check */
const u32 u32_aa = MICRO_TEST_4_BYTES_AA;
const u32 u32_55 = MICRO_TEST_4_BYTES_55;

/* For continuous flash check */
u32 cal_chksum;
u8* current_calculated_address;
extern u32 __build_checksum;
u32* build_checksum_address;
extern u32 __version;

/* For continuous RAM check */
u32* current_testing_RAM_address;

void micro_test_init(void)
{
    /* For continuous flash check */
    cal_chksum = 0;
    current_calculated_address = (u8*)MICRO_TEST_PROGRAM_MEMORY_START_ADDRESS;
    build_checksum_address = &__build_checksum;

    /* For continuous RAM check */
    current_testing_RAM_address = (u32*)MICRO_TEST_RAM_START_ADDRESS;

    return;
}

status_t micro_test_continuous_flash_check(void)
{
    status_t crc_status = STATUS_UNKNOWN;

    /* calculate crc */
    crc_status = calculate_crc_32(current_calculated_address,
                                  MICRO_TEST_CONTI_FLASH_CHECK_SIZE_PER_CALL,
                                  cal_chksum,
                                  &cal_chksum
                                 );

    if(crc_status == STATUS_OK)
    {
        current_calculated_address += MICRO_TEST_CONTI_FLASH_CHECK_SIZE_PER_CALL;

        /* if completed calculating crc for the whole program section of flash */
        if(current_calculated_address >= (u8*)build_checksum_address )
        {
                /* check if crc match */
                if(__build_checksum == cal_chksum)
                {
                    /* reinitialize variable */
                    cal_chksum = 0;
                    current_calculated_address = (u8*)MICRO_TEST_PROGRAM_MEMORY_START_ADDRESS;
                }
                else
                {
                    crc_status = STATUS_FAIL;
                }
        }
    }



    return crc_status;
}

#warning where to run this function? data collection interrupt? task loop? is disabling interrupt neccessary
status_t micro_test_continuous_RAM_check(void)
{
    /* disable interrupt */
    __disable_irq();

    /* local variables have to be on the cpu registers */
    register u32 temp_stored_data = 0;
    register u32 temp_stored_data_inverse = 0;
    register u32* current_testing_RAM_address_copy = current_testing_RAM_address;
    register u32 loop_counter = 0;

    /* read the data that is currently in ram and its inverse */
    temp_stored_data = *current_testing_RAM_address_copy;
    temp_stored_data_inverse = ~(*current_testing_RAM_address_copy);

    /* test clear*/
    *current_testing_RAM_address_copy = MICRO_TEST_RAM_TEST_LOW;
    if( *current_testing_RAM_address_copy != MICRO_TEST_RAM_TEST_LOW)
    {
        return STATUS_FAIL;
    }

    /* test all high */
    *current_testing_RAM_address_copy = MICRO_TEST_RAM_TEST_HIGH;
    if( *current_testing_RAM_address_copy != MICRO_TEST_RAM_TEST_HIGH)
    {
        return STATUS_FAIL;
    }

    /* test 0x55 */
    *current_testing_RAM_address_copy = MICRO_TEST_RAM_TEST_55;
    if( *current_testing_RAM_address_copy != MICRO_TEST_RAM_TEST_55)
    {
        return STATUS_FAIL;
    }

    /* test 0xAA */
    *current_testing_RAM_address_copy = MICRO_TEST_RAM_TEST_AA;
    if( *current_testing_RAM_address_copy != MICRO_TEST_RAM_TEST_AA)
    {
        return STATUS_FAIL;
    }

    /* test walking 1s */
    for(loop_counter = 0; loop_counter < MICRO_TEST_U32_BITS; loop_counter++)
    {
        *current_testing_RAM_address_copy = (MICRO_TEST_RAM_TEST_WALK_1 << loop_counter);
        if( *current_testing_RAM_address_copy != (MICRO_TEST_RAM_TEST_WALK_1 << loop_counter))
        {
            return STATUS_FAIL;
        }
    }

    /* test walking 0s */
    for(loop_counter = 0; loop_counter < MICRO_TEST_U32_BITS; loop_counter++)
    {
        *current_testing_RAM_address_copy = ~(MICRO_TEST_RAM_TEST_WALK_1 << loop_counter);
        if( *current_testing_RAM_address_copy != (~(MICRO_TEST_RAM_TEST_WALK_1 << loop_counter)) )
        {
            return STATUS_FAIL;
        }
    }

    /* restore value back to ram byte under test */
    if( (temp_stored_data) == ~(temp_stored_data_inverse) )
    {
        *current_testing_RAM_address_copy = temp_stored_data;
    }
    else
    {
        return STATUS_FAIL;
    }

    /* move to the next set of ram bytes under test */
    current_testing_RAM_address++;

    /* check if complete the whole ram image */
    if(current_testing_RAM_address > (u32*)MICRO_TEST_RAM_END_ADDRESS)
    {
        current_testing_RAM_address = (u32*)MICRO_TEST_RAM_START_ADDRESS;
    }

    /* re-enable interrupt */
    __enable_irq();

    return STATUS_OK;
}
