#ifndef _MICRO_TEST_API_H
#define _MICRO_TEST_API_H
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
 * @brief External interface for the micro_test component.
 *
 * @file micro_test_api.h
 * @ingroup micro_test
 *
 *//*
 *
 **************************************************************************************************/

/*************************************************************************************************/
/**
 * @brief Initialization for microprocessor self test routine.
 *
 * This is the init function for the microprocessor self test routine.
 *
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void micro_test_init(void);

/*************************************************************************************************/
/**
 * @brief Routine to enter micro continuous self test
 *
 * This function should be called periodically to run continuous self test on the microprocessor
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
void micro_test_continuous_check(u32 it);

/*************************************************************************************************/
/**
 * @brief  Main purpose of this function is to check program image checksum and verify the checksum
 * stays the same.
 *
 * This function checks 0x10 bytes per call and should be called periodically. Once the whole
 * program image has been calculated, this functions checks if the calculated checksum(calculated
 * at run time) is the same as the checksum(calculated at build time) that is being stored in the
 * flash.
 *
 * @return STATUS_OK if crc check ok.
 *         STATUS_FAIL if crc check error.
 *
 * @exception none
 *
 *************************************************************************************************/
status_t micro_test_continuous_flash_check(void);

/*************************************************************************************************/
/**
 * @brief Check for RAM memory error
 *
 * The main purpose of this function is to test if any bit in the RAM region is stuck held high or
 * low. Using 4 CPU registers as local variables through out this function, otherwise the local
 * variables will be stored in RAM and the test will result in errors.
 *
 * This function test 4 bytes at a time and should be called periodically. Each time the function
 * is executed, it mainly runs 6 tests for the 4 bytes of RAM memory under test.
 *  - test all clear
 *  - test all high
 *  - test 0x55555555
 *  - test 0xAAAAAAAA
 *  - test walking 1s (circulating shift of 0x00000001)
 *  - test walking 0s (circulating shift of 0xFFFFFFFE)
 *
 *
 *
 * @return none
 *
 * @exception none
 *
 *************************************************************************************************/
status_t micro_test_continuous_RAM_check(void);

#endif /*_MICRO_TEST_API_H */

