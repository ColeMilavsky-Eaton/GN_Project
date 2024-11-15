#ifndef _MICRO_TEST__H
#define _MICRO_TEST__H
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
 * @brief Header file for micro_test component.
 *
 * @file micro_test_.h
 * @ingroup micro_test
 *
 *//*
 *
 **************************************************************************************************/

#include "micro_test_internal.h"

/* For continuous cpu test */
#define MICRO_TEST_4_BYTES_AA       (0xAAAAAAAA)
#define MICRO_TEST_4_BYTES_55       (0x55555555)

/* For continuous flash check */
#define MICRO_TEST_PROGRAM_MEMORY_START_ADDRESS     (0x08000000)
#define MICRO_TEST_PROGRAM_MEMORY_END_ADDRESS 		(0x0801DFFF) //JF TODO this seems to be an incorrect value please verify.
#define MICRO_TEST_CONTI_FLASH_CHECK_SIZE_PER_CALL  (0x10)

/* For continuous RAM check */
#define MICRO_TEST_RAM_START_ADDRESS              (0x20000000)
#define MICRO_TEST_RAM_END_ADDRESS                (0x20008FFF)
#define MICRO_TEST_RAM_SIZE                       (0x9000)  //36K RAM for STM32G071 //8K RAM for STM32G031K8
#define MICRO_TEST_RAM_TEST_LOW                   (0x00000000)
#define MICRO_TEST_RAM_TEST_HIGH                  (0xFFFFFFFF)
#define MICRO_TEST_RAM_TEST_55                    (0x55555555)
#define MICRO_TEST_RAM_TEST_AA                    (0xAAAAAAAA)
#define MICRO_TEST_RAM_TEST_WALK_1                (0x00000001)
#define MICRO_TEST_RAM_TEST_WALK_0                (0xFFFFFFFE)
#define MICRO_TEST_U32_BITS                       (32)

#endif /*_MICRO_TEST__H */

