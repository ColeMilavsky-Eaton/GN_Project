/*************************************************************************************************/
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
 * @defgroup micro_test
 *
 * @brief This file check for cpu registers at runtime.

 * This file is a modified version from ST "Class B 60730-1 and 60335-1 Functional Safety Package
 * with software expansion for STM32Cube" version 2.3.0
 *
 *
 *
 * @file continuous_cpu_test.s
 * @ingroup micro_test
 *
 *
 *//*
 *
 *************************************************************************************************/

  SECTION .text:CODE(2)

  EXTERN u32_aa
  EXTERN u32_55


/**************************************************************************************************/
/**
 * @brief Test CPU registers.
 *
 * This function/file is a modified version from ST "Class B 60730-1 and 60335-1 Functional Safety
 * Package with software expansion for STM32Cube" version 2.3.0.
 *
 *Cortex-M0 CPU test during run-time
 *       Note: when possible, BRANCH are 16-bit only (depending on relative offset to final BL
 *             instruction)
 *
 ********************* (C) COPYRIGHT 2019 STMicroelectronics ********************
 * File Name          : stm32g0xx_STLcpurunIAR.s
 * Author             : MCD Application Team
 * Date First Issued  : 28-Jun-2019
 * Version            : V2.3.0
 * Description        : This file contains the Cortex-M3 CPU tests to be done
 *                      during run-time.
 *****************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 *
 * @return CPUTEST_SUCCESS (=1), if test is ok
 *         CPUTEST_FAIL    (=0), if test fail
 *
 * @exception none
 *
 **************************************************************************************************/
STL_RunTimeCPUTest:
    EXPORT STL_RunTimeCPUTest

    PUSH {R4-R7}              ; Safe registers

    ; Register R1
    LDR R0, = u32_aa
    LDR R1,[R0]
    LDR R0,[R0]
    CMP R0,R1
    BNE CPUTestFail
    LDR R0, =u32_55
    LDR R1,[R0]
    LDR R0,[R0]
    CMP R0,R1
    BNE CPUTestFail
    MOVS R1, #0x1             ; For ramp test

    ; Register R2
    LDR R0, =u32_aa
    LDR R2,[R0]
    LDR R0,[R0]
    CMP R0,R2
    BNE CPUTestFail
    LDR R0, =u32_55
    LDR R2,[R0]
    LDR R0,[R0]
    CMP R0,R2
    BNE CPUTestFail
    MOVS R2, #0x2             ; For ramp test

    ; Register R3
    LDR R0, =u32_aa
    LDR R3,[R0]
    LDR R0,[R0]
    CMP R0,R3
    BNE CPUTestFail
    LDR R0, =u32_55
    LDR R3,[R0]
    LDR R0,[R0]
    CMP R0,R3
    BNE CPUTestFail
    MOVS R3, #0x3             ; For ramp test

    ; Register R4
    LDR R0, =u32_aa
    LDR R4,[R0]
    LDR R0,[R0]
    CMP R0,R4
    BNE CPUTestFail
    LDR R0, =u32_55
    LDR R4,[R0]
    LDR R0,[R0]
    CMP R0,R4
    BNE CPUTestFail
    MOVS R4, #0x4             ; For ramp test

    ; Register R5
    LDR R0, =u32_aa
    LDR R5,[R0]
    LDR R0,[R0]
    CMP R0,R5
    BNE CPUTestFail
    LDR R0, =u32_55
    LDR R5,[R0]
    LDR R0,[R0]
    CMP R0,R5
    BNE CPUTestFail
    MOVS R5, #0x5             ; For ramp test

    ; Register R6
    LDR R0, =u32_aa
    LDR R6,[R0]
    LDR R0,[R0]
    CMP R0,R6
    BNE CPUTestFail
    LDR R0, =u32_55
    LDR R6,[R0]
    LDR R0,[R0]
    CMP R0,R6
    BNE CPUTestFail
    MOVS R6, #0x6             ; For ramp test

    ; Register R7
    LDR R0, =u32_aa
    LDR R7,[R0]
    LDR R0,[R0]
    CMP R0,R7
    BNE CPUTestFail
    LDR R0, =u32_55
    LDR R7,[R0]
    LDR R0,[R0]
    CMP R0,R7
    BNE CPUTestFail
    MOVS R7, #0x7             ; For ramp test

    ; Register R8
    LDR R0, =u32_aa
    LDR R0,[R0]
    MOV R8,R0
    CMP R0,R8
    BNE CPUTestFail
    LDR R0, =u32_55
    LDR R0,[R0]
    MOV R8,R0
    CMP R0,R8
    BNE CPUTestFail
    MOVS R0, #0x08            ; For ramp test
    MOV R8,R0

    BAL CPUTstCont

CPUTestFail
    POP {R4-R7}              ; Restore registers
    MOVS R0, #0x0       ; CPUTEST_FAIL
    BX LR               ; return to the caller

CPUTstCont
    ; Register R9
    LDR R0, =u32_aa
    LDR R0,[R0]
    MOV R9,R0
    CMP R0,R9
    BNE CPUTestFail
    LDR R0, =u32_55
    LDR R0,[R0]
    MOV R9,R0
    CMP R0,R9
    BNE CPUTestFail
    MOVS R0, #0x09            ; For ramp test
    MOV R9,R0

    ; Register R10
    LDR R0, =u32_aa
    LDR R0,[R0]
    MOV R10,R0
    CMP R0,R10
    BNE CPUTestFail
    LDR R0, =u32_55
    LDR R0,[R0]
    MOV R10,R0
    CMP R0,R10
    BNE CPUTestFail
    MOVS R0, #0x0A            ; For ramp test
    MOV R10,R0

    ; Register R11
    LDR R0, =u32_aa
    LDR R0,[R0]
    MOV R11,R0
    CMP R0,R11
    BNE CPUTestFail
    LDR R0, =u32_55
    LDR R0,[R0]
    MOV R11,R0
    CMP R0,R11
    BNE CPUTestFail
    MOVS R0, #0x0B            ; For ramp test
    MOV R11,R0

    ; Register R12
    LDR R0, =u32_aa
    LDR R0,[R0]
    MOV R12,R0
    CMP R0,R12
    BNE CPUTestFail
    LDR R0, =u32_55
    LDR R0,[R0]
    MOV R12,R0
    CMP R0,R12
    BNE CPUTestFail
    MOVS R0, #0x0C            ; For ramp test
    MOV R12,R0
    LDR R0, =CPUTstCont

    ; Ramp pattern verification (R0 is not tested)
    CMP R1, #0x01
    BNE CPUTestFail
    CMP R2, #0x02
    BNE CPUTestFail
    CMP R3, #0x03
    BNE CPUTestFail
    CMP R4, #0x04
    BNE CPUTestFail
    CMP R5, #0x05
    BNE CPUTestFail
    CMP R6, #0x06
    BNE CPUTestFail
    CMP R7, #0x07
    BNE CPUTestFail
    MOVS R0, #0x08
    CMP R0,R8
    BNE CPUTestFail
    MOVS R0, #0x09
    CMP R0,R9
    BNE CPUTestFail
    MOVS R0, #0x0A
    CMP R0,R10
    BNE CPUTestFail
    MOVS R0, #0x0B
    CMP R0,R11
    BNE CPUTestFail
    MOVS R0, #0x0C
    CMP R0,R12
    BNE CPUTestFail


    POP {R4-R7}              ; Restore registers

    MOVS R0, #0x1       ; CPUTEST_SUCCESS
    BX LR               ; return to the caller

  END
