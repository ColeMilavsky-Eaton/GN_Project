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
 * @defgroup gpio GPIO component
 *
 * @brief The GPIO component handles the initialization of GPIO and setting/reseting/reading bits.
 *
 * # Overview
 * The GPIO component performs the following functions:
 *  - Initializing GPIO pins.
 *  - Set output pin
 *  - Reset output pin
 *  - Toggle output pin
 *  - Read input pin
 *
 *
 * @file gpio.c
 * @ingroup gpio
 *
 *
 *//*
 *
 **************************************************************************************************/
#include "main_internal.h"
#include "types.h"
#include "gpio_.h"

inline void gpio_set_output_pin(GPIO_TypeDef *GPIOx, u32 pin_mask)
{
    /* Set output */
    LL_GPIO_SetOutputPin(GPIOx, pin_mask);

    return;
}

inline void gpio_reset_output_pin(GPIO_TypeDef *GPIOx, u32 pin_mask)
{
    /* Reset output */
    LL_GPIO_ResetOutputPin(GPIOx, pin_mask);

    return;
}

gpio_pin_state_t gpio_read_input_pin(GPIO_TypeDef *GPIOx, u32 pin_mask)
{
    /* pass result from low layer function back to caller */
    return ((gpio_pin_state_t) LL_GPIO_IsInputPinSet(GPIOx, pin_mask));
}

void gpio_toggle_output_pin(GPIO_TypeDef *GPIOx, u32 pin_mask)
{
    /* toggle pin*/
    LL_GPIO_TogglePin(GPIOx, pin_mask);

    return;
}

