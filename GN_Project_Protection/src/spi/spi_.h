#ifndef _SPI__H
#define _SPI__H
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
 * @brief Internal (to the component) interface for the spi component.
 *
 * @file spi.h
 * @ingroup spi
 *
 *//*
 *
 **************************************************************************************************/
#include "spi_internal.h"

/* timeouts */
#define SPI_TIMEOUT_COUNTS      (100)
#define SPI_SET_TIMEOUT(_X)     {spi_timeout_timer = (_X) ;}
#define SPI_TIMEOUT_EXPIRED     ((--spi_timeout_timer) <= 0)

#endif /*_SPI__H */
