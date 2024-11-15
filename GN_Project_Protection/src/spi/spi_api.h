#ifndef _SPI_API_H
#define _SPI_API_H
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
 * @brief External interface for the spi component.
 *
 * @file spi_api.h
 * @ingroup spi
 *
 *//*
 *
 **************************************************************************************************/


#include "main_internal.h"


/**************************************************************************************************/
 /**
  * @brief Initialize SPI pins
  *
  * This function initializes SPI as follows.
  * - Spi full duplex master mode.
  * - Spi clock polarity low.
  * - Spi clock phase 1 edge.
  * - Spi NSS signal type software.
  * - Spi baud rate equals baud rate pre-scaler devide by 2.
  * - Spi bit order to MSB first.
  * - Spi crc calculation disabled.
  *
  * @return none
  *
  * @requirement{2009031500}
  *
  *************************************************************************************************/
void spi_init_component(void);

/**************************************************************************************************/
/**
 * @brief Transmit data via spi communication.
 *
 * Send an amount of data starting from a buffer via SPI communication.
 *
 *
 * @param[in] SPIx - Spi instance.
 * @param[in] pData - Pointer to buffer to send.
 * @param[in] size - The amount of bytes to send.
 *
 *
 * @return STATUS_OK if complete transfer,
 *         STATUS_NULL_P if SPIx or pData are null pointers,
 *         STATUS_FAIL, if timeout.
 *
 * @requirement{2009031501}
 *
 **************************************************************************************************/
status_t spi_transmit(SPI_TypeDef *SPIx, u8 *pData, u16 size);

/**************************************************************************************************/
/**
 * @brief Receive data via spi communication.
 *
 * Receive an amount of data and store it in a buffer.
 *
 *
 * @param[in] SPIx - Spi instance.
 * @param[out] pData - Pointer to buffer to store.
 * @param[in] size - The amount of bytes to receive.
 *
 * @return STATUS_OK if complete transfer,
 *         STATUS_NULL_P if SPIx or pData are null pointers,
 *         STATUS_FAIL, if timeout.
 *
 * @requirement{2009031502}
 *
 **************************************************************************************************/
status_t spi_receive(SPI_TypeDef *SPIx, u8 *pData, u16 size);

/**************************************************************************************************/
/**
 * @brief Transmit and receive data via spi communication.
 *
 * Transfer and receive data at the same time. Note that spi should be configured in 2 line
 * full duplex mode when using this function.
 *
 *
 * @param[in] SPIx - Spi instance.
 * @param[out] pData_t - Pointer to buffer to send.
 * @param[out] pData_r - Pointer to buffer to store.
 * @param[in] size - The amount of bytes to transmit and receive.
 *
 * @return STATUS_OK if complete transfer,
 *         STATUS_NULL_P if SPIx or pData_t or pData_r are null pointers,
 *         STATUS_FAIL, if timeout.
 *
 * @requirement{2009031503}
 *
 **************************************************************************************************/
status_t spi_transmit_receive(SPI_TypeDef *SPIx, u8 *pData_t, u8 *pData_r, u16 size);


#endif /*_SPI_API_H */

