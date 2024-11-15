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
 * @defgroup spi SPI component
 *
 * @brief The SPI component handles the initialization of SPI and sending/receiving data.
 *
 * # Overview
 * The SPI component performs the following functions:
 *  - Initializing SPI pins.
 *  - Send an amount of data from a buffer.
 *  - Receive an amount of data and store it in a buffer.
 *
 * @file spi.c
 * @ingroup spi
 *
 *
 *//*
 *
 **************************************************************************************************/
#include "spi_.h"
#include "main_internal.h"

volatile s16 spi_timeout_timer;


status_t spi_transmit(SPI_TypeDef *SPIx, u8 *pData, u16 size)
{
    status_t status = STATUS_OK;
    volatile u8 spi_delay;

    /* Check for null pointer */
    if( (SPIx == NULL) || (pData == NULL) )
    {
        status =  STATUS_NULL_P;
    }

    if(status == STATUS_OK)
    {
        /* Check if SPI is enabled */
        if(!LL_SPI_IsEnabled(SPIx))
        {
            LL_SPI_Enable(SPIx);
        }

        /* pull chip select low */
        LL_GPIO_ResetOutputPin(SPI_CS_GPIO_Port, SPI_CS_Pin);

        /* While size is still greater than 0 and status ok */
        while( (size > 0) && (status == STATUS_OK) )
        {
            /* Send data */
            LL_SPI_TransmitData8(SPIx, *pData);

            pData++;
            size--;

            /* set timeout */
            SPI_SET_TIMEOUT(SPI_TIMEOUT_COUNTS);

            /* Poll TXE flag goes true. */
            while( (!LL_SPI_IsActiveFlag_TXE(SPIx)) && (status == STATUS_OK) )
            {
                if(SPI_TIMEOUT_EXPIRED)
                {
                    status = STATUS_FAIL;
                }
            }
        }

        if(status == STATUS_OK)
        {
            /* set timeout */
            SPI_SET_TIMEOUT(SPI_TIMEOUT_COUNTS);

            /* Wait while busy flag is true or TxFIFO not empty*/
            while( ((LL_SPI_IsActiveFlag_BSY(SPIx)) || (LL_SPI_GetTxFIFOLevel(SPIx) != LL_SPI_TX_FIFO_EMPTY))
                    && (status == STATUS_OK)
                 )
            {
                if(SPI_TIMEOUT_EXPIRED)
                {
                    status = STATUS_FAIL;
                }
            }
        }

        if(status == STATUS_OK)
        {
            /* set timeout */
            SPI_SET_TIMEOUT(SPI_TIMEOUT_COUNTS);

            /* Since transfer and receive happens at the same time. This clears the RXNE flag */
            while( (LL_SPI_IsActiveFlag_RXNE(SPIx)) && (status == STATUS_OK) )
            {
                (void)LL_SPI_ReceiveData8(SPIx);

                if(SPI_TIMEOUT_EXPIRED)
                {
                    status = STATUS_FAIL;
                }
            }
        }

        /* Disable SPI */
        LL_SPI_Disable(SPIx);

    }

    /* pull chip select back high  */
    LL_GPIO_SetOutputPin(SPI_CS_GPIO_Port, SPI_CS_Pin);

    /* Return */
    return status;
}

status_t spi_receive(SPI_TypeDef *SPIx, u8 *pData, u16 size)
{
    status_t status = STATUS_OK;
    volatile u8 spi_delay;
    u8 dummy_byte = 0xBB; // this can be any value.

    /* Check for null pointer */
    if( (SPIx == NULL) || (pData == NULL) )
    {
        return STATUS_NULL_P;
    }

    if(status == STATUS_OK)
    {
        /* Check if SPI is enabled */
        if(!LL_SPI_IsEnabled(SPIx))
        {
            LL_SPI_Enable(SPIx);
        }

        /* pull chip select low */
        LL_GPIO_ResetOutputPin(SPI_CS_GPIO_Port, SPI_CS_Pin);

        /* While size is still greater than 0 and status ok. */
        while( (size > 0) && (status == STATUS_OK) )
        {
            /* set timeout */
            SPI_SET_TIMEOUT(SPI_TIMEOUT_COUNTS);

            /* send dummy byte to trigger/start the clock. */
            LL_SPI_TransmitData8(SPIx, dummy_byte);

            /* wait for RXNE flag to set. */
            while( (!LL_SPI_IsActiveFlag_RXNE(SPIx)) && (status == STATUS_OK) )
            {
                if(SPI_TIMEOUT_EXPIRED)
                {
                    status = STATUS_FAIL;
                }
            }

            if( status == STATUS_OK )
            {
                *pData = LL_SPI_ReceiveData8(SPIx);

                pData++;
                size--;
            }
        }

        if(status == STATUS_OK)
        {
            /* set timeout */
            SPI_SET_TIMEOUT(SPI_TIMEOUT_COUNTS);

            /* Wait while busy flag is true or RxFIFO not empty, this part of the code should not
             * enter, since all of the bytes are read. If code enters here, then size between
             * transfer and receive might not match. */
            while( ((LL_SPI_IsActiveFlag_BSY(SPIx)) || (LL_SPI_GetRxFIFOLevel(SPIx) != LL_SPI_RX_FIFO_EMPTY))
                    && (status == STATUS_OK)
                 )
            {
                if(SPI_TIMEOUT_EXPIRED)
                {
                    status = STATUS_FAIL;
                }
            }
        }

        /* Disable SPI */
        LL_SPI_Disable(SPIx);
    }

    /* pull chip select back high  */
    LL_GPIO_SetOutputPin(SPI_CS_GPIO_Port, SPI_CS_Pin);

    /* Return */
    return status;
}

status_t spi_transmit_receive(SPI_TypeDef *SPIx, u8 *pData_t, u8 *pData_r, u16 size)
{
    status_t status = STATUS_OK;
    volatile u8 spi_delay;

    /* Check for null pointer */
    if( (SPIx == NULL) || (pData_t == NULL) || (pData_r == NULL) )
    {
        status =  STATUS_NULL_P;
    }

    if(status == STATUS_OK)
    {
        /* Check if SPI is enabled */
        if(!LL_SPI_IsEnabled(SPIx))
        {
            LL_SPI_Enable(SPIx);
        }

        /* pull chip select low */
        LL_GPIO_ResetOutputPin(SPI_CS_GPIO_Port, SPI_CS_Pin);

        /* While size is still greater than 0 and status ok */
        while( (size > 0)  && (status == STATUS_OK) )
        {
            /* Send data */
            LL_SPI_TransmitData8(SPIx, *pData_t);

            /* Add a slight delay, this is needed when transferring at high speed
             * in case the slave device is not fast enough.  */
            for(spi_delay = 0; spi_delay < 10; spi_delay++);

            /* set timeout */
            SPI_SET_TIMEOUT(SPI_TIMEOUT_COUNTS);

            /* Poll TXE flag goes true. */
            while( (!LL_SPI_IsActiveFlag_TXE(SPIx)) && (status == STATUS_OK) )
            {
                if(SPI_TIMEOUT_EXPIRED)
                {
                    status = STATUS_FAIL;
                }
            }

            /* set timeout */
            SPI_SET_TIMEOUT(SPI_TIMEOUT_COUNTS);

            /* wait for RXNE flag to set. */
            while( (!LL_SPI_IsActiveFlag_RXNE(SPIx)) && (status == STATUS_OK) )
            {
                if(SPI_TIMEOUT_EXPIRED)
                {
                    status = STATUS_FAIL;
                }
            }

            /* read incoming data */
            if( LL_SPI_IsActiveFlag_RXNE(SPIx) && (status == STATUS_OK) )
            {
                *pData_r = LL_SPI_ReceiveData8(SPIx);
            }

            pData_t++;
            pData_r++;
            size--;
        }

        if(status == STATUS_OK)
        {
            /* set timeout */
            SPI_SET_TIMEOUT(SPI_TIMEOUT_COUNTS);

            /* Wait while busy flag is true or TxFIFO not empty*/
            while( ((LL_SPI_IsActiveFlag_BSY(SPIx)) ||
                    (LL_SPI_GetTxFIFOLevel(SPIx) != LL_SPI_TX_FIFO_EMPTY) ||
                    (LL_SPI_GetRxFIFOLevel(SPIx) != LL_SPI_RX_FIFO_EMPTY)
                   ) &&
                   (status == STATUS_OK)
                 )
            {
                if(SPI_TIMEOUT_EXPIRED)
                {
                    status = STATUS_FAIL;
                }
            }
        }

        /* Disable SPI */
        LL_SPI_Disable(SPIx);

    }

    /* pull chip select back high  */
    LL_GPIO_SetOutputPin(SPI_CS_GPIO_Port, SPI_CS_Pin);

    /* Return */
    return status;
}
