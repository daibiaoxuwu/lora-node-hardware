/*!
 * \file      serialio-board.c
 *
 * \brief     Target board serial io driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *               _______ _____ _____ _   _  _____ _    _ _    _
 *              |__   __/ ____|_   _| \ | |/ ____| |  | | |  | |  /\
 *                 | | | (___   | | |  \| | |  __| |__| | |  | | /  \
 *                 | |  \___ \  | | | . ` | | |_ |  __  | |  | |/ /\ \
 *                 | |  ____) |_| |_| |\  | |__| | |  | | |__| / ____ \
 *                 |_| |_____/|_____|_| \_|\_____|_|  |_|\____/_/    \_\
 *              (C)2017-2018 Tsinghua
 *
 * \endcode
 *
 * \author    jkadbear( Tsinghua )
 */
#include <stdio.h>
#include "board-config.h"
#include "uart.h"
#include "serialio-board.h"

extern Uart_t Uart2;

uint8_t Serialio_TxBuffer[SERIALIO_FIFO_TX_SIZE];
uint8_t Serialio_RxBuffer[SERIALIO_FIFO_RX_SIZE];

void SerialioMcuInit(void)
{
    UartInit(&Uart2, UART_2, SERIALIO_TX, SERIALIO_RX);
    UartConfig(&Uart2, RX_TX, USART_BAUDRATE, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL);

    FifoInit(&Uart2.FifoTx, Serialio_TxBuffer, SERIALIO_FIFO_TX_SIZE);
    FifoInit(&Uart2.FifoRx, Serialio_RxBuffer, SERIALIO_FIFO_RX_SIZE);
}

// redirect stdout
uint8_t SerialioMcuPutChar( char data )
{
    UartPutChar(&Uart2, (uint8_t) data);
    return 1;
}

// redirect stdin
uint8_t SerialioMcuGetChar( char *pdata )
{
    while (UartGetChar(&Uart2, (uint8_t *) pdata));
    return 1;
}
