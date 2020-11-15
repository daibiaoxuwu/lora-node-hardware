/*!
 * \file      serialio-board.h
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
#ifndef __SERIALIO_BOARD_H__
#define __SERIALIO_BOARD_H__

#include "serialio.h"

void SerialioMcuInit(void);

// redirect stdout
uint8_t SerialioMcuPutChar( char data );

// redirect stdin
uint8_t SerialioMcuGetChar( char *pdata );

#endif // __SERIALIO_BOARD_H__
