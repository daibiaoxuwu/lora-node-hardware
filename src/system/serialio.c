/*!
 * \file      serialio.c
 *
 * \brief     Serial port log input/output
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
#include "serialio.h"
#include "serialio-board.h"

void SerialioInit(void)
{
    SerialioMcuInit();
}

// redirect stdout
int _write (int fd, char *pBuffer, int size)
{
    for (int i = 0; i < size; i++)
        SerialioMcuPutChar(pBuffer[i]);
    return size;
}

// redirect stdin
int _read (int fd, char *pBuffer, int size)
{
    SerialioMcuGetChar(pBuffer);
    return 1;
}
