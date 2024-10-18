/*!
 * \file      serialio.h
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
#ifndef __SERIALIO_H__
#define __SERIALIO_H__

#include <stdio.h>
#include <stdint.h>
/*!
 * FIFO buffers size
 */
#define SERIALIO_FIFO_TX_SIZE                   128
#define SERIALIO_FIFO_RX_SIZE                   128

#define USART_BAUDRATE                          115200

/*!
 * \brief Initializes the serial port to support log
 */
void SerialioInit(void);

/*!
 * Log level
 */
#define  LOG_ERR    0
#define	 LOG_WARN   1
#define  LOG_INFO   2
#define	 LOG_DEBUG  3

/* Log level setting */
#ifndef LOGLEVEL
#define LOGLEVEL LOG_ERR
#endif

#if (defined(SERIALIO) && (LOGLEVEL >= LOG_ERR))
#define log_err(format, ...) printf("ERR:");printf(format, ##__VA_ARGS__)
#else
#define log_err(format, ...)
#endif

#if (defined(SERIALIO) && (LOGLEVEL >= LOG_WARN))
#define log_warn(format, ...) printf("WARN:");printf(format, ##__VA_ARGS__)
#else
#define log_warn(format, ...)
#endif

#if (defined(SERIALIO) && (LOGLEVEL >= LOG_INFO))
#define log_info(format, ...) printf("INFO:");printf(format, ##__VA_ARGS__)
#else
#define log_info(format, ...)
#endif

#if (defined(SERIALIO) && (LOGLEVEL >= LOG_DEBUG))
#define log_debug(format, ...) printf("DBG:");printf(format, ##__VA_ARGS__)
#else
#define log_debug(format, ...)
#endif

#endif // __SERIALIO_H__
