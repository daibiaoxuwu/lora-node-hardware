/*!
 * \file      SHT2x.h
 *
 * \brief     Temperature and Humidity Sensor API
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
#ifndef __SHT2x_H__
#define __SHT2x_H__

#include <stdbool.h>
#include <stdint.h>

/*!
 * SHT2x I2C address
 */
#define SHT2x_I2C_ADDRESS 0x80

/*!
 * \brief Initializes the device
 *
 * \retval status [SUCCESS, FAIL]
 */
uint8_t SHT2xInit(void);

/*!
 * \brief Resets the device
 *
 * \retval status [SUCCESS, FAIL]
 */
uint8_t SHT2xReset(void);

/*!
 * \brief Writes a byte at specified address in the device
 *
 * \param [IN]: addr
 * \param [IN]: data
 * \retval status [SUCCESS, FAIL]
 */
uint8_t SHT2xWrite(uint8_t addr, uint8_t data);

/*!
 * \brief Reads a byte at specified address in the device
 *
 * \param [IN]: addr
 * \param [OUT]: data
 * \retval status [SUCCESS, FAIL]
 */
uint8_t SHT2xRead(uint8_t addr, uint8_t *data);

/*!
 * \brief Reads a buffer at specified address in the device
 *
 * \param [IN]: addr
 * \param [OUT]: data
 * \param [IN]: size
 * \retval status [SUCCESS, FAIL]
 */
uint8_t SHT2xReadBuffer(uint8_t addr, uint8_t *data, uint8_t size);

/*!
 * \brief Reads Temperature and Humidity
 *
 * \param [OUT]: temp
 * \param [OUT]: humi
 */
void SHT2xGetTempHumi(uint16_t *temp, uint16_t *humi);

/*!
 * \brief Sets the I2C device slave address
 *
 * \param [IN]: addr
 */
void SHT2xSetDeviceAddr(uint8_t addr);

/*!
 * \brief Gets the I2C device slave address
 *
 * \retval: addr Current device slave address
 */
uint8_t SHT2xGetDeviceAddr(void);

#endif // __SHT2x_H__
