/*!
 * \file      gps-board.c
 *
 * \brief     Target board GPS driver implementation
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
#include "board-config.h"
#include "gpio.h"
#include "gps.h"
#include "rtc-board.h"
#include "gps-board.h"


/*!
 * FIFO buffers size
 */
#define FIFO_RX_SIZE                                128

uint8_t RxBuffer[FIFO_RX_SIZE];

/*!
 * \brief Buffer holding the  raw data received from the gps
 */
uint8_t NmeaString[128];

/*!
 * \brief Maximum number of data byte that we will accept from the GPS
 */
uint8_t NmeaStringSize = 0;

Gpio_t GpsPowerEn;
Gpio_t GpsPps;
extern Uart_t Uart1;

PpsTrigger_t PpsTrigger;

void GpsMcuOnPpsSignal( void )
{
    bool parseData = false;

    GpsPpsHandler( &parseData );

    if( parseData == true )
    {
        UartInit( &Uart1, UART_1, UART_TX, UART_RX );
        UartConfig( &Uart1, RX_ONLY, 9600, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );
    }
}

void GpsMcuInvertPpsTrigger( void )
{
    if( PpsTrigger == PpsTriggerIsRising )
    {
        PpsTrigger = PpsTriggerIsFalling;
        GpioSetInterrupt( &GpsPps, IRQ_FALLING_EDGE, IRQ_VERY_LOW_PRIORITY, &GpsMcuOnPpsSignal );
    }
    else
    {
        PpsTrigger = PpsTriggerIsRising;
        GpioSetInterrupt( &GpsPps, IRQ_RISING_EDGE, IRQ_VERY_LOW_PRIORITY, &GpsMcuOnPpsSignal );
    }
}

void GpsMcuInit( void )
{
    NmeaStringSize = 0;
    PpsTrigger = PpsTriggerIsFalling;

    GpioInit( &GpsPowerEn, GPS_POWER_ON, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

    GpioInit( &GpsPps, GPS_PPS, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioSetInterrupt( &GpsPps, IRQ_FALLING_EDGE, IRQ_VERY_LOW_PRIORITY, &GpsMcuOnPpsSignal );

    FifoInit( &Uart1.FifoRx, RxBuffer, FIFO_RX_SIZE );
    Uart1.IrqNotify = GpsMcuIrqNotify;

    GpsMcuStart( );
}

void GpsMcuStart( void )
{
    GpioWrite( &GpsPowerEn, 0 );    // power up the GPS
}

void GpsMcuStop( void )
{
    GpioWrite( &GpsPowerEn, 1 );    // power down the GPS
}

void GpsMcuProcess( void )
{

}

void GpsMcuIrqNotify( UartNotifyId_t id )
{
    uint8_t data;
    if( id == UART_NOTIFY_RX )
    {
        if( UartGetChar( &Uart1, &data ) == 0 )
        {
            if( ( data == '$' ) || ( NmeaStringSize >= 127 ) )
            {
                NmeaStringSize = 0;
            }

            NmeaString[NmeaStringSize++] = ( int8_t )data;

            if( data == '\n' )
            {
                NmeaString[NmeaStringSize++] = '\0';
                GpsParseGpsData( ( int8_t* )NmeaString, NmeaStringSize );
                UartDeInit( &Uart1 );
                BlockLowPowerDuringTask ( false );
            }
        }
    }
}
