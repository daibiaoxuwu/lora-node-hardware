/*!
 * \file      main.c
 *
 * \brief     Ping-Pong implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <string.h>
#include "board.h"
#include "gpio.h"
#include "delay.h"
#include "timer.h"
#include "radio.h"
#include "serialio.h"

#define REGION_CN470      1
#define USE_MODEM_LORA    1

#if defined( REGION_AS923 )

#define RF_FREQUENCY                                923000000 // Hz

#elif defined( REGION_AU915 )

#define RF_FREQUENCY                                915000000 // Hz

#elif defined( REGION_CN470 )

#define RF_FREQUENCY                                478500000 // Hz

#elif defined( REGION_CN779 )

#define RF_FREQUENCY                                779000000 // Hz

#elif defined( REGION_EU433 )

#define RF_FREQUENCY                                433000000 // Hz

#elif defined( REGION_EU868 )

#define RF_FREQUENCY                                868000000 // Hz

#elif defined( REGION_KR920 )

#define RF_FREQUENCY                                920000000 // Hz

#elif defined( REGION_IN865 )

#define RF_FREQUENCY                                865000000 // Hz

#elif defined( REGION_US915 )

#define RF_FREQUENCY                                915000000 // Hz

#elif defined( REGION_US915_HYBRID )

#define RF_FREQUENCY                                915000000 // Hz

#else
    #error "Please define a frequency band in the compiler options."
#endif

#define TX_OUTPUT_POWER                             1        // dBm

#if defined( USE_MODEM_LORA )

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#elif defined( USE_MODEM_FSK )

#define FSK_FDEV                                    25000     // Hz
#define FSK_DATARATE                                50000     // bps
#define FSK_BANDWIDTH                               50000     // Hz
#define FSK_AFC_BANDWIDTH                           83333     // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#else
    #error "Please define a modem in the compiler options."
#endif

typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;

#define RX_TIMEOUT_VALUE                            0
#define BUFFER_SIZE                                 64 // Define the payload size here

const uint8_t PingMsg[] = "QING";
const uint8_t PongMsg[] = "QONG";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];
uint32_t recv_cnt = 0;
uint32_t tx_cnt = 0;

uint32_t rf_frequency = 470000000;

bool rxDone = true;
bool isSleep = true;

States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * LED GPIO pins objects
 */
// extern Gpio_t Led1;
// extern Gpio_t Led2;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );

static uint32_t send_cnt = 0, rx_cnt = 0;
/**
 * Main application entry point.
 */
int main( void )
{
    bool isMaster = false;
    uint8_t i;

    // Target board initialization
    BoardInitMcu( );
    BoardInitPeriph( );

    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );

    Radio.SetChannel( rf_frequency );
    printf("start\n");

#if defined( USE_MODEM_LORA )

    bool CRC_ON = true;           // Enables/Disables the CRC [0: OFF, 1: ON]
    bool FREQ_HOP_ON = false;     // Enables disables the intra-packet frequency hopping
    uint8_t HOP_PERIOD = 0;       // Number of symbols between each hop
    uint32_t TIME_OUT = 3000;     // Transmission timeout [ms]
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   CRC_ON, FREQ_HOP_ON, HOP_PERIOD, LORA_IQ_INVERSION_ON, TIME_OUT );

    bool RX_CONTINUOUS = true;    // Sets the reception in continuous mode [false: single mode, true: continuous mode]
    uint8_t PAYLOAD_LEN = 0;      // Sets payload length when fixed length is used
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   PAYLOAD_LEN, CRC_ON, FREQ_HOP_ON, HOP_PERIOD, LORA_IQ_INVERSION_ON, RX_CONTINUOUS );

#elif defined( USE_MODEM_FSK )

    Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                                  FSK_DATARATE, 0,
                                  FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, 0, 3000 );

    Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                  0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                  0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                                  0, 0,false, true );

#else
    #error "Please define a frequency band in the compiler options."
#endif

    if(isMaster){
        State = TX;
    }else{
        State = RX;
        // Radio.Rx( RX_TIMEOUT_VALUE );
    }

    // Detect start signal
    Radio.SetChannel( 507500000 );
    Radio.Rx( RX_TIMEOUT_VALUE );
    
    while( 1 )
    {
        if(isSleep){
            continue;
        }

        if(!isMaster){
            if(!rxDone){
                continue;
            }
        }
        switch( State )
        {
        case TX:
            if( BufferSize > 0 )
            {
                // Send the next PING frame
                Buffer[0] = 'P';
                Buffer[1] = 'I';
                Buffer[2] = 'N';
                Buffer[3] = 'G';
                // We fill the buffer with numbers for the payload
                for( i = 4; i < BufferSize; i++ )
                {
                    Buffer[i] = i - 4;
                }
                Buffer[4] = tx_cnt;
                
                printf("master tx cnt: %d\n", tx_cnt);
                tx_cnt++;
                
                DelayMs( 1 );
                Radio.Send( Buffer, BufferSize );
                DelayMs(5000);
                
                if(tx_cnt > 2){
                    tx_cnt = 0;
                    isSleep = true;
                    Radio.SetChannel( 507500000 );
                    Radio.Rx( RX_TIMEOUT_VALUE );
                    printf("Go to sleep...\n");
                }
            }
            break;

        case RX:
        case RX_TIMEOUT:
        case RX_ERROR:
            Radio.Rx( RX_TIMEOUT_VALUE );
            rxDone = false;
            break;

        case TX_TIMEOUT:
            if(isMaster){
                State = TX;
            }
        default:
            break;
        }

        // TimerLowPowerHandler( );
    }
}

void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    if(isSleep){
        isSleep = false;
        Radio.Sleep();
        printf("WAKE UP!\n");

        uint32_t step = payload[4];
        rf_frequency = step*125000 + 470000000;
        printf("CONF:step=%d,rf_freq=%d\n",step,rf_frequency);

        Radio.SetChannel( rf_frequency );
        DelayMs(1);
        return;
    }

    uint32_t pkt_id = payload[4];
    printf("R%d: p%d,RSSI=%d,SNR=%d\n",recv_cnt,pkt_id,rssi,snr);
    recv_cnt++;

    if(pkt_id > 1 || recv_cnt > 15){
        recv_cnt = 0;
        isSleep = true;
        Radio.SetChannel( 507500000 );
        Radio.Rx( RX_TIMEOUT_VALUE );
        printf("Go to sleep...\n");
    }

    Radio.Sleep( );
    // BufferSize = size;
    // memcpy( Buffer, payload, BufferSize );
    // RssiValue = rssi;
    // SnrValue = snr;
    State = RX;
    rxDone = true;
    DelayMs(1);
}

void OnTxTimeout( void )
{
    printf("tx timeout\n");
    Radio.Sleep( );
    State = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    printf("rx timeout\n");
    Radio.Sleep( );
    State = RX_TIMEOUT;
}

void OnRxError( void )
{
    printf("rx error\n");
    Radio.Sleep( );
    State = RX_ERROR;
}