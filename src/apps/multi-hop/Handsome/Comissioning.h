/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: End device commissioning parameters

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __LORA_COMMISSIONING_H__
#define __LORA_COMMISSIONING_H__

#include "timer.h"
#include "radio.h"

/**************************************************************/
/*              Mesh LoRa                                    */
/************************************************************/

/*
* define Rx/Tx config
*/
#define RF_FREQUENCY                                478500000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm
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
	
#define BUFFER_SIZE                                 64         // Define the payload size here

/*
* define Timer
*/
#define ROUTER_MIN_INTERVAL                         8000      //ms
#define ROUTER_MAX_INTERVAL                         300000
#define DATAINTERVAL                                5000
#define RTSINTERVAL                                 300     
#define WAITFORDATATIME                             300    
#define BACKOFFTIME                                 150

#define AWAKETIME                                   3000
#define SLEEPTIME                                   1000

static uint32_t router_interval = ROUTER_MIN_INTERVAL;
static TimerEvent_t SendRouterTimer, SendDataTimer, BackOffTimer, DutyCycleTimer; 
static bool beginRouterTimer = false;

/*
* define addr
*/
#define DEVICE_ADDRESS                               ( uint16_t )0x0000
#define GATEWAY_ADDRESS						         ( uint16_t )0x0000

/*
* define fixed relay
*/
#define MESHLORA_FIX_RELAY                           true
#define FIXED_RELAY_ADDRESS						     ( uint16_t )0x0000

//acount pkts
#define TOTAL_NODES                                      3
static int32_t getDataNum[TOTAL_NODES];
#define RELAY_NODES                                      3
static int32_t relayDataNum[RELAY_NODES];
static int32_t relaySendDataNum[RELAY_NODES];

/*
* define pts size
*/
#define RTS0_ACKS_SIZE                                4
#define RTS1_SIZE                                     6

/*
* define length of AppData 
*/
#define MESHLORA_APPDATA_PAYLOAD_LENGTH               8

/*
* define CAD Backoff
*/
#define CAD_BACKOFF_TIME                              200    //ms  
static bool meshLoRaChannelActivityDetected = false;

/*
* states
*/
typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
	  CAD
}States_t;

/*
* duty-cycle states
*/
typedef enum
{
	AWAKE,
	MID_SLEEP,
	SLEEP
}DcStates_t;

/*
* packet type
*/
typedef enum
{
    NOTHING,
    RTS,
    ACK,
    ROUTER,
    DATA,
    RELAY
}Packets_t;

typedef union uMeshLoRaMacHeader
{
    /*!
     * Byte-access to the bits
     */
    uint8_t Value;
    /*!
     * Structure containing single access to header bits
     */
    struct sMeshLoRaHdrBits
    {
        /*!
         * Major version
         */
        uint8_t Major           : 2;
        /*!
         * RFU
         */
        uint8_t RFU             : 3;
        /*!
         * Message type
         */
        uint8_t MType           : 3;
    }Bits;
}MeshLoRaMacHeader_t;

typedef struct uMeshLoRaFrameHeader
{
	MeshLoRaMacHeader_t Mhdr;
	uint16_t FrameType;
	int8_t FrameCnt;
	uint8_t FramePayloadLen;
}MeshLoRaFrameHeader_t;

/*
* router table
*/
#define MESHLORA_ROUTER_TABLES_LENGTH                 100

static uint32_t meshLoRaRouterSequenceNum = 0;
static int8_t meshLoRaCurrentIndLen = 0;    //indicate nums of dev_addrs
static uint16_t meshLoRaCurrentInd[MESHLORA_ROUTER_TABLES_LENGTH];  //contain dev_addrs
static uint16_t meshLoRaNxtAddr[MESHLORA_ROUTER_TABLES_LENGTH];
static uint8_t  meshLoRaCost[MESHLORA_ROUTER_TABLES_LENGTH];
static int16_t meshLoRaRssi[MESHLORA_ROUTER_TABLES_LENGTH];

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

static uint8_t BufferSize = BUFFER_SIZE;
static uint8_t Buffer[BUFFER_SIZE];  //for rx
static uint8_t BufferSize_send = BUFFER_SIZE;
static uint8_t Buffer_send[BUFFER_SIZE];  //for tx

//Buffer to save pkts wait to send, only data pkts
#define BUFFER_SAVE_DATA                             6
#define BUFFER_SAVE_RELAY                            4
static uint8_t bf_save_data_now = 0, bf_save_data_have = 0, bf_save_relay_now = 0, bf_save_relay_have = 0;
static uint16_t lost_data = 0, lost_relay = 0;
static uint8_t Buffer_save_data_len[BUFFER_SAVE_DATA], Buffer_save_relay_len[BUFFER_SAVE_RELAY];
static uint8_t Buffer_save_data[BUFFER_SAVE_DATA][BUFFER_SIZE], Buffer_save_relay[BUFFER_SAVE_RELAY][BUFFER_SIZE];

static States_t State = LOWPOWER;
static DcStates_t dcState = AWAKE;
static Packets_t Ptype = NOTHING;

static bool willSendRouter = false;

static bool getAck = false;
static bool isConnectToGW = false;
static bool rxNotimerOut = false;

static int16_t RssiValue = 0; 
static int8_t SnrValue = 0;

static uint8_t sendingACKAndWaitData = 0; // 0 -- not 1 -- first 2 -- second
static uint8_t misDataCount = 0;
static uint32_t meshLoRaDataSequenceNum = 0;

#endif // __LORA_COMMISSIONING_H__
