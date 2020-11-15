/*!
 * \file      main.c
 *
 * \brief     Pure LoRa Measurement
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
#include "board.h"
#include "delay.h"
#include "gpio.h"
#include "gps.h"
#include "SHT2x.h"
#include "radio.h"
#include "serialio.h"
#include "timer.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// string format for longitude and latitude
char lng_s[20];
char lat_s[20];

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE 242

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

uint8_t pw_map[] = {20, 16, 14, 12, 10, 7, 5, 2};

// node state
// LSB and MSB are implementation-specific
typedef struct sState
{
    uint8_t id; // node id
    uint8_t is_on : 1; // flag: send pkt (is_on == 1) or not (is_on == 0)
    uint8_t sf : 3; // spreading factor [0:SF6, 1:SF7, 2:SF8, 3:SF9, 4:SF10, 5:SF11, 6:SF12]
    uint8_t cr : 2; // coderate [0: 4/5, 1: 4/6, 2: 4/7, 3: 4/8], +1 when passed to SetTxConfig
    uint8_t bw : 2; // bandwidth [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
    uint8_t pw_index : 3; // power index [0:20, 1:16, 2:14, 3:12, 4:10, 5:7, 6:5, 7:2] (dBm)
    uint8_t dc : 7; // tx dutycycle (>= 3s)
    uint8_t freq_index; // freq index: 96 channels
    uint8_t pkt_size; // packet size (>= 8 bytes)
    uint32_t cnt : 24; // packet counter
}State; // 8 bytes

State state =
{
    .id = 1,
    .bw = 0,
    .cr = 0,
    .sf = 6,
    .is_on = 0,
    .dc = 3,
    .pw_index = 0,
    .freq_index = 85,
    .pkt_size = 40,
    .cnt = 0
};

static uint8_t loc_node_id = 42;

static void onFhssChangeChannel(uint8_t s)
{
    printf("onFhssChangeChannel---%d\n",s);
    // if (state.cnt % 2 == 0)
    {
        uint8_t bseq[50] = {1,1,1,0,0,0,1,0,0,1,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,0,0,1,0,0};
        // uint8_t bseq[30] = {1,1,0,1,0,1,0,1,0,1,0,1,1,1,1,1,0,0,0,0,0,0,1,0,1,0,1,0,1,0,1};
        if (s<2 || bseq[s] == 1)
        {
            Radio.SetTxConfig(MODEM_LORA, pw_map[state.pw_index], 0, state.bw,
                            12, state.cr + 1,
                            8, false, true, true, 1, false, 3000);
        }
        else
        {
            Radio.SetTxConfig(MODEM_LORA, pw_map[state.pw_index], 0, state.bw,
                            11, state.cr + 1,
                            8, false, true, true, 1, false, 3000);
        }
    }
}

static void OnRadioTxDone(void)
{
    Radio.SetChannel(475500000); // Hz
    Radio.SetRxConfig(MODEM_LORA, 0, 12, 1, 0, 8, 5000, false, 0, false, 0, 0, false, true);
    Radio.Rx(0);
    // printf("OnRadioTxDone\n");
}

static void double2s(char *s, double d)
{
    char *tmpSign = (d < 0) ? "-" : "";
    float tmpVal = (d < 0) ? -d : d;

    int tmpInt1 = tmpVal;                  // Get the integer (678).
    float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123).
    int tmpInt2 = trunc(tmpFrac * 1000000);  // Turn into integer (123).

    // Print as parts, note that you need 0-padding for fractional bit.
    sprintf(s, "%s%d.%06d", tmpSign, tmpInt1, tmpInt2);
}

static void OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    printf("size:%d,snr:%d,rssi:%d\n", size, snr, rssi);
    uint32_t d_cnt = 0;
    double latitude = 0, longitude = 0;
    if (size == 4)
    {
        for (int i = 0; i < size; i++)
        {
            d_cnt += payload[i];
            if (i < size - 1) d_cnt <<= 8;
        }
        if( snr & 0x80 ) // The SNR sign bit is 1
        {
            // Invert and divide by 4
            snr = ( ( ~snr + 1 ) & 0xFF ) >> 2;
            snr = -snr;
        }
        else
        {
            // Divide by 4
            snr = ( snr & 0xFF ) >> 2;
        }
        GpsGetLatestGpsPositionDouble(&latitude, &longitude);

        double2s(lat_s, latitude);
        double2s(lng_s, longitude);
        // Print as parts, note that you need 0-padding for fractional bit.
        printf("cnt:%d,snr:%d,rssi:%d,lng:%s,lat:%s\n", d_cnt, snr, rssi, lng_s, lat_s);
    }

    if (size == 5 && *payload == loc_node_id)
    {
        memcpy(&state, payload, 5);
        state.cnt = 0;
    }
}

static void OnRadioRxError(void)
{
    // Radio.Sleep();
}

static void OnRadioTxTimeout(void)
{
    // Radio.Sleep();
}

static void OnRadioRxTimeout(void)
{
    // Radio.Sleep();
}

static void PreparePacket()
{
    static int32_t latitude = 0, longitude = 0;
    static uint16_t temperature = 0, humidity = 0;
    memcpy(AppData, &state, 8);      // store node id in AppData
    GpsGetLatestGpsPositionBinary(&latitude, &longitude);
    SHT2xGetTempHumi(&temperature, &humidity);

    AppData[8] = latitude & 0xFF;
    AppData[9] = ( latitude >> 8 ) & 0xFF;
    AppData[10] = ( latitude >> 16 ) & 0xFF;
    AppData[11] = longitude & 0xFF;
    AppData[12] = ( longitude >> 8 ) & 0xFF;
    AppData[13] = ( longitude >> 16 ) & 0xFF;
    AppData[14] = temperature & 0xFF;
    AppData[15] = ( temperature >> 8 ) & 0xFF;
    AppData[16] = humidity & 0xFF;
    AppData[17] = ( humidity >> 8 ) & 0xFF;

    for(uint16_t i = 0; i < LORAWAN_APP_DATA_MAX_SIZE; ++i)
    {
        AppData[i] = Random;
    }
}

/**
 * Main application entry point.
 */
int main(void)
{
    BoardInitMcu();
    BoardInitPeriph();

    // Initialize Radio driver
    RadioEvents.TxDone = OnRadioTxDone;
    RadioEvents.RxDone = OnRadioRxDone;
    RadioEvents.RxError = OnRadioRxError;
    RadioEvents.TxTimeout = OnRadioTxTimeout;
    RadioEvents.RxTimeout = OnRadioRxTimeout;
    RadioEvents.FhssChangeChannel = onFhssChangeChannel;
    Radio.Init(&RadioEvents);

    // Random seed initialization
    srand1(Radio.Random());

    bool PublicNetwork = true;
    Radio.SetPublicNetwork(PublicNetwork);
    // Radio.Sleep( );

    // log_info("scanf ID, PW, SF, BW, CR, TX_DUTYCYCLE, DATASIZE, FREQ\n");
    // scanf("%d", &loc_node_id);
    // scanf("%d", (uint32_t*)&power);
    // scanf("%d", &datarate);
    // scanf("%d", &bandwidth);
    // scanf("%d", (uint32_t*)&coderate);
    // scanf("%d", &tx_dutycycle);
    // scanf("%d", (uint32_t*)&data_size);
    // scanf("%d", &freq);

    // log_info("AppData: ");
    // for (int i = 0; i < data_size; i++)
    // {
    //     printf("%x", AppData[i]);
    // }
    // printf("\n");

    // OnRadioTxDone();
    // log_info("on radio tx done\n");
    uint32_t cnt = 0;

    while (1)
    {
        // if (!state.is_on)
        // {
        //     DelayMs(state.dc * 1000); // have a rest~~
        //     continue;
        // }

        log_info("circle oooo\n");

        /*!
        * \brief Sets the transmission parameters
        *
        * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
        * \param [IN] power        Sets the output power [dBm] [20, 16, 14, 12, 10, 7, 5, 2]
        * \param [IN] fdev         Sets the frequency deviation (FSK only)
        *                          FSK : [Hz]
        *                          LoRa: 0
        * \param [IN] bandwidth    Sets the bandwidth (LoRa only)
        *                          FSK : 0
        *                          LoRa: [0: 125 kHz, 1: 250 kHz,
        *                                 2: 500 kHz, 3: Reserved]
        * \param [IN] datarate     Sets the Datarate
        *                          FSK : 600..300000 bits/s
        *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
        *                                10: 1024, 11: 2048, 12: 4096  chips]
        * \param [IN] coderate     Sets the coding rate (LoRa only)
        *                          FSK : N/A ( set to 0 )
        *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
        * \param [IN] preambleLen  Sets the preamble length
        *                          FSK : Number of bytes
        *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
        * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
        * \param [IN] crcOn        Enables disables the CRC [0: OFF, 1: ON]
        * \param [IN] FreqHopOn    Enables disables the intra-packet frequency hopping
        *                          FSK : N/A ( set to 0 )
        *                          LoRa: [0: OFF, 1: ON]
        * \param [IN] HopPeriod    Number of symbols bewteen each hop
        *                          FSK : N/A ( set to 0 )
        *                          LoRa: Number of symbols
        * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
        *                          FSK : N/A ( set to 0 )
        *                          LoRa: [0: not inverted, 1: inverted]
        * \param [IN] timeout      Transmission timeout [ms]
        *
        void    ( *SetTxConfig )( RadioModems_t modem, int8_t power, uint32_t fdev,
                            uint32_t bandwidth, uint32_t datarate,
                            uint8_t coderate, uint16_t preambleLen,
                            bool fixLen, bool crcOn, bool FreqHopOn,
                            uint8_t HopPeriod, bool iqInverted, uint32_t timeout
        );*/
        Radio.Standby();
        Radio.SetChannel(480000000);
        Radio.SetTxConfig(MODEM_LORA, 10, 0, 0,
                            7, 1,
                            8, false, true, false, 1, false, 3000);

        /*!
        * \brief Sets the maximum payload length.
        *
        * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
        * \param [IN] max        Maximum payload length in bytes
        *
        void SX1276SetMaxPayloadLength( RadioModems_t modem, uint8_t max );*/
        // Setup maximum payload lenght of the radio driver
        Radio.SetMaxPayloadLength(MODEM_LORA, state.pkt_size);

        log_info("sent_cnt:%d\n", state.cnt++);
        
        PreparePacket();
        AppData[4] = cnt& 0xFF;
        cnt = cnt + 1;

        // Send now
        Radio.Send(AppData, state.pkt_size);

        
        // state.cnt++;
        // log_debug("sent_cnt:%d, power:%d, bw:%d, dr:%d, cr:%d, freq:%d\n", sent_cnt++,
                //   power, bandwidth, datarate, coderate, freq);

        DelayMs(5000); // have a rest~~
    }
}
