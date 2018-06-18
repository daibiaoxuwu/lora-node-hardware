/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: LoRaMac classA device implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include "board.h"
#include "serialio.h"
#include "delay.h"
#include "gpio.h"
#include "radio.h"
#include "timer.h"

#include "adc.h"
#include "rtc-board.h"
#include "utilities.h"
#include "mma8451.h"
#include "mpl3115.h"
#include "sx9500.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"
#include "gps.h"

#include "Comissioning.h"

bool isMeshLoRaPkts(void)
{
    MeshLoRaMacHeader_t mhdr;
    mhdr.Value = Buffer[0];
    if (mhdr.Bits.Major == 0 && mhdr.Bits.MType == 7 && mhdr.Bits.RFU == 1)
    { //packets of mesh lora
        return true;
    }
    return false;
}
int isRouterOrData(void) //0 neither 1 router 2 data
{
    MeshLoRaMacHeader_t mhdr;
    mhdr.Value = Buffer[0];
    if (mhdr.Bits.Major == 0 && mhdr.Bits.MType == 7 && mhdr.Bits.RFU == 1)
    { //packets of mesh lora
        if (Buffer[1] == 8)
        {
            return 1;
        }
        else if (Buffer[1] == 10 && Buffer[7] == ((uint16_t)DEVICE_ADDRESS & 0xFF) && Buffer[8] == (((uint16_t)DEVICE_ADDRESS >> 8) & 0xFF))
        {
            return 2;
        }
    }

    return 0;
}

void MeshLoRaUpdateRouterTable(uint16_t srcAddr, uint16_t desAddr, uint8_t cost)
{
    if (desAddr == (uint16_t)DEVICE_ADDRESS)
    {
        return;
    }

    //only to sink path
    if (desAddr != (uint16_t)GATEWAY_ADDRESS)
    {
        return;
    }

    //find desAddr
    int8_t desInd = -1;
    for (uint8_t i = 0; i < meshLoRaCurrentIndLen; i++)
    {
        //to do sort then search will be more effect
        if (meshLoRaCurrentInd[i] == desAddr)
        {
            desInd = i;
            break;
        }
    }

    if (desInd == -1)
    { //don't have, just add
        meshLoRaCurrentInd[meshLoRaCurrentIndLen] = desAddr;
        meshLoRaCurrentIndLen += 1;
        meshLoRaNxtAddr[desAddr] = srcAddr;
        meshLoRaCost[desAddr] = cost + 1;
        meshLoRaRssi[desAddr] = RssiValue;
        if (desAddr == (uint16_t)GATEWAY_ADDRESS)
        {
            if (SHOW_DEBUG_DETAIL)
            {
                printf("%d to GW by %d\n", (uint16_t)DEVICE_ADDRESS, srcAddr);
            }
            willSendRouter = true;
        }
        router_interval = ROUTER_MIN_INTERVAL;
        meshLoRaRouterSequenceNum = 0;
    }
    else
    {
        if (cost + 1 < meshLoRaCost[desAddr])
        { //less then update
            meshLoRaNxtAddr[desAddr] = srcAddr;
            meshLoRaCost[desAddr] = cost + 1;
            meshLoRaRssi[desAddr] = RssiValue;
            if (desAddr == (uint16_t)GATEWAY_ADDRESS)
            {
                if (SHOW_DEBUG_DETAIL)
                {
                    printf("%d to GW by %d\n", (uint16_t)DEVICE_ADDRESS, srcAddr);
                }
                willSendRouter = true;
            }
            router_interval = ROUTER_MIN_INTERVAL;
            meshLoRaRouterSequenceNum = 0;
        }
        else if (cost + 1 == meshLoRaCost[desAddr])
        {
            if (RssiValue > meshLoRaRssi[desAddr])
            {
                meshLoRaNxtAddr[desAddr] = srcAddr;
                meshLoRaRssi[desAddr] = RssiValue;
                if (desAddr == (uint16_t)GATEWAY_ADDRESS)
                {
                    if (SHOW_DEBUG_DETAIL)
                    {
                        printf("%d to GW by %d\n", (uint16_t)DEVICE_ADDRESS, srcAddr);
                    }
                    willSendRouter = true;
                }
                router_interval = ROUTER_MIN_INTERVAL;
                meshLoRaRouterSequenceNum = 0;
            }
        }
    }
}

void MeshLoRaAddRelayToRouterTable(void)
{
    meshLoRaCurrentIndLen += 1;
    meshLoRaCurrentInd[meshLoRaCurrentIndLen - 1] = (uint16_t)GATEWAY_ADDRESS;
    meshLoRaNxtAddr[(uint16_t)GATEWAY_ADDRESS] = (uint16_t)FIXED_RELAY_ADDRESS;
    meshLoRaCost[(uint16_t)GATEWAY_ADDRESS] = 0;
    meshLoRaRssi[(uint16_t)GATEWAY_ADDRESS] = 0;
}

/*
* prepare frame
*/
void MeshLoRaPrepareFrame(Packets_t pt)
{
    if (pt == RTS)
    {
        memset1(Buffer_send, 0, BUFFER_SIZE);
        BufferSize_send = 0;

        MeshLoRaFrameHeader_t meshLoRaFrHd;
        //Mhdr
        meshLoRaFrHd.Mhdr.Bits.Major = 0;
        meshLoRaFrHd.Mhdr.Bits.RFU = 1;
        meshLoRaFrHd.Mhdr.Bits.MType = 7;

        if ((bf_save_relay_now != bf_save_relay_have) || (bf_save_data_now != bf_save_data_have))
        {
            Buffer_send[BufferSize_send++] = meshLoRaFrHd.Mhdr.Value;
            Buffer_send[BufferSize_send++] = 1;
            Buffer_send[BufferSize_send++] = meshLoRaNxtAddr[(uint16_t)GATEWAY_ADDRESS] & 0xFF;
            Buffer_send[BufferSize_send++] = (meshLoRaNxtAddr[(uint16_t)GATEWAY_ADDRESS] >> 8) & 0xFF;
            Buffer_send[BufferSize_send++] = (uint16_t)DEVICE_ADDRESS & 0xFF;
            Buffer_send[BufferSize_send++] = ((uint16_t)DEVICE_ADDRESS >> 8) & 0xFF;
        }
        else if (willSendRouter)
        {
            Buffer_send[BufferSize_send++] = meshLoRaFrHd.Mhdr.Value;
            Buffer_send[BufferSize_send++] = 0;
            Buffer_send[BufferSize_send++] = (uint16_t)DEVICE_ADDRESS & 0xFF;
            Buffer_send[BufferSize_send++] = ((uint16_t)DEVICE_ADDRESS >> 8) & 0xFF;
        }
    }
    else if (pt == ACK)
    {
        MeshLoRaFrameHeader_t meshLoRaFrHd;
        uint8_t addr0 = 0, addr1 = 0;

        //Mhdr
        meshLoRaFrHd.Mhdr.Bits.Major = 0;
        meshLoRaFrHd.Mhdr.Bits.RFU = 1;
        meshLoRaFrHd.Mhdr.Bits.MType = 7;

        if (Buffer[1] == 0)
        {
            addr0 = Buffer[2];
            addr1 = Buffer[3];
            memset1(Buffer_send, 0, BUFFER_SIZE);
            BufferSize_send = 0;
            Buffer_send[BufferSize_send++] = meshLoRaFrHd.Mhdr.Value;
            Buffer_send[BufferSize_send++] = 2;
            Buffer_send[BufferSize_send++] = addr0;
            Buffer_send[BufferSize_send++] = addr1;
        }
        else if (Buffer[1] == 1)
        {
            uint8_t freq_hop_ind = randr(0, HOP_NUM - 1);
            rx_freq_ind = freq_hop_ind;
            addr0 = Buffer[4];
            addr1 = Buffer[5];
            memset1(Buffer_send, 0, BUFFER_SIZE);
            BufferSize_send = 0;
            Buffer_send[BufferSize_send++] = meshLoRaFrHd.Mhdr.Value;
            Buffer_send[BufferSize_send++] = 3 | (freq_hop_ind << 4);
            Buffer_send[BufferSize_send++] = addr0;
            Buffer_send[BufferSize_send++] = addr1;
        }
    }
    else if (pt == ROUTER)
    {
        memset1(Buffer_send, 0, BUFFER_SIZE);
        BufferSize_send = 0;
        //empty router table
        if (meshLoRaCurrentIndLen == 0)
        {
            return;
        }

        MeshLoRaFrameHeader_t meshLoRaFrHd;
        uint8_t i = 0;

        //Mhdr
        meshLoRaFrHd.Mhdr.Bits.Major = 0;
        meshLoRaFrHd.Mhdr.Bits.RFU = 1;
        meshLoRaFrHd.Mhdr.Bits.MType = 7;

        if ((uint16_t)DEVICE_ADDRESS == (uint16_t)GATEWAY_ADDRESS)
        {
            meshLoRaFrHd.FrameType = (1 << 8) | 8;
        }
        else
        {
            meshLoRaFrHd.FrameType = (0 << 8) | 8;
        }

        meshLoRaFrHd.FrameCnt = meshLoRaRouterSequenceNum % 255;
        meshLoRaRouterSequenceNum += 1;

        meshLoRaFrHd.FramePayloadLen = 3 * meshLoRaCurrentIndLen + 3;

        //header
        Buffer_send[BufferSize_send++] = meshLoRaFrHd.Mhdr.Value;
        Buffer_send[BufferSize_send++] = meshLoRaFrHd.FrameType & 0xFF;
        Buffer_send[BufferSize_send++] = (meshLoRaFrHd.FrameType >> 8) & 0xFF;
        Buffer_send[BufferSize_send++] = meshLoRaFrHd.FrameCnt;
        Buffer_send[BufferSize_send++] = meshLoRaFrHd.FramePayloadLen;

        //payload
        Buffer_send[BufferSize_send++] = (uint16_t)DEVICE_ADDRESS & 0xFF;
        Buffer_send[BufferSize_send++] = ((uint16_t)DEVICE_ADDRESS >> 8) & 0xFF;
        Buffer_send[BufferSize_send++] = meshLoRaCurrentIndLen;

        //des addr
        for (i = 0; i < meshLoRaCurrentIndLen; i++)
        {
            Buffer_send[BufferSize_send++] = meshLoRaCurrentInd[i] & 0xFF;
            Buffer_send[BufferSize_send++] = (meshLoRaCurrentInd[i] >> 8) & 0xFF;
        }

        //cost
        for (i = 0; i < meshLoRaCurrentIndLen; i++)
        {
            Buffer_send[BufferSize_send++] = meshLoRaCost[meshLoRaCurrentInd[i]];
        }
    }
    else if (pt == DATA)
    {
        uint8_t buffer_tmp[BUFFER_SIZE];
        uint8_t buffer_size_tmp = 0;
        memset1(buffer_tmp, 0, BUFFER_SIZE);

        MeshLoRaFrameHeader_t meshLoRaFrHd;
        uint8_t i = 0;

        //Mhdr
        meshLoRaFrHd.Mhdr.Bits.Major = 0;
        meshLoRaFrHd.Mhdr.Bits.RFU = 1;
        meshLoRaFrHd.Mhdr.Bits.MType = 7;

        meshLoRaFrHd.FrameType = (0 << 8) | 10;

        meshLoRaFrHd.FrameCnt = meshLoRaDataSequenceNum % 255;
        meshLoRaDataSequenceNum += 1;

        uint8_t AppData[MESHLORA_APPDATA_PAYLOAD_LENGTH];
        AppData[0] = (uint8_t)(02 & 0xFF);
        AppData[1] = (uint8_t)(00 & 0xFF);
        AppData[2] = (uint8_t)(06 & 0xFF);
        AppData[3] = (uint8_t)(00 & 0xFF);
        AppData[4] = (uint8_t)(00 & 0xFF);
        AppData[5] = (uint8_t)(02 & 0xFF);
        AppData[6] = (uint8_t)(01 & 0xFF);
        AppData[7] = (uint8_t)(00 & 0xFF);

        meshLoRaFrHd.FramePayloadLen = 4 + MESHLORA_APPDATA_PAYLOAD_LENGTH;

        //header
        buffer_tmp[buffer_size_tmp++] = meshLoRaFrHd.Mhdr.Value;
        buffer_tmp[buffer_size_tmp++] = meshLoRaFrHd.FrameType & 0xFF;
        buffer_tmp[buffer_size_tmp++] = (meshLoRaFrHd.FrameType >> 8) & 0xFF;
        buffer_tmp[buffer_size_tmp++] = meshLoRaFrHd.FrameCnt;
        buffer_tmp[buffer_size_tmp++] = meshLoRaFrHd.FramePayloadLen;

        //payload
        buffer_tmp[buffer_size_tmp++] = (uint16_t)DEVICE_ADDRESS & 0xFF;
        buffer_tmp[buffer_size_tmp++] = ((uint16_t)DEVICE_ADDRESS >> 8) & 0xFF;
        //des addr
        buffer_tmp[buffer_size_tmp++] = meshLoRaNxtAddr[(uint16_t)GATEWAY_ADDRESS] & 0xFF;
        buffer_tmp[buffer_size_tmp++] = (meshLoRaNxtAddr[(uint16_t)GATEWAY_ADDRESS] >> 8) & 0xFF;

        for (i = 0; i < MESHLORA_APPDATA_PAYLOAD_LENGTH; i++)
        {
            buffer_tmp[buffer_size_tmp++] = AppData[i];
        }
        //add to save buffer
        if ((bf_save_data_have + 1) % BUFFER_SAVE_DATA == bf_save_data_now)
        { //full
            lost_data += 1;
            printf("lst data: %d\n", lost_data);
        }
        else
        {
            memset1(Buffer_save_data[bf_save_data_have], 0, BUFFER_SIZE);
            memcpy1(Buffer_save_data[bf_save_data_have], buffer_tmp, buffer_size_tmp);
            Buffer_save_data_len[bf_save_data_have] = buffer_size_tmp;
            bf_save_data_have += 1;
            bf_save_data_have = bf_save_data_have % BUFFER_SAVE_DATA;
        }
    }
}

/*
* send
*/
void relayData(void)
{
    if (getAck)
    {
        if (rxNotimerOut)
        {
            //printf("stby\n");
            Radio.Standby();
            rxNotimerOut = false;
        }
        Ptype = RELAY;
        DelayMs(1);
        if (tx_freq_ind == -1)
        {
            if (SHOW_FREQ_HOP)
            {
                printf("freq_ind %d\n", -1);
            }
            Radio.SetChannel(RF_FREQUENCY);
        }
        else
        {
            if (SHOW_FREQ_HOP)
            {
                printf("freq_ind %d\n", tx_freq_ind);
            }
            Radio.SetChannel(freq_hop[tx_freq_ind]);
        }
        Radio.Send(Buffer_save_relay[bf_save_relay_now], Buffer_save_relay_len[bf_save_relay_now]);
    }
    else
    {
        if (rxNotimerOut)
        {
            //printf("stby\n");
            Radio.Standby();
            rxNotimerOut = false;
            DelayMs(1);
        }
        Radio.StartCad();
    }
}
void sendData(void)
{
    if (getAck)
    {
        if (rxNotimerOut)
        {
            //printf("stby\n");
            Radio.Standby();
            rxNotimerOut = false;
        }
        Ptype = DATA;
        if (SHOW_TIMEONAIR)
        {
            TimerTime_t TxTimeOnAir = Radio.TimeOnAir(MODEM_LORA, Buffer_save_data_len[bf_save_data_now]);
            printf("TAir %d %dms\n", Buffer_save_data_len[bf_save_data_now], TxTimeOnAir);
        }
        DelayMs(1);
        if (tx_freq_ind == -1)
        {
            if (SHOW_FREQ_HOP)
            {
                printf("freq_ind %d\n", -1);
            }
            Radio.SetChannel(RF_FREQUENCY);
        }
        else
        {
            if (SHOW_FREQ_HOP)
            {
                printf("freq_ind %d\n", tx_freq_ind);
            }
            Radio.SetChannel(freq_hop[tx_freq_ind]);
        }
        Radio.Send(Buffer_save_data[bf_save_data_now], Buffer_save_data_len[bf_save_data_now]);
    }
    else
    {
        if (rxNotimerOut)
        {
            //printf("stby\n");
            Radio.Standby();
            rxNotimerOut = false;
            DelayMs(1);
        }
        Radio.StartCad();
    }
}
void sendRouter(void)
{
    if (getAck)
    {
        if (rxNotimerOut)
        {
            //printf("stby\n");
            Radio.Standby();
            rxNotimerOut = false;
        }
        Ptype = ROUTER;
        MeshLoRaPrepareFrame(ROUTER);
        if (SHOW_TIMEONAIR)
        {
            TimerTime_t TxTimeOnAir = Radio.TimeOnAir(MODEM_LORA, BufferSize_send);
            printf("TAir %d %dms\n", BufferSize_send, TxTimeOnAir);
        }
        DelayMs(1);
        if (tx_freq_ind == -1)
        {
            if (SHOW_FREQ_HOP)
            {
                printf("freq_ind %d\n", -1);
            }
            Radio.SetChannel(RF_FREQUENCY);
        }
        else
        {
            if (SHOW_FREQ_HOP)
            {
                printf("freq_ind %d\n", tx_freq_ind);
            }
            Radio.SetChannel(freq_hop[tx_freq_ind]);
        }
        Radio.Send(Buffer_send, BufferSize_send);
    }
    else
    {
        if (rxNotimerOut)
        {
            //printf("stby\n");
            Radio.Standby();
            rxNotimerOut = false;
            DelayMs(1);
        }
        Radio.StartCad();
    }
}

//decide how to do after rx data or not
void checkAndSendPkts(bool getData)
{
    if (getData)
    {
        if (sendingACKAndWaitData == 0 || sendingACKAndWaitData == 2)
        {
            if (sendingACKAndWaitData == 2)
            {
                if (SHOW_DEBUG_DETAIL)
                {
                    printf("dt0\n");
                }
                sendingACKAndWaitData = 0;
            }

            if (bf_save_relay_now != bf_save_relay_have)
            {
                if (SHOW_DEBUG_DETAIL)
                {
                    printf("relayData\n");
                }
                relayData();
            }
            else if (bf_save_data_now != bf_save_data_have && isConnectToGW)
            {
                if (SHOW_DEBUG_DETAIL)
                {
                    printf("sendData\n");
                }
                sendData();
            }
            else if (willSendRouter)
            {
                if (SHOW_DEBUG_DETAIL)
                {
                    printf("sendRouter\n");
                }
                sendRouter();
            }
            else if (Radio.GetStatus() == RF_IDLE)
            {
                if (SHOW_DEBUG_DETAIL)
                {
                    printf("rx1 0\n");
                }
                if (rx_freq_ind == -1)
                {
                    if (SHOW_FREQ_HOP)
                    {
                        printf("freq_ind %d\n", -1);
                    }
                    Radio.SetChannel(RF_FREQUENCY);
                }
                else
                {
                    if (SHOW_FREQ_HOP)
                    {
                        printf("freq_ind %d\n", rx_freq_ind);
                    }
                    Radio.SetChannel(freq_hop[rx_freq_ind]);
                }
                Radio.Rx(0);
                rxNotimerOut = true;
                //printf("(rd)%d\n", Radio.GetStatus());
                if (dcState == MID_SLEEP)
                {
                    //printf("stby\n");
                    Radio.Standby();
                    rxNotimerOut = false;
                    DelayMs(1);
                    Radio.Sleep();
                    dcState = SLEEP;
                    printf("(%d)sleep\n", RtcGetTimerValue());
                    //printf("(rd)%d\n", Radio.GetStatus());
                    TimerSetValue(&DutyCycleTimer, SLEEPTIME);
                    TimerStart(&DutyCycleTimer);
                }
            }
            else
            {
                printf("stil1 %d\n", Radio.GetStatus());
            }
        }
        else if (sendingACKAndWaitData == 1)
        {
            if (SHOW_DEBUG_DETAIL)
            {
                printf("dt2 Rx %d\n", WAITFORDATATIME);
            }
            sendingACKAndWaitData = 2;
            if (rx_freq_ind == -1)
            {
                if (SHOW_FREQ_HOP)
                {
                    printf("freq_ind %d\n", -1);
                }
                Radio.SetChannel(RF_FREQUENCY);
            }
            else
            {
                if (SHOW_FREQ_HOP)
                {
                    printf("freq_ind %d\n", rx_freq_ind);
                }
                Radio.SetChannel(freq_hop[rx_freq_ind]);
            }
            Radio.Rx(WAITFORDATATIME);
            //printf("(rd)%d\n", Radio.GetStatus());
        }
        else
        {
            printf("err1\n");
        }
    }
    else
    {
        //not router & exact data
        if (sendingACKAndWaitData != 0)
        {
            if (SHOW_DEBUG_DETAIL)
            {
                printf("mis\n");
            }
            sendingACKAndWaitData = 0;
        }
        if (bf_save_relay_now != bf_save_relay_have)
        {
            if (SHOW_DEBUG_DETAIL)
            {
                printf("relayData\n");
            }
            relayData();
        }
        else if (bf_save_data_now != bf_save_data_have && isConnectToGW)
        {
            if (SHOW_DEBUG_DETAIL)
            {
                printf("sendData\n");
            }
            sendData();
        }
        else if (willSendRouter)
        {
            if (SHOW_DEBUG_DETAIL)
            {
                printf("sendRouter\n");
            }
            sendRouter();
        }
        else if (Radio.GetStatus() == RF_IDLE)
        {
            if (SHOW_DEBUG_DETAIL)
            {
                printf("rx2 0\n");
            }
            if (rx_freq_ind == -1)
            {
                if (SHOW_FREQ_HOP)
                {
                    printf("freq_ind %d\n", -1);
                }
                Radio.SetChannel(RF_FREQUENCY);
            }
            else
            {
                if (SHOW_FREQ_HOP)
                {
                    printf("freq_ind %d\n", rx_freq_ind);
                }
                Radio.SetChannel(freq_hop[rx_freq_ind]);
            }
            Radio.Rx(0);
            rxNotimerOut = true;
            //printf("(rd)%d\n", Radio.GetStatus());
            if (dcState == MID_SLEEP)
            {
                //printf("stby\n");
                Radio.Standby();
                rxNotimerOut = false;
                DelayMs(1);
                Radio.Sleep();
                dcState = SLEEP;
                printf("(%d)sleep\n", RtcGetTimerValue());
                //printf("(rd)%d\n", Radio.GetStatus());
                TimerSetValue(&DutyCycleTimer, SLEEPTIME);
                TimerStart(&DutyCycleTimer);
            }
        }
        else
        {
            printf("stil2 %d\n", Radio.GetStatus());
        }
    }
}

/*
* events
*/
void OnSendRouterTimerEvent(void)
{
    if (SHOW_DEBUG_DETAIL)
    {
        printf("evt router\n");
    }
    TimerStop(&SendRouterTimer);

    if (meshLoRaRouterSequenceNum == 20)
    {
        router_interval = router_interval * 2;
    }
    else if (meshLoRaRouterSequenceNum == 50)
    {
        router_interval = router_interval * 2;
    }
    else if (meshLoRaRouterSequenceNum == 100)
    {
        router_interval = router_interval * 2;
    }
    else if (meshLoRaRouterSequenceNum == 500)
    {
        router_interval = router_interval * 2;
    }
    else if (meshLoRaRouterSequenceNum == 1000)
    {
        router_interval = router_interval * 2;
    }
    else
    {
        router_interval = ROUTER_MAX_INTERVAL;
    }

    router_interval = (router_interval > ROUTER_MAX_INTERVAL) ? ROUTER_MAX_INTERVAL : router_interval;
    TimerSetValue(&SendRouterTimer, router_interval);

    if (willSendRouter)
    {
        TimerStart(&SendRouterTimer);
        return;
    }

    willSendRouter = true;

    TimerStart(&SendRouterTimer);

    if (rxNotimerOut && dcState != MID_SLEEP)
    {
        if (SHOW_DEBUG_DETAIL)
        {
            printf("newCc router\n");
        }
        sendRouter();
    }
}
void OnSendDataTimerEvent(void)
{
    if (SHOW_DEBUG_DETAIL)
    {
        printf("evt data\n");
    }
    TimerStop(&SendDataTimer);

    if (isConnectToGW)
    {
        MeshLoRaPrepareFrame(DATA);
    }

    TimerStart(&SendDataTimer);

    if (rxNotimerOut && isConnectToGW && dcState != MID_SLEEP)
    {
        if (SHOW_DEBUG_DETAIL)
        {
            printf("newCc Data\n");
        }
        sendData();
    }
}
void OnBackOffTimerEvent(void)
{
    if (SHOW_DEBUG_DETAIL)
    {
        printf("evt back-off\n");
    }
    TimerStop(&BackOffTimer);
    checkAndSendPkts(false);
}
void OnDutyCycleTimerEvent(void)
{
    if (SHOW_DEBUG_DETAIL)
    {
        printf("duty-cycle\n");
    }
    TimerStop(&DutyCycleTimer);
    if (dcState == AWAKE)
    {
        dcState = MID_SLEEP;
        if (SHOW_DEBUG_DETAIL)
        {
            printf("to sleep\n");
        }
        if (rxNotimerOut)
        {
            //printf("(rd)%d\n", Radio.GetStatus());
            //printf("stby\n");
            Radio.Standby();
            rxNotimerOut = false;
            DelayMs(1);
            Radio.Sleep();
            dcState = SLEEP;
            printf("(%d)sleep\n", RtcGetTimerValue());
            //printf("(rd)%d\n", Radio.GetStatus());
            TimerSetValue(&DutyCycleTimer, SLEEPTIME);
            TimerStart(&DutyCycleTimer);
        }
    }
    else if (dcState == SLEEP)
    {
        dcState = AWAKE;
        printf("(%d)awake\n", RtcGetTimerValue());
        TimerSetValue(&DutyCycleTimer, AWAKETIME);
        TimerStart(&DutyCycleTimer);
        checkAndSendPkts(false);
    }
}
void OnCADAaginTimerEvent(void)
{
    if (SHOW_DEBUG_DETAIL)
    {
        printf("cad-ag\n");
    }
    TimerStop(&CADAgainTimer);
    Radio.StartCad();
}

/*
* radio callbacks
*/
void OnTxDone(void)
{
    if (SHOW_DEBUG_DETAIL)
    {
        printf("(%d)txd\n", RtcGetTimerValue());
    }
    //printf("(rd)%d\n", Radio.GetStatus());
    Radio.Sleep();
    //printf("(rd)%d\n", Radio.GetStatus());
    State = TX;
}
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    if (SHOW_DEBUG_DETAIL)
    {
        printf("(%d)rxd\n", RtcGetTimerValue());
    }
    /*
	for(uint8_t i = 0; i < size; ++i){
			printf("%d ", payload[i]);
	}
	printf("\n");
	*/
    //printf("(rd)%d\n", Radio.GetStatus());
    if (rxNotimerOut)
    {
        //printf("stby\n");
        Radio.Standby();
        rxNotimerOut = false;
        DelayMs(1);
    }
    BufferSize = size;
    memset1(Buffer, 0, BUFFER_SIZE);
    memcpy1(Buffer, payload, BufferSize);
    RssiValue = rssi;
    SnrValue = snr;
    Radio.Sleep();
    //printf("(rd)%d\n", Radio.GetStatus());
    State = RX;
}
void OnTxTimeout(void)
{
    printf("tx tmout\n");
    //printf("(rd)%d\n", Radio.GetStatus());
    Radio.Sleep();
    //printf("(rd)%d\n", Radio.GetStatus());
    State = TX_TIMEOUT;
}
void OnRxTimeout(void)
{
    if (SHOW_DEBUG_DETAIL)
    {
        printf("rx tmout\n");
    }
    //printf("(rd)%d\n", Radio.GetStatus());
    Radio.Sleep();
    //printf("(rd)%d\n", Radio.GetStatus());
    State = RX_TIMEOUT;
}
void OnRxError(void)
{
    printf("(%d)rx error\n", RtcGetTimerValue());
    //printf("(rd)%d\n", Radio.GetStatus());
    if (rxNotimerOut)
    {
        //printf("stby\n");
        Radio.Standby();
        rxNotimerOut = false;
        DelayMs(1);
    }
    Radio.Sleep();
    //printf("(rd)%d\n", Radio.GetStatus());
    State = RX_ERROR;
}
void OnCadDone(bool channelActivityDetected)
{ //only relayData, sendData, sendRouter calls or again cadTimer
    if (SHOW_DEBUG_DETAIL)
    {
        printf("CAD Done\n");
    }
    //printf("(rd)%d\n", Radio.GetStatus());
    meshLoRaChannelActivityDetected = channelActivityDetected;
    Radio.Sleep();
    //printf("(rd)%d\n", Radio.GetStatus());
    State = CAD;
}

/*
* init
*/
void MeshLoRaRadioInit(void)
{
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    RadioEvents.CadDone = OnCadDone;

    Radio.Init(&RadioEvents);

    Radio.SetChannel(RF_FREQUENCY);
    //Radio.SetMaxPayloadLength(MODEM_LORA, BUFFER_SIZE);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
}
void MeshLoRaPacketTimerInit(void)
{
    TimerInit(&SendRouterTimer, OnSendRouterTimerEvent);
    TimerSetValue(&SendRouterTimer, router_interval);

    if ((uint16_t)DEVICE_ADDRESS != (uint16_t)GATEWAY_ADDRESS)
    {
        TimerInit(&SendDataTimer, OnSendDataTimerEvent);
        TimerSetValue(&SendDataTimer, DATAINTERVAL);
        TimerStart(&SendDataTimer);
    }

    TimerInit(&BackOffTimer, OnBackOffTimerEvent);
    TimerInit(&DutyCycleTimer, OnDutyCycleTimerEvent);
    TimerInit(&CADAgainTimer, OnCADAaginTimerEvent);
}
void MeshLoRaRouterTableInit(void)
{
    meshLoRaCurrentIndLen += 1;
    meshLoRaCurrentInd[meshLoRaCurrentIndLen - 1] = (uint16_t)DEVICE_ADDRESS;
    meshLoRaNxtAddr[(uint16_t)DEVICE_ADDRESS] = (uint16_t)DEVICE_ADDRESS;
    meshLoRaCost[(uint16_t)DEVICE_ADDRESS] = 0;
    meshLoRaRssi[(uint16_t)DEVICE_ADDRESS] = 0;
}

/**
 * Main application entry point.
 */
int main(void)
{
    // Target board initialization
    BoardInitMcu();
    BoardInitPeriph();

    // Radio initialization
    MeshLoRaRadioInit();
    // Packet-timer init
    MeshLoRaPacketTimerInit();
    // Router Table init
    if (MESHLORA_FIX_RELAY)
    {
        MeshLoRaAddRelayToRouterTable();
        if ((uint16_t)DEVICE_ADDRESS != (uint16_t)GATEWAY_ADDRESS)
        {
            isConnectToGW = true;
        }
    }
    if ((uint16_t)DEVICE_ADDRESS == (uint16_t)GATEWAY_ADDRESS)
    {
        if (!MESHLORA_FIX_RELAY)
        {
            MeshLoRaRouterTableInit();
        }
        willSendRouter = true;
    }

    if (willSendRouter)
    {
        if (SHOW_DEBUG_DETAIL)
        {
            printf("sendRouter\n");
        }
        sendRouter();
    }
    else
    {
        if (SHOW_DEBUG_DETAIL)
        {
            printf("rx 0\n");
        }
        if (rx_freq_ind == -1)
        {
            if (SHOW_FREQ_HOP)
            {
                printf("freq_ind %d\n", -1);
            }
            Radio.SetChannel(RF_FREQUENCY);
        }
        else
        {
            if (SHOW_FREQ_HOP)
            {
                printf("freq_ind %d\n", rx_freq_ind);
            }
            Radio.SetChannel(freq_hop[rx_freq_ind]);
        }
        Radio.Rx(0);
        rxNotimerOut = true;
        //printf("(rd)%d\n", Radio.GetStatus());
    }

    //begin duty-cycle
    if ((uint16_t)DEVICE_ADDRESS != (uint16_t)GATEWAY_ADDRESS)
    {
        TimerSetValue(&DutyCycleTimer, AWAKETIME);
        TimerStart(&DutyCycleTimer);
    }

    while (1)
    {
        switch (State)
        {
        case RX:
            if (BufferSize == RTS0_ACKS_SIZE && isMeshLoRaPkts())
            {
                if (sendingACKAndWaitData == 1)
                {
                    //must wait for data at least 2 times
                    if (misDataCount == 2)
                    {
                        if (SHOW_DEBUG_DETAIL)
                        {
                            printf("mis out\n");
                        }
                        misDataCount = 0;
                        sendingACKAndWaitData = 0;
                        rx_freq_ind = -1;
                    }
                    else
                    {
                        misDataCount += 1;
                        if (SHOW_DEBUG_DETAIL)
                        {
                            printf("%d, mis %d\n", BufferSize, misDataCount);
                        }
                        if (rx_freq_ind == -1)
                        {
                            if (SHOW_FREQ_HOP)
                            {
                                printf("freq_ind %d\n", -1);
                            }
                            Radio.SetChannel(RF_FREQUENCY);
                        }
                        else
                        {
                            if (SHOW_FREQ_HOP)
                            {
                                printf("freq_ind %d\n", rx_freq_ind);
                            }
                            Radio.SetChannel(freq_hop[rx_freq_ind]);
                        }
                        Radio.Rx(WAITFORDATATIME);
                        //printf("(rd)%d\n", Radio.GetStatus());
                        State = LOWPOWER;
                        break;
                    }
                }
                if (Buffer[1] == 0)
                { //rts0
                    if (SHOW_DEBUG_DETAIL)
                    {
                        printf("rx rts0\n");
                    }
                    Ptype = ACK;
                    MeshLoRaPrepareFrame(ACK);
                    sendingACKAndWaitData = 1;
                    misDataCount = 0;
                    if (SHOW_TIMEONAIR)
                    {
                        TimerTime_t TxTimeOnAir = Radio.TimeOnAir(MODEM_LORA, BufferSize_send);
                        printf("TAir %d %dms\n", BufferSize_send, TxTimeOnAir);
                    }
                    DelayMs(1);
                    if (tx_freq_ind == -1)
                    {
                        if (SHOW_FREQ_HOP)
                        {
                            printf("freq_ind %d\n", -1);
                        }
                        Radio.SetChannel(RF_FREQUENCY);
                    }
                    else
                    {
                        if (SHOW_FREQ_HOP)
                        {
                            printf("freq_ind %d\n", tx_freq_ind);
                        }
                        Radio.SetChannel(freq_hop[tx_freq_ind]);
                    }
                    Radio.Send(Buffer_send, BufferSize_send);
                }
                else if ((Buffer[1] & 0x0F) == 2)
                { //ack0
                    if (Buffer[2] == ((uint16_t)DEVICE_ADDRESS & 0xFF) && (Buffer[3] == (((uint16_t)DEVICE_ADDRESS >> 8) & 0xFF)))
                    {
                        if (SHOW_DEBUG_DETAIL)
                        {
                            printf("(%d)rx ack0\n", RtcGetTimerValue());
                        }
                        getAck = true;
                        sendRouter();
                    }
                    else
                    {
                        //back off
                        if (Ptype == RTS)
                        {
                            if (SHOW_DEBUG_DETAIL)
                            {
                                printf("back-off\n");
                            }
                            TimerStop(&BackOffTimer);
                            TimerSetValue(&BackOffTimer, randr(56, BACKOFFTIME));
                            TimerStart(&BackOffTimer);
                        }
                        else
                        {
                            if (SHOW_DEBUG_DETAIL)
                            {
                                printf("rx randr\n");
                            }
                            if (rx_freq_ind == -1)
                            {
                                if (SHOW_FREQ_HOP)
                                {
                                    printf("freq_ind %d\n", -1);
                                }
                                Radio.SetChannel(RF_FREQUENCY);
                            }
                            else
                            {
                                if (SHOW_FREQ_HOP)
                                {
                                    printf("freq_ind %d\n", rx_freq_ind);
                                }
                                Radio.SetChannel(freq_hop[rx_freq_ind]);
                            }
                            Radio.Rx(randr(56, BACKOFFTIME));
                        }
                    }
                }
                else if ((Buffer[1] & 0x0F) == 3)
                { //ack1
                    if (Buffer[2] == ((uint16_t)DEVICE_ADDRESS & 0xFF) && (Buffer[3] == (((uint16_t)DEVICE_ADDRESS >> 8) & 0xFF)))
                    {
                        if (SHOW_DEBUG_DETAIL)
                        {
                            printf("(%d)rx ack1\n", RtcGetTimerValue());
                        }
                        getAck = true;
                        if (bf_save_relay_now != bf_save_relay_have)
                        {
                            tx_freq_ind = (Buffer[1] & 0xF0) >> 4;
                            relayData();
                        }
                        else if (bf_save_data_now != bf_save_data_have)
                        {
                            tx_freq_ind = (Buffer[1] & 0xF0) >> 4;
                            sendData();
                        }
                        else
                        {
                            printf("err2\n");
                        }
                    }
                    else
                    {
                        //back off
                        if (Ptype == RTS)
                        {
                            if (SHOW_DEBUG_DETAIL)
                            {
                                printf("back-off\n");
                            }
                            TimerStop(&BackOffTimer);
                            TimerSetValue(&BackOffTimer, randr(56, BACKOFFTIME));
                            TimerStart(&BackOffTimer);
                        }
                        else
                        {
                            if (SHOW_DEBUG_DETAIL)
                            {
                                printf("rx randr\n");
                            }
                            if (rx_freq_ind == -1)
                            {
                                if (SHOW_FREQ_HOP)
                                {
                                    printf("freq_ind %d\n", -1);
                                }
                                Radio.SetChannel(RF_FREQUENCY);
                            }
                            else
                            {
                                if (SHOW_FREQ_HOP)
                                {
                                    printf("freq_ind %d\n", rx_freq_ind);
                                }
                                Radio.SetChannel(freq_hop[rx_freq_ind]);
                            }
                            Radio.Rx(randr(56, BACKOFFTIME));
                        }
                    }
                }
                else
                {
                    //other pkts
                    if (SHOW_DEBUG_DETAIL)
                    {
                        printf("sther pkts %d\n", BufferSize);
                    }

                    if (dcState == MID_SLEEP)
                    {
                        //begin duty-cycle
                        dcState = SLEEP;
                        printf("(%d)sleep\n", RtcGetTimerValue());
                        TimerStop(&DutyCycleTimer);
                        TimerSetValue(&DutyCycleTimer, SLEEPTIME);
                        TimerStart(&DutyCycleTimer);
                        State = LOWPOWER;
                        break;
                    }

                    checkAndSendPkts(false);
                }
            }
            else if (BufferSize == RTS1_SIZE && isMeshLoRaPkts() && Buffer[1] == 1 && (Buffer[2] == ((uint16_t)DEVICE_ADDRESS & 0xFF)) && (Buffer[3] == (((uint16_t)DEVICE_ADDRESS >> 8) & 0xFF)))
            { //rts1
                if (sendingACKAndWaitData == 1)
                {
                    //must wait for data at least 2 times
                    if (misDataCount == 2)
                    {
                        misDataCount = 0;
                        sendingACKAndWaitData = 0;
                        rx_freq_ind = -1;
                    }
                    else
                    {
                        misDataCount += 1;
                        if (SHOW_DEBUG_DETAIL)
                        {
                            printf("%d, mis %d\n", BufferSize, misDataCount);
                        }
                        if (rx_freq_ind == -1)
                        {
                            if (SHOW_FREQ_HOP)
                            {
                                printf("freq_ind %d\n", -1);
                            }
                            Radio.SetChannel(RF_FREQUENCY);
                        }
                        else
                        {
                            if (SHOW_FREQ_HOP)
                            {
                                printf("freq_ind %d\n", rx_freq_ind);
                            }
                            Radio.SetChannel(freq_hop[rx_freq_ind]);
                        }
                        Radio.Rx(WAITFORDATATIME);
                        //printf("(rd)%d\n", Radio.GetStatus());
                        State = LOWPOWER;
                        break;
                    }
                }
                if (SHOW_DEBUG_DETAIL)
                {
                    printf("rx rts1\n");
                }
                if (bf_save_relay_now == (bf_save_relay_have + 1) % BUFFER_SAVE_RELAY)
                { //already have frames to relay
                    if (SHOW_DEBUG_DETAIL)
                    {
                        printf("fulRe\n");
                    }
                    relayData();
                }
                else
                {
                    //back ack
                    Ptype = ACK;
                    MeshLoRaPrepareFrame(ACK);
                    sendingACKAndWaitData = 1;
                    misDataCount = 0;
                    if (SHOW_TIMEONAIR)
                    {
                        TimerTime_t TxTimeOnAir = Radio.TimeOnAir(MODEM_LORA, BufferSize_send);
                        printf("TAir %d %dms\n", BufferSize_send, TxTimeOnAir);
                    }
                    DelayMs(1);
                    if (tx_freq_ind == -1)
                    {
                        if (SHOW_FREQ_HOP)
                        {
                            printf("freq_ind %d\n", -1);
                        }
                        Radio.SetChannel(RF_FREQUENCY);
                    }
                    else
                    {
                        if (SHOW_FREQ_HOP)
                        {
                            printf("freq_ind %d\n", tx_freq_ind);
                        }
                        Radio.SetChannel(freq_hop[tx_freq_ind]);
                    }
                    Radio.Send(Buffer_send, BufferSize_send);
                }
            }
            else
            {
                int pkt = isRouterOrData();
                if (pkt == 1)
                { //router
                    if (SHOW_DEBUG_DETAIL)
                    {
                        printf("rx router\n");
                    }
                    rx_freq_ind = -1;
                    misDataCount = 0;
                    if (MESHLORA_FIX_RELAY)
                    {
                        if (!beginRouterTimer)
                        {
                            beginRouterTimer = true;
                            // isConnectToGW = true;
                            TimerStart(&SendRouterTimer);
                        }
                    }
                    else
                    {
                        uint16_t srcAddr = Buffer[5] | (Buffer[6] << 8);
                        if (srcAddr != (uint16_t)DEVICE_ADDRESS)
                        {
                            uint8_t desNum = Buffer[7];
                            //update router table
                            uint8_t desBeginInd = 8;
                            uint8_t cosBeginInd = 8 + desNum * 2;
                            for (int i = 0; i < desNum; i++)
                            {
                                uint16_t desAddr = Buffer[desBeginInd] | (Buffer[desBeginInd + 1] << 8);
                                uint8_t cost = Buffer[cosBeginInd];
                                desBeginInd += 2;
                                cosBeginInd += 1;
                                if (isConnectToGW == false && desAddr == (uint16_t)GATEWAY_ADDRESS) //update 'isConnectToGW'
                                {
                                    isConnectToGW = true;
                                }
                                MeshLoRaUpdateRouterTable(srcAddr, desAddr, cost);
                            }
                        }
                    }

                    checkAndSendPkts(true);
                }
                else if (pkt == 2)
                { //data
                    if (SHOW_DEBUG_DETAIL)
                    {
                        printf("rx data\n");
                    }
                    rx_freq_ind = -1;
                    misDataCount = 0;
                    if ((uint16_t)DEVICE_ADDRESS == (uint16_t)GATEWAY_ADDRESS)
                    {
                        uint16_t addr = (Buffer[6] << 8) | Buffer[5];
                        if (addr > TOTAL_NODES)
                        {
                            if (SHOW_DEBUG_DETAIL)
                            {
                                printf("(not)%d%d\n", Buffer[6], Buffer[5]);
                            }
                        }
                        else
                        {
                            getDataNum[addr - 1] += 1;
                            printf("node:%d, cnt: %d\n", addr, getDataNum[addr - 1]);
                        }
                        checkAndSendPkts(true);
                    }
                    else
                    {
                        uint16_t addr = (Buffer[6] << 8) | Buffer[5];
                        if (addr > RELAY_NODES)
                        {
                            if (SHOW_DEBUG_DETAIL)
                            {
                                printf("(mot)%d%d\n", Buffer[6], Buffer[5]);
                            }
                        }
                        else
                        {
                            relayDataNum[addr - 1] += 1;
                            printf("relay: %d, cnt: %d\n", addr, relayDataNum[addr - 1]);
                            sendingACKAndWaitData = 0;
                            Buffer[7] = meshLoRaNxtAddr[(uint16_t)GATEWAY_ADDRESS] & 0xFF;
                            Buffer[8] = (meshLoRaNxtAddr[(uint16_t)GATEWAY_ADDRESS] >> 8) & 0xFF;
                            if ((bf_save_relay_have + 1) % BUFFER_SAVE_RELAY == bf_save_relay_now)
                            { //full
                                lost_relay += 1;
                                printf("lst relay: %d\n", lost_relay);
                            }
                            else
                            {
                                memset1(Buffer_save_relay[bf_save_relay_have], 0, BUFFER_SIZE);
                                memcpy1(Buffer_save_relay[bf_save_relay_have], Buffer, BufferSize);
                                Buffer_save_relay_len[bf_save_relay_have] = BufferSize;
                                bf_save_relay_have += 1;
                                bf_save_relay_have = bf_save_relay_have % BUFFER_SAVE_RELAY;
                            }
                            relayData();
                        }
                    }
                }
                else
                {
                    if (sendingACKAndWaitData == 1)
                    {
                        //must wait for data at least 2 times
                        if (misDataCount == 2)
                        {
                            misDataCount = 0;
                            sendingACKAndWaitData = 0;
                            rx_freq_ind = -1;
                        }
                        else
                        {
                            misDataCount += 1;
                            if (SHOW_DEBUG_DETAIL)
                            {
                                printf("%d, mis %d\n", BufferSize, misDataCount);
                            }
                            /*
								for(uint8_t i = 0; i < BufferSize; ++i){
										printf("%d ", Buffer[i]);
								}
								printf("\n");
								*/
                            if (rx_freq_ind == -1)
                            {
                                if (SHOW_FREQ_HOP)
                                {
                                    printf("freq_ind %d\n", -1);
                                }
                                Radio.SetChannel(RF_FREQUENCY);
                            }
                            else
                            {
                                if (SHOW_FREQ_HOP)
                                {
                                    printf("freq_ind %d\n", rx_freq_ind);
                                }
                                Radio.SetChannel(freq_hop[rx_freq_ind]);
                            }
                            Radio.Rx(WAITFORDATATIME);
                            //printf("(rd)%d\n", Radio.GetStatus());
                            State = LOWPOWER;
                            break;
                        }
                    }

                    if (SHOW_DEBUG_DETAIL)
                    {
                        printf("lther pkts %d\n", BufferSize);
                    }

                    if (dcState == MID_SLEEP)
                    {
                        //begin duty-cycle
                        dcState = SLEEP;
                        printf("(%d)sleep\n", RtcGetTimerValue());
                        TimerStop(&DutyCycleTimer);
                        TimerSetValue(&DutyCycleTimer, SLEEPTIME);
                        TimerStart(&DutyCycleTimer);
                        State = LOWPOWER;
                        break;
                    }

                    checkAndSendPkts(false);
                }
            }

            State = LOWPOWER;
            break;
        case TX:
            if (Ptype == RTS)
            {
                if (SHOW_DEBUG_DETAIL)
                {
                    printf("(%d)send RTS\n", RtcGetTimerValue());
                }
                if (rx_freq_ind == -1)
                {
                    if (SHOW_FREQ_HOP)
                    {
                        printf("freq_ind %d\n", -1);
                    }
                    Radio.SetChannel(RF_FREQUENCY);
                }
                else
                {
                    if (SHOW_FREQ_HOP)
                    {
                        printf("freq_ind %d\n", rx_freq_ind);
                    }
                    Radio.SetChannel(freq_hop[rx_freq_ind]);
                }
                Radio.Rx(RTSINTERVAL);
                //printf("(rd)%d\n", Radio.GetStatus());
            }
            else if (Ptype == ROUTER)
            {
                tx_freq_ind = -1;
                rx_freq_ind = -1;
                nodeSendRouterNum += 1;
                printf("send Router, len %d, cnt %d\n", BufferSize_send, nodeSendRouterNum);
                willSendRouter = false;
                getAck = false;
                if (!beginRouterTimer)
                {
                    beginRouterTimer = true;
                    TimerStart(&SendRouterTimer);
                }

                if (dcState == MID_SLEEP)
                {
                    //begin duty-cycle
                    dcState = SLEEP;
                    printf("(%d)sleep\n", RtcGetTimerValue());
                    TimerStop(&DutyCycleTimer);
                    TimerSetValue(&DutyCycleTimer, SLEEPTIME);
                    TimerStart(&DutyCycleTimer);
                    State = LOWPOWER;
                    break;
                }

                checkAndSendPkts(false);
            }
            else if (Ptype == ACK)
            {
                if (SHOW_DEBUG_DETAIL)
                {
                    printf("(%d)send ACK\n", RtcGetTimerValue());
                }
                if (sendingACKAndWaitData == 1)
                {
                    if (SHOW_DEBUG_DETAIL)
                    {
                        printf("rx %d\n", WAITFORDATATIME);
                    }
                    if (rx_freq_ind == -1)
                    {
                        if (SHOW_FREQ_HOP)
                        {
                            printf("freq_ind %d\n", -1);
                        }
                        Radio.SetChannel(RF_FREQUENCY);
                    }
                    else
                    {
                        if (SHOW_FREQ_HOP)
                        {
                            printf("freq_ind %d\n", rx_freq_ind);
                        }
                        Radio.SetChannel(freq_hop[rx_freq_ind]);
                    }
                    Radio.Rx(WAITFORDATATIME);
                    //printf("(rd)%d\n", Radio.GetStatus());
                }
                else
                {
                    printf("err3\n");
                }
            }
            else if (Ptype == DATA)
            {
                tx_freq_ind = -1;
                rx_freq_ind = -1;
                nodeSendDataNum += 1;
                printf("send Data, len %d, cnt %d\n", Buffer_save_data_len[bf_save_data_now], nodeSendDataNum);
                getAck = false;
                /*
					for(uint8_t i = 0; i < Buffer_save_data_len[bf_save_data_now]; ++i){
							printf("%d ", Buffer_save_data[bf_save_data_now][i]);
					}
					printf("\n");
					*/
                bf_save_data_now += 1;
                bf_save_data_now = bf_save_data_now % BUFFER_SAVE_DATA;

                if (dcState == MID_SLEEP)
                {
                    //begin duty-cycle
                    dcState = SLEEP;
                    printf("(%d)sleep\n", RtcGetTimerValue());
                    TimerStop(&DutyCycleTimer);
                    TimerSetValue(&DutyCycleTimer, SLEEPTIME);
                    TimerStart(&DutyCycleTimer);
                    State = LOWPOWER;
                    break;
                }

                checkAndSendPkts(false);
            }
            else if (Ptype == RELAY)
            {
                tx_freq_ind = -1;
                rx_freq_ind = -1;
                uint16_t addr = (Buffer_save_relay[bf_save_relay_now][6] << 8) | Buffer_save_relay[bf_save_relay_now][5];
                if (addr > RELAY_NODES)
                {
                    if (SHOW_DEBUG_DETAIL)
                    {
                        printf("(mot)%d%d\n", Buffer[6], Buffer[5]);
                    }
                }
                else
                {
                    relaySendDataNum[addr - 1] += 1;
                }
                printf("relay Data, len %d, cnt %d\n", Buffer_save_relay_len[bf_save_relay_now], relaySendDataNum[addr - 1]);
                getAck = false;

                bf_save_relay_now += 1;
                bf_save_relay_now = bf_save_relay_now % BUFFER_SAVE_RELAY;

                if (dcState == MID_SLEEP)
                {
                    //begin duty-cycle
                    dcState = SLEEP;
                    printf("(%d)sleep\n", RtcGetTimerValue());
                    TimerStop(&DutyCycleTimer);
                    TimerSetValue(&DutyCycleTimer, SLEEPTIME);
                    TimerStart(&DutyCycleTimer);
                    State = LOWPOWER;
                    break;
                }

                checkAndSendPkts(false);
            }

            State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            if (dcState == MID_SLEEP)
            {
                //begin duty-cycle
                dcState = SLEEP;
                printf("(%d)sleep\n", RtcGetTimerValue());
                TimerStop(&DutyCycleTimer);
                TimerSetValue(&DutyCycleTimer, SLEEPTIME);
                TimerStart(&DutyCycleTimer);
                State = LOWPOWER;
                break;
            }
            //back-off to relax
            if (SHOW_DEBUG_DETAIL)
            {
                printf("back-off\n");
            }
            TimerStop(&BackOffTimer);
            TimerSetValue(&BackOffTimer, BACKOFFTIME);
            TimerStart(&BackOffTimer);

            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
            rx_freq_ind = -1;
            if (dcState == MID_SLEEP)
            {
                //begin duty-cycle
                dcState = SLEEP;
                printf("(%d)sleep\n", RtcGetTimerValue());
                TimerStop(&DutyCycleTimer);
                TimerSetValue(&DutyCycleTimer, SLEEPTIME);
                TimerStart(&DutyCycleTimer);
                State = LOWPOWER;
                break;
            }

            if (Ptype == RTS)
            {
                MeshLoRaPrepareFrame(RTS);
                if (SHOW_TIMEONAIR)
                {
                    TimerTime_t TxTimeOnAir = Radio.TimeOnAir(MODEM_LORA, BufferSize_send);
                    printf("TAir %d %dms\n", BufferSize_send, TxTimeOnAir);
                }
                DelayMs(1);
                if (tx_freq_ind == -1)
                {
                    if (SHOW_FREQ_HOP)
                    {
                        printf("freq_ind %d\n", -1);
                    }
                    Radio.SetChannel(RF_FREQUENCY);
                }
                else
                {
                    if (SHOW_FREQ_HOP)
                    {
                        printf("freq_ind %d\n", tx_freq_ind);
                    }
                    Radio.SetChannel(freq_hop[tx_freq_ind]);
                }
                Radio.Send(Buffer_send, BufferSize_send);
            }
            else
            {
                checkAndSendPkts(false);
            }

            State = LOWPOWER;
            break;
        case RX_ERROR:
            rx_freq_ind = -1;
            if (dcState == MID_SLEEP)
            {
                //begin duty-cycle
                dcState = SLEEP;
                printf("(%d)sleep\n", RtcGetTimerValue());
                TimerStop(&DutyCycleTimer);
                TimerSetValue(&DutyCycleTimer, SLEEPTIME);
                TimerStart(&DutyCycleTimer);
            }
            else
            {
                if (Ptype == RTS)
                {
                    if (SHOW_DEBUG_DETAIL)
                    {
                        printf("rx %d\n", RTSINTERVAL);
                    }
                    if (Radio.GetStatus() == RF_IDLE)
                    {
                        if (rx_freq_ind == -1)
                        {
                            if (SHOW_FREQ_HOP)
                            {
                                printf("freq_ind %d\n", -1);
                            }
                            Radio.SetChannel(RF_FREQUENCY);
                        }
                        else
                        {
                            if (SHOW_FREQ_HOP)
                            {
                                printf("freq_ind %d\n", rx_freq_ind);
                            }
                            Radio.SetChannel(freq_hop[rx_freq_ind]);
                        }
                        Radio.Rx(RTSINTERVAL);
                        //printf("(rd)%d\n", Radio.GetStatus());
                    }
                    else
                    {
                        printf("stil3 %d\n", Radio.GetStatus());
                    }
                }
                else if (Ptype == ACK)
                {
                    if (SHOW_DEBUG_DETAIL)
                    {
                        printf("rx %d\n", WAITFORDATATIME);
                    }
                    if (Radio.GetStatus() == RF_IDLE)
                    {
                        if (rx_freq_ind == -1)
                        {
                            if (SHOW_FREQ_HOP)
                            {
                                printf("freq_ind %d\n", -1);
                            }
                            Radio.SetChannel(RF_FREQUENCY);
                        }
                        else
                        {
                            if (SHOW_FREQ_HOP)
                            {
                                printf("freq_ind %d\n", rx_freq_ind);
                            }
                            Radio.SetChannel(freq_hop[rx_freq_ind]);
                        }
                        Radio.Rx(WAITFORDATATIME);
                        //printf("(rd)%d\n", Radio.GetStatus());
                    }
                    else
                    {
                        printf("stil4 %d\n", Radio.GetStatus());
                    }
                }
                else if (Radio.GetStatus() == RF_IDLE)
                {
                    if (SHOW_DEBUG_DETAIL)
                    {
                        printf("rx3 0\n");
                    }
                    if (rx_freq_ind == -1)
                    {
                        if (SHOW_FREQ_HOP)
                        {
                            printf("freq_ind %d\n", -1);
                        }
                        Radio.SetChannel(RF_FREQUENCY);
                    }
                    else
                    {
                        if (SHOW_FREQ_HOP)
                        {
                            printf("freq_ind %d\n", rx_freq_ind);
                        }
                        Radio.SetChannel(freq_hop[rx_freq_ind]);
                    }
                    Radio.Rx(0);
                    rxNotimerOut = true;
                    //printf("(rd)%d\n", Radio.GetStatus());
                    if (dcState == MID_SLEEP)
                    {
                        //printf("stby\n");
                        Radio.Standby();
                        rxNotimerOut = false;
                        DelayMs(1);
                        Radio.Sleep();
                        dcState = SLEEP;
                        printf("(%d)sleep\n", RtcGetTimerValue());
                        //printf("(rd)%d\n", Radio.GetStatus());
                        TimerSetValue(&DutyCycleTimer, SLEEPTIME);
                        TimerStart(&DutyCycleTimer);
                    }
                }
                else
                {
                    printf("stil5 %d\n", Radio.GetStatus());
                }
            }

            State = LOWPOWER;
            break;
        case CAD:
            if (meshLoRaChannelActivityDetected == true)
            {
                meshLoRaChannelActivityDetected = false;
                cad_detect_time = 0;
                if (SHOW_DEBUG_DETAIL)
                {
                    printf("cad rx\n");
                }
                if (rx_freq_ind == -1)
                {
                    if (SHOW_FREQ_HOP)
                    {
                        printf("freq_ind %d\n", -1);
                    }
                    Radio.SetChannel(RF_FREQUENCY);
                }
                else
                {
                    if (SHOW_FREQ_HOP)
                    {
                        printf("freq_ind %d\n", rx_freq_ind);
                    }
                    Radio.SetChannel(freq_hop[rx_freq_ind]);
                }
                Radio.Rx(randr(56, CAD_BACKOFF_TIME));
            }
            else
            {
                cad_detect_time += 1;
                if (cad_detect_time > 1)
                {
                    cad_detect_time = 0;
                    Ptype = RTS;
                    MeshLoRaPrepareFrame(RTS);
                    if (SHOW_TIMEONAIR)
                    {
                        TimerTime_t TxTimeOnAir = Radio.TimeOnAir(MODEM_LORA, BufferSize_send);
                        printf("TAir %d %dms\n", BufferSize_send, TxTimeOnAir);
                    }
                    DelayMs(1);
                    if (tx_freq_ind == -1)
                    {
                        if (SHOW_FREQ_HOP)
                        {
                            printf("freq_ind %d\n", -1);
                        }
                        Radio.SetChannel(RF_FREQUENCY);
                    }
                    else
                    {
                        if (SHOW_FREQ_HOP)
                        {
                            printf("freq_ind %d\n", tx_freq_ind);
                        }
                        Radio.SetChannel(freq_hop[tx_freq_ind]);
                    }
                    Radio.Send(Buffer_send, BufferSize_send);
                }
                else
                {
                    TimerSetValue(&CADAgainTimer, CAD_AGAIN_TIME);
                    TimerStart(&CADAgainTimer);
                }
            }

            State = LOWPOWER;
            break;
        case LOWPOWER:
            break;
        default:
            break;
        }
    }
}
