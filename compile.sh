#!/bin/bash

APP=LoRaMeasurement
#APP=BeaconAndSend
# APP=LoRaMac
# APP=ping-pong
cmake -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-none-eabi.cmake -DAPPLICATION="$APP" -DCLASS="classA" -DBOARD=Handsome -DACTIVE_REGION=LORAMAC_REGION_CN470 -DREGION_CN470=ON -DREGION_EU868=OFF -DNNN=1 ..
