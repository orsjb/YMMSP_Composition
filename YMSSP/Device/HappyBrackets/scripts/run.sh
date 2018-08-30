#!/bin/bash

### Script to autorun HappyBrackets on device

mkdir -p /home/pi/HappyBrackets/ramfs;
sudo mount -t ramfs -o size=512 ramfs /home/pi/HappyBrackets/ramfs

### move to the correct dir for running this script (one level above where this script is)
DIR=`dirname $0`
cd ${DIR}/..


#if we want to set specific parameters for a device, place them in device.config
CONFIG_FILE=scripts/$HOSTNAME.config

### run HappyBrackets
### args to HB.jar are: buf (buffer size, default=1024), sr (sample rate, default=44100), bits (sample bit size, default=16), ins (input channels, default=0), outs (output channels, default=1), start (autostart audio, default=true), access (live code access mode, either ‘open’, ‘local’ or ‘closed’, default=open), followed by the full class path to any HBAction you wish to auto run. All args except the last one can be entered in any order.

#define default values

BUF=1024
SR=44100
BITS=16
INS=0
OUTS=1 
DEVICE=0
AUTOSTART=true 
ACCESSMODE=open
ACTION=
CONFIG=device-config.json

#let us see if we have any specific values we want to use
while IFS="=" read line val
do
    if [ "$line" = "BUF"   ]
    then
        BUF=$val
        echo "Set BUF to "$BUF
    fi

    if [ "$line" = "SR"   ]
    then
        SR=$val
        echo "Set SR to "$SR
    fi

    if [ "$line" = "BITS"   ]
    then
        BITS=$val
        echo "Set Bits to "$BITS
    fi

    if [ "$line" = "INS"   ]
    then
        INS=$val
        echo "Set INS to "$INS
    fi


    if [ "$line" = "OUTS"   ]
    then
        OUTS=$val
        echo "Set OUTS to "$OUTS
    fi

    if [ "$line" = "DEVICE"   ]
    then
        DEVICE=$val
        echo "Set DEVICE to "$DEVICE
    fi

    if [ "$line" = "AUTOSTART"   ]
    then
        AUTOSTART=$val
        echo "Set AUTOSTART to "$AUTOSTART
    fi

    if [ "$line" = "ACCESSMODE"   ]
    then
        ACCESSMODE=$val
        echo "Set ACCESSMODE to "$ACCESSMODE
    fi

    if [ "$line" = "CONFIG"   ]
    then
        CONFIG=$val
        echo "Set CONFIG to "$CONFIG
    fi

done <$CONFIG_FILE


### run the auto-rename script
scripts/auto-rename.sh

echo “Running HappyBrackets”

(/usr/bin/sudo /usr/bin/java -cp "data/classes:HB.jar:data/jars/*" -Xmx512m net.happybrackets.device.DeviceMain buf=$BUF sr=$SR bits=$BITS ins=$INS outs=$OUTS device=$DEVICE start=$AUTOSTART access=$ACCESSMODE $ACTION config=$CONFIG > ramfs/stdout 2>&1) &

### Finally, run the network-monitor.sh script to keep WiFi connection alive
# Ignore this for stretch (scripts/network-monitor.sh > netstatus &) &
