#!/bin/sh

# Check if user is root/running with sudo
if [ `whoami` != root ]; then
    echo Please run this script with sudo
    exit
fi

ORIG_PATH=`pwd`
cd `dirname $0`
SCRIPT_PATH=`pwd`
cd $ORIG_PATH

if [ "`uname -s`" != "Darwin" ]; then
    # Install UDEV rules for USB device
    cp ${SCRIPT_PATH}/56-orbbec-usb.rules /etc/udev/rules.d/56-orbbec-usb.rules 
    echo "usb rules file install at /etc/udev/rules.d/56-orbbec-usb.rules"
    cp ${SCRIPT_PATH}/50-fish_eye_webcam.rules /etc/udev/rules.d/50-fish_eye_webcam.rules
    echo "usb rules file install at /etc/udev/rules.d/50-fish_eye_webcam.rules"
    cp ${SCRIPT_PATH}/50-hokuyo.rules /etc/udev/rules.d/50-hokuyo.rules
    echo "usb rules file install at /etc/udev/rules.d/50-hokuyo.rules"
    cp ${SCRIPT_PATH}/50-rfid.rules /etc/udev/rules.d/50-rfid.rules
    echo "usb rules file install at /etc/udev/rules.d/50-rfid.rules"
fi
echo "exit"