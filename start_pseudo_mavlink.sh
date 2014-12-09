#! /usr/bin/env sh

# -------------------------------------------------
#   Setup
# -------------------------------------------------
#
# Run this once
# $ sudo apt-get socat
#   (utility to create virtual serial ports)
# $ sudo chown <user> /dev  
#   (gives you permissions to create new virtual serial ports)


socat -d -d pty,raw,echo=0,link=/dev/ttyVA00 pty,raw,echo=0,link=/dev/ttyUSB0 &
sleep 1
./pseudo_mavlink &
jobs
sleep 9999999999


kill %1 %2
