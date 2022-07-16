#! /bin/bash
idf.py build
idf.py -p /dev/cu.usbserial-2220 -b 460800 flash
# idf.py -p /dev/cu.usbserial-2220 -b 115200 monitor
