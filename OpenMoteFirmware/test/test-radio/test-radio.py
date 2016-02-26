#!/usr/bin/python

'''
@file       test-radio.py
@author     Pere Tuset-Peiro  (peretuset@openmote.com)
@version    v0.1
@date       May, 2015
@brief      

@copyright  Copyright 2015, OpenMote Technologies, S.L.
            This file is licensed under the GNU General Public License v2.
'''

import serial
import struct
import os

def program():
    print("Checking the RSSI...")

    ser = serial.Serial(port     = '/dev/ttyUSB0',
                        baudrate = 115200,
                        parity   = serial.PARITY_ODD,
                        stopbits = serial.STOPBITS_TWO,
                        bytesize = serial.SEVENBITS,
                        xonxoff  = False,
                        rtscts   = False,
                        dsrdtr   = False)
                        
    ser.setRTS(False) # Disable bootloader
    ser.setDTR(False) # Disable reboot

    while(True):
        buff = ser.read(8);
        data = struct.unpack("8b", buff)
        
        address = data[0:5]
        rssi    = data[6] - 128
        crc     = data[7]

def main():
    program()
    
if __name__ == "__main__":
    main()
