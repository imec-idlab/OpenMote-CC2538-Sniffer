#!/usr/bin/python

'''
@file       Crc16.py
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
import time

test_board_src = "../../../../OpenMote-Firmware/projects/test-board/src"

def program(): 
    print("Programming the OpenMote-CC2538...")
    
    os.chdir(test_board_src)
    # os.system("make TARGET=cc2538 clean")
    os.system("make TARGET=cc2538 all")
    os.system("make TARGET=cc2538 bsl")

    print("Checking the RSSI...")

    ser = serial.Serial(port     = '/dev/ttyUSB0',
                        baudrate = 115200,
                        bytesize = serial.SEVENBITS,
                        parity   = serial.PARITY_NONE,
                        stopbits = serial.STOPBITS_ONE,
                        xonxoff  = 0,
                        rtscts   = 0,
                        dsrdtr   = 0,
                        timeout  = 2.5)
    
    ser.setDTR(0) # Disable reset
    ser.setDTR(1) # Enable reset
    ser.setRTS(0) # Enable bootloader
    ser.setDTR(0) # Disable reset
    
    rssi(ser)

def rssi(ser = None):  
    iteracions = 25
    mitjana = 0.0
    
    i = iteracions
    while(i != 0):
        c = ser.read(1);
        rssi = struct.unpack("b", c)[0] - 128
        if (rssi > -128 and rssi < 10):
            print("El valor de la RSSI es: " + str(rssi))
            mitjana += float(rssi)
            i = i-1
        else:
            print("Error, el valor de la RSSI es invalid ( " + str(rssi) + ")")
    
    print("El valor promig de la RSSI es: " + str(mitjana/iteracions))

def main():
    program()
    
if __name__ == "__main__":
    main()
