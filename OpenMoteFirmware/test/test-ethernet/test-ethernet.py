#!/usr/bin/python

'''
@file       test-ethernet.py
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
from scapy.all import Ether,IP,UDP,Raw
from scapy.all import sendp, sniff
from scapy.arch import get_if_hwaddr

eth_iface = "eth0"
eth_addr  = get_if_hwaddr(eth_iface)
eth_delay = 0.5

sniff_addr = "ff:ff:ff:ff:ff:ff"
payload  = "0123456789ABCDEF123456789ABCDEF"

def program():
    global sniff_addr
    
    print("Testing from " + eth_iface + " (" + eth_addr + ")...")

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
        # Sniff a packet with a MAC address filter
        print("Sniffing for: " + sniff_addr)
        filt = "ether dst " + sniff_addr + " or ether src " + sniff_addr
        packet = sniff(iface=eth_iface, count=1, filter=filt)

        # Get source and destination addresses
        dst_addr = packet[0][Ether].dst
        src_addr = packet[0][Ether].src
        sniff_addr = src_addr
        
        print("Got a packet from: " + src_addr + " to: " + dst_addr + "!")
        
        # Create a packet to transmit
        packet[0][Ether].dst = src_addr
        packet[0][Ether].src = eth_addr
        
        # Send a packet
        sendp(packet, iface=eth_iface, verbose=False)
        
        print("Sent a packet from: " + packet[0][Ether].src + " to: " + packet[0][Ether].dst + "!")

def main():
    program()
    
if __name__ == "__main__":
    main()
