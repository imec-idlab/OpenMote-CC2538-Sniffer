'''
@file       TunInterface.py
@author     Pere Tuset-Peiro  (peretuset@openmote.com)
@version    v0.1
@date       May, 2015
@brief      

@copyright  Copyright 2015, OpenMote Technologies, S.L.
            This file is licensed under the GNU General Public License v2.
'''

# Import Python libraries
import os
import fcntl
import struct
import subprocess
import logging

# Import logging configuration
logger = logging.getLogger(__name__)

class TunInterface():
    TUNSETIFF   = 0x400454ca
    TUNSETOWNER = TUNSETIFF + 2
    IFF_TUN     = 0x0001
    IFF_TAP     = 0x0002
    IFF_NO_PI   = 0x1000
    
    def __init__(self, tun_name = None):
        logger.info("init: Creating the TunInterface object.")
        
        # Save the TUN name
        self.tun_name = tun_name
        
        try:
            logger.info("init: Opening the TUN device file on /dev/net/tun.")
            
            # Open TUN device file
            self.tun_if = open('/dev/net/tun', 'r+b')

            # Tell it we want a TUN device
            ifr = struct.pack('16sH', self.tun_name, self.IFF_TAP | self.IFF_NO_PI)

            # Create TUN interface
            fcntl.ioctl(self.tun_if, self.TUNSETIFF, ifr)

            # Make the TUN interface be accessed by regular users
            fcntl.ioctl(self.tun_if, self.TUNSETOWNER, 1000)
        except:
            logger.error('init: Error while creating the TUN device file.')
            raise Exception
        
        logger.info("init: TunInterface object created.")
    
    def start(self):
        logger.info("start: Starting the TUN interface.")
        
        # Bring TUN interface up
        subprocess.check_call(['ifconfig', self.tun_name, 'up'])
    
    def stop(self):
        logger.info("stop: Stopping the TUN interface.")
        
        # Bring TUN interface down
        subprocess.check_call(['ifconfig', self.tun_name, 'down'])
    
    def inject(self, packet):
        logger.info("inject: Injecting a packet to the TUN interface.")
        
        # Write a packet to the TUN interface
        os.write(self.tun_if.fileno(), ''.join(packet))
