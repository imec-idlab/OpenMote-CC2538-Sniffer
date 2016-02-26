'''
@file       ieee802154-sniffer.py
@author     Pere Tuset-Peiro  (peretuset@openmote.com)
@version    v0.1
@date       May, 2015
@brief      

@copyright  Copyright 2015, OpenMote Technologies, S.L.
            This file is licensed under the GNU General Public License v2.
'''

#!/usr/bin/python

# Import Python libraries
import os
import sys
import time
import getopt
import logging
import logging.config

# Define path of the OpenMote libraries
library_path = os.path.abspath('../../python/library')
sys.path.append(library_path)

# Import OpenMote libraries
import Serial as Serial
import TunInterface as TunInterface

# Define logging configuration
logging.config.fileConfig("ieee802154-sniffer.cfg", disable_existing_loggers = False)
logger = logging.getLogger(__name__)

class Sniffer():
    default_channel    = 20
    cmd_change_channel = chr(0xCC)
    
    sniffer_type     = None
        
    serial_name      = None
    serial_baudrate  = None
    serial_interface = None

    tun_name         = None
    tun_interface    = None
    
    def __init__(self, sniffer_mode = None, serial_name = None, baud_rate = None, bsl_mode = None, tun_name = None):
        assert sniffer_mode != None, logger.error("Sniffer mode not defined.")
        assert serial_name  != None, logger.error("Serial port not defined.")
        assert bsl_mode     != None, logger.error("Bootloader mode not defined.")
        assert baud_rate    != None, logger.error("Serial baudrate not defined.")
        
        self.sniffer_mode = sniffer_mode
        self.serial_name  = serial_name
        self.bsl_mode     = bsl_mode
        self.baud_rate    = baud_rate
        
        if (self.sniffer_mode == "serial"):
            assert tun_name != None, logger.error("TUN interface not defined.")
            self.tun_name = tun_name    
           
    def run(self):
        stop = False
        
        # Create the Serial port
        logging.info("run: Creating the Serial port.")
        try:
            self.serial_port = Serial.Serial(serial_name = self.serial_name,
                                             baud_rate = self.baud_rate,
                                             bsl_mode = self.bsl_mode)
        except:                                   
            return

        # If using the serial port to sniff
        if (self.sniffer_mode == "serial"):
            # Create the TUN interface
            logging.info("run: Creating the TUN interface.")
            try:
                self.tun_interface = TunInterface.TunInterface(tun_name = self.tun_name)
            except:
                # Stop the serial port 
                self.serial_port.stop()
                return
            
        # Start the Serial port
        print("- Serial: Listening to port %s at %s bps." % (self.serial_name, self.baud_rate))
        self.serial_port.start()
        
        # Start the TUN interface
        print("- Tun:    Injecting packets to interface %s." % self.tun_name)
        self.tun_interface.start()
        
        # Define the IEEE 802.15.4 channel
        stop = self.set_radio_channel()
        
        # Run until stopped by user
        while (not stop):
            try:
                if (self.sniffer_mode == "serial"):
                    # Try to receive a packet from the Serial port
                    stop, packet, length = self.serial_port.receive()
                    
                    # If a packet is successfully received
                    if (packet):
                        logger.info("run: Received a message with %s bytes.", length)
                        # Inject the packet to the TUN interface
                        self.tun_interface.inject(packet)
                else:
                    time.sleep(0.5)

            except (KeyboardInterrupt):
                # Define the IEEE 802.15.4 channel
                stop = self.set_radio_channel()
            
        # Stop the serial port    
        self.serial_port.stop()
        
         # If using the serial port to sniff
        if (self.sniffer_mode == "serial"):
            # Stop the TUN interface
            self.tun_interface.stop()
                    
    def set_radio_channel(self):
        channel = -1
        while (channel != 0 and (channel < 11 or channel > 26)): 
            try:
                channel = int(raw_input("- Radio:  Select the IEEE 802.15.4 channel number (11-26, 0 = exit): "))
            except (KeyboardInterrupt):
                print
        
        if (channel == 0):
            return True
        else:
            logging.info("set_radio_channel: Setting the IEEE 802.15.4 radio channel to %d.", channel)
            print("- Radio:  Changing to IEEE 802.15.4 channel %d." % channel)
            output_message = ''.join([self.cmd_change_channel, chr(channel)])
            self.serial_port.transmit(str(output_message))
            return False              
                

def parse_config(config = None, arguments = None):
    assert config    != None, logger.error("Config not defined.")
    assert arguments != None, logger.error("Arguments not defined.")

    try:
        opts, args = getopt.getopt(arguments, "s:p:b:q:t:")
    except getopt.GetoptError as error:
        print(str(error))
        sys.exit(1)
    
    # Parse command line options
    for option, value in opts:
        if option == '-s':
            config['sniffer_mode'] = value
        elif option == '-p':
            config['serial_name'] = value
        elif option == '-b':
            config['baud_rate'] = value
        elif option == '-q':
            config['bsl_mode'] = value
        elif option == '-t':
            config['tun_name'] = value
        else:
            assert False, logger.error("Unhandled options while parsing the command line arguments.")
       
    return config
    
def main():
    default_config = {
        'sniffer_mode': 'serial',
        'serial_name' : '/dev/ttyUSB0',
        'baud_rate'   : '115200',
        'bsl_mode'    : 'true',
        'tun_name'    : 'tun0'
    }
    
    # Parse the command line arguments
    arguments = sys.argv[1:]
    config    = parse_config(default_config, arguments)
    
    # Create the sniffer based on user configuration
    sniffer = Sniffer(sniffer_mode = config['sniffer_mode'],
                      serial_name  = config['serial_name'],
                      baud_rate    = config['baud_rate'],
                      bsl_mode     = config['bsl_mode'],
                      tun_name     = config['tun_name'])
    
    # Execute the sniffer
    sniffer.run()
    
    # Finish the execution
    sys.exit(0)
    
if __name__ == "__main__":
    main()
