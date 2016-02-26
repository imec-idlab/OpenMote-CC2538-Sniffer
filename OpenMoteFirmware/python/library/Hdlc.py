'''
@file       Hdlc.py
@author     Pere Tuset-Peiro  (peretuset@openmote.com)
@version    v0.1
@date       May, 2015
@brief      

@copyright  Copyright 2015, OpenMote Technologies, S.L.
            This file is licensed under the GNU General Public License v2.
'''

# Import Python libraries
import logging

# Import OpenMote libraries
import Crc16 as Crc16

# Import logging configuration
logger = logging.getLogger(__name__)

class Hdlc(object):
    HDLC_FLAG           = '\x7E'
    HDLC_FLAG_ESCAPED   = '\x5E'
    HDLC_ESCAPE         = '\x7D'
    HDLC_ESCAPE_ESCAPED = '\x5D'
    
    def __init__(self):
        pass
    
    # Converts a buffer into an HDLC frame
    def hdlcify(self, input = None):
        # Make a copy of the original input
        output = ''.join(input[:])
        
        # Check the CRC checksum
        crc_engine = Crc16.Crc16()
        for o in output:
            crc_engine.push(o)
        crc_result = crc_engine.get()
        
        # Append the CRC checksum
        output += chr((crc_result >> 8) & 0xFF)
        output += chr((crc_result >> 0) & 0xFF)
        
        # Substitute the HDLC flags
        output = output.replace(self.HDLC_ESCAPE, self.HDLC_ESCAPE + self.HDLC_ESCAPE_ESCAPED)
        output = output.replace(self.HDLC_FLAG, self.HDLC_ESCAPE + self.HDLC_FLAG_ESCAPED)
        
        # Append the HDLC flags
        output = self.HDLC_FLAG + output + self.HDLC_FLAG
        
        return output
    
    # Parses an HDLC frame
    # Returns the extracted frame or -1 if wrong CRC checksum
    def dehdlcify(self, input = None):
        # Check if the input contains HDLC_FLAG
        assert input[0] == self.HDLC_FLAG
        assert input[-1] == self.HDLC_FLAG
        
        # Make a copy of the original input
        output = input[1:-1]
        
        # Replace the HDLC flags
        output = output.replace(self.HDLC_ESCAPE + self.HDLC_FLAG_ESCAPED, self.HDLC_FLAG)
        output = output.replace(self.HDLC_ESCAPE + self.HDLC_ESCAPE_ESCAPED, self.HDLC_ESCAPE)
        
        # Check output input size
        if (len(output) < 2):
            logging.error("dehldicfy: Invalid frame length!")
        
        # Get the CRC checksum
        crc_value = int(output[-2:].encode('hex'), 16)
        
        # Compute the CRC checksum
        crc_engine = Crc16.Crc16()
        for o in output[:-2]:
            crc_engine.push(o)
        crc_result = crc_engine.get()
        
        # Check CRC checksum
        if (crc_value != crc_result):
            logging.error("dehldicfy: CRC value does not match!")
        
        # Remove the CRC checksum
        output = output[:-2]
        
        return output
    
