'''
@file       Utils.py
@author     Pere Tuset-Peiro  (peretuset@openmote.com)
@version    v0.1
@date       May, 2015
@brief      

@copyright  Copyright 2015, OpenMote Technologies, S.L.
            This file is licensed under the GNU General Public License v2.
'''

# Import Python libraries
import sys
import glob

PLATFORM_WINDOWS = 'WINDOWS'
PLATFORM_MAC     = 'MACINTOSH'
PLATFORM_LINUX   = 'LINUX'

# Enumerates all the serial ports in the computer
def find_serial_ports():
    # Enumerate the serial ports
    serial_ports = None
    while (not serial_ports):
        serial_ports = enumerate_serial_ports()
        if (not serial_ports):
            out = raw_input("No serial port found! Retry (y/n)? ")
            if (out == 'N' or out == 'n'):
                return None
            else:
                continue
    return sorted(serial_ports)

# Returns the serial port selected by the user
def select_serial_port(serial_ports = None):
    if (not serial_ports):
        print("No serial ports found!")
        return None
    
    serial_ports.append("Exit")
    while (True):
        try:
            print("Please, select your serial port from the list: ")
            r = range(len(serial_ports))
            for i in r:
                print "- [" + str(i) + "] " + str(serial_ports[i])
            n = int(input("Choose your serial port: "))
        except:
            print("Input parameter is not a number!")
            continue
        else:
            if (n not in r):
                print("Input number is outside bounds!")
                continue
            if (n == len(serial_ports) - 1):
                return None
            else:
                return serial_ports[n]

def enumerate_serial_ports():
    serial_ports = []
    
    # Find the current operating system
    platform = find_operating_system()
    
    # Find serial ports based on current operating system
    if (platform == PLATFORM_WINDOWS):
        import _winreg as winreg
        path = 'HARDWARE\\DEVICEMAP\\SERIALCOMM'
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path)
        for i in range(winreg.QueryInfoKey(key)[1]):
            try:
                val = winreg.EnumValue(key,i)
            except:
                pass
            else:
                if   val[0].find('VCP') > -1:
                    serial_ports.append(str(val[1]))
    elif (platform == PLATFORM_MAC):
        serial_ports = [s for s in glob.glob('/dev/tty.usbserial*')]
    elif(platform == PLATFORM_LINUX):
        serial_ports = [s for s in glob.glob('/dev/ttyUSB*')]
                         
    return serial_ports

def find_operating_system():
    import platform
    platform = platform.system()
    
    if (platform == 'Windows'):
        return PLATFORM_WINDOWS
    elif (platform == 'Darwin'):
        return PLATFORM_MAC
    elif (platform == 'Linux'):
        return PLATFORM_LINUX
    else:
        error = "Error: Unsuported platform"
        print error
