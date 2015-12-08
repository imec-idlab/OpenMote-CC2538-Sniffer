#!/usr/bin/python

import serial

HDLC_FLAG        = 0x7E
HDLC_ESCAPE      = 0x7D
HDLC_ESCAPE_MASK = 0x20

def decode(msg):
    escaped = False
    string = ''
    for c in msg:
        if ord(c) == HDLC_ESCAPE:
            escaped = True
            continue
        else:
            if escaped:
                string += chr(ord(c) ^ HDLC_ESCAPE_MASK)
                escaped = False
            else:
                string += c

    return string


def main():
    ser = serial.Serial(port     = '/dev/ttyUSB0',
                        baudrate = 460800,
                        parity   = serial.PARITY_NONE,
                        stopbits = serial.STOPBITS_ONE,
                        bytesize = serial.EIGHTBITS,
                        xonxoff  = False,
                        rtscts   = False,
                        dsrdtr   = False)

    ser.setRTS(False)
    ser.setDTR(False)

    print("Listener started")

    receiving = False
    while(True):
        c = ser.read(1)
        if not receiving:
            if c == chr(HDLC_FLAG):
                receiving = True
                word = ''
            else:
                print('WARNING: byte dropped')
        else:
            if c == chr(HDLC_FLAG):
                if len(word) == 0:
                    print('WARNING: out of sync detected')
                else:
                    receiving = False
                    word = decode(word)
                    if len(word) > 2:
                        print(word[0:-2])
                    else:
                        print('ERROR: Message too short')
            else:
                word += c

if __name__ == "__main__":
    main()
