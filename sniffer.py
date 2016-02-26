#!/usr/bin/env python

import serial
import struct
import os
import sys
import glob
import subprocess
import time
import platform
import array
import signal
import subprocess
import threading
import argparse

platform = platform.system()
if platform != 'Windows' and platform != 'Linux' and platform != 'Darwin':
    raise RuntimeError('ERROR: Sniffer does not support platform "' + platform + '"')

if platform == 'Windows':
    from win32process import DETACHED_PROCESS

if (sys.version_info > (3, 0)):
    INPUT = input
    if platform == 'Windows':
        import winreg
else:
    INPUT = raw_input
    if platform == 'Windows':
        import _winreg as winreg

stopSniffingThread = False
ser = serial.Serial()
output = None
outputIsFile = True
actualSnifferTerminated = False

HDLC_FLAG        = 0x7E
HDLC_ESCAPE      = 0x7D
HDLC_ESCAPE_MASK = 0x20

precompiled_crc16_table = [
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
    0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
    0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
    0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
    0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
    0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
    0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
    0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
    0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
    0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
    0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
    0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
    0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
    0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
    0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
    0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
    0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
    0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
    0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
    0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
    0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
    0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78
]

def pickSerialPort():
    ports = []
    if platform == 'Darwin':
        ports = [port for port in glob.glob('/dev/tty.usbserial*')]
    elif platform == 'Linux':
        ports = [port for port in glob.glob('/dev/ttyUSB*')]
    else:
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, 'HARDWARE\\DEVICEMAP\\SERIALCOMM')
        for i in range(winreg.QueryInfoKey(key)[1]):
            try:
                val = winreg.EnumValue(key,i)
                if val[0].find('VCP') > -1:
                    ports.append(str(val[1]))
            except:
                pass

    if len(ports) == 0:
        print("No serial ports found!")
        return None
    elif len(ports) == 1:
        print("Using serial port '" + str(ports[0]) + "'")
        return ports[0]
    else:
        ports = sorted(ports)
        while (True):
            try:
                print("Multiple serial ports were found:")
                for i in range(len(ports)):
                    print("- [" + str(i) + "] " + str(ports[i]))
                selectedPort = int(INPUT("Choose your serial port: "))
                if selectedPort < 0 or selectedPort >= len(ports):
                    print("Input number is outside bounds!")
                    continue
                else:
                    return ports[selectedPort]
            except KeyboardInterrupt:
                return None
            except:
                print("Input parameter is not a number!")
                continue


def pickRadioChannel():
    channel = 1
    while (channel < 11 or channel > 26) and channel != 0:
        try:
            channel = int(INPUT("Select the IEEE 802.15.4 channel number (11-26, 0 to exit): "))
        except KeyboardInterrupt:
            print('')
            return None
        except:
            pass

    if channel == 0:
        return None
    else:
        print('Setting radio channel to ' + str(channel))
        return channel


def createPipe(name, force):
    if platform != 'Windows':
        if os.path.exists(name):
            if not force:
                response = str(INPUT('File ' + name + ' already exists. Delete it and continue? [y/N] '))
                if response == 'y' or response == 'Y':
                    os.remove(name)
                else:
                    sys.exit()
            else:
                os.remove(name)

        os.mkfifo(name)
    else:
        global output
        output = win32pipe.CreateNamedPipe("\\\\.\\pipe\\" + name,
                                           win32pipe.PIPE_ACCESS_OUTBOUND,
                                           win32pipe.PIPE_TYPE_MESSAGE | win32pipe.PIPE_WAIT,
                                           1,
                                           65536,
                                           65536,
                                           300,
                                           None)

def removePipe(name):
    if platform != 'Windows':
        os.remove(name)
    else:
        output.close()


def calcRadioCRC(string):
    crc = 0x0000
    for val in string:
        crc = precompiled_crc16_table[(crc ^ val) & 0xff] ^ (crc >> 8)
    return crc
    crc ^= 0xFFFF
    return crc ^ 0xffff;


def calcCRC(string):
    crc = 0xFFFF
    for c in string:
        crc = precompiled_crc16_table[ord(c) ^ ((crc >> 8) & 0xFF)] ^ ((crc << 8) & 0xFFFF)
    return crc;


def decode(msg, quiet=False):
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

    if len(string) < 6:
        if not quiet:
            print("WARNING: Received string too short")
        return ('', 0)

    if calcCRC(string[:-2]) != ((ord(string[-2]) << 8) + ord(string[-1])):
        if not quiet:
            print("WARNING: CRC check failed!")
        return ('', 0)

    return (string[:-2], (ord(string[-2]) << 8) + ord(string[-1]))


def encode(string):
    crc = calcCRC(string)

    byteArray = bytearray()
    byteArray.append(HDLC_FLAG)
    for c in string:
        if ord(c) == HDLC_FLAG or ord(c) == HDLC_ESCAPE:
            byteArray.append(HDLC_ESCAPE)
            byteArray.append(ord(c) ^ HDLC_ESCAPE_MASK)
        else:
            byteArray.append(ord(c))

    crcByte = (crc >> 8) & 0xFF
    if crcByte == HDLC_FLAG or crcByte == HDLC_ESCAPE:
        byteArray.append(HDLC_ESCAPE)
        byteArray.append(crcByte ^ HDLC_ESCAPE_MASK)
    else:
        byteArray.append(crcByte)

    crcByte = (crc >> 0) & 0xFF
    if crcByte == HDLC_FLAG or crcByte == HDLC_ESCAPE:
        byteArray.append(HDLC_ESCAPE)
        byteArray.append(crcByte ^ HDLC_ESCAPE_MASK)
    else:
        byteArray.append(crcByte)

    byteArray.append(HDLC_FLAG)
    return byteArray


def connectToOpenMote(channel, frameFilteringLevel):
    ser.flushInput()
    ser.flushOutput()

    if frameFilteringLevel == 3:
        enableFrameFiltering = True
    else:
        enableFrameFiltering = False

    # Keep sending RESET packet and discard all bytes until the READY packet arrives
    connected = False
    receiving = False
    for i in range(3):
        print('Connecting to OpenMote...')
        ser.write(encode('RST' + chr(channel) + chr(enableFrameFiltering + ord('0'))))
        begin = time.time()
        while time.time() - begin < 1:
            c = ser.read(1)
            if len(c) > 0:
                c = bytearray(c)[0]
                if not receiving:
                    if c == HDLC_FLAG:
                        receiving = True
                        word = ''
                else:
                    if c == HDLC_FLAG:
                        if len(word) != 0:
                            receiving = False
                            word = decode(word, quiet=True)[0]
                            if len(word) > 0:
                                if word == 'READY':
                                    connected = True
                                    break
                                elif len(word) == 9 and word[4:] == 'READY':
                                    connectToOpenMote(channel, frameFilteringLevel)
                                    return
                    else:
                        word += chr(c)
        if connected:
            break
    else:
        print('ERROR: Failed to connect to OpenMote')
        sys.exit(1)

    print('Connected to OpenMote')


def actual_sniffer(channel, frameFilteringLevel, replaceFCS):
    global actualSnifferTerminated

    word = ''
    count = 0
    totalCount = 0
    unackedByteCount = 0
    lastIndex = 0
    lastSeqNr = 0
    expectedSeqNr = 1
    receiving = False
    faultyPacketIgnored = False

    try:
        while not stopSniffingThread:
            c = ser.read(1)
            if len(c) > 0:
                c = bytearray(c)[0]
                if not receiving:
                    if c == HDLC_FLAG:
                        receiving = True
                        word = ''
                    else:
                        print('WARNING: byte dropped')
                else:
                    if c == HDLC_FLAG:
                        if len(word) == 0:
                            print('WARNING: out of sync detected')
                        else:
                            # Get the timestamp
                            ts_sec = int(time.time())
                            ts_usec = int((time.time() - ts_sec) * 1000000)

                            receiving = False
                            word = decode(word)[0]
                            if len(word) > 0:
                                # Something is wrong when sequence number is 0
                                receivedSeqNr = (ord(word[2]) << 8) + ord(word[3])
                                if receivedSeqNr == 0:
                                    if len(word) == 9 and word[4:] == 'READY':
                                        print('Sniffer reset detected, restarting')
                                    else:
                                        print('ERROR: invalid sequence number occured, restarting')

                                    connectToOpenMote(channel, frameFilteringLevel, replaceFCS)
                                    actual_sniffer(channel)
                                    return

                                # Ignore the packet if it had a wrong sequence number
                                if expectedSeqNr == (ord(word[2]) << 8) + ord(word[3]):
                                    expectedSeqNr += 1
                                    if expectedSeqNr == 65536:
                                        expectedSeqNr = 1

                                    # Remember the index and sequence number of this packet in case the next one is corrupted
                                    lastIndex = (ord(word[0]) << 8) + ord(word[1])
                                    lastSeqNr = (ord(word[2]) << 8) + ord(word[3])

                                    # When requested, only accept packets with a valid CRC
                                    if frameFilteringLevel != 1 or ord(word[-1]) & 128 == 1:

                                        # TODO: Does LQI value has to be adapted?

                                        # Convert the string to a bytearray to write it to the pipe
                                        packet = bytearray()
                                        for character in word[4:-2]:
                                            packet.append(ord(character))

                                        # Recalculate CRC if it was correct and requested
                                        if replaceFCS or ord(word[-1]) & 128 == 0:
                                            packet.append(ord(word[-2]))
                                            packet.append(ord(word[-1]))
                                        else:
                                            crc = calcRadioCRC(packet)
                                            packet.append(crc & 0xff)
                                            packet.append(crc >> 8)

                                        # Write Record Header and the packet to pipe
                                        header = bytearray()
                                        header.append((ts_sec >> 24) & 0xff) # timestamp seconds
                                        header.append((ts_sec >> 16) & 0xff) # timestamp seconds
                                        header.append((ts_sec >> 8) & 0xff)  # timestamp seconds
                                        header.append((ts_sec >> 0) & 0xff)  # timestamp seconds
                                        header.append((ts_usec >> 24) & 0xff) # timestamp microseconds
                                        header.append((ts_usec >> 16) & 0xff) # timestamp microseconds
                                        header.append((ts_usec >> 8) & 0xff)  # timestamp microseconds
                                        header.append((ts_usec >> 0) & 0xff)  # timestamp microseconds
                                        header.extend([0, 0, len(packet) >> 8, len(packet) & 0xff]) # nr of octets of packet saved
                                        header.extend([0, 0, len(packet) >> 8, len(packet) & 0xff]) # actual length of packet
                                        if outputIsFile:
                                            output.write(header)
                                            output.write(packet)
                                        else:
                                            win32file.WriteFile(output, header)
                                            win32file.WriteFile(output, packet)
                                else:
                                    if receivedSeqNr > expectedSeqNr:
                                        print('WARNING: Received seqNr=' + str(receivedSeqNr)
                                            + ' higher than expecting seqNr=' + str(expectedSeqNr))

                                # Send an ACK after enough bytes have been received
                                unackedByteCount += len(word)
                                if unackedByteCount >= 250 and lastSeqNr != 0:
                                    unackedByteCount = 0
                                    ser.write(encode('ACK' + chr((lastIndex >> 8) & 0xff) + chr(lastIndex & 0xff)
                                                           + chr((lastSeqNr >> 8) & 0xff) + chr(lastSeqNr & 0xff)))
                            else:
                                print('NACK ' + str((lastIndex >> 8) & 0xff) + ' ' + str(lastIndex & 0xff) + ' '
                                              + str((lastSeqNr >> 8) & 0xff) + ' ' + str(lastSeqNr & 0xff))
                                ser.write(encode('NACK' + chr((lastIndex >> 8) & 0xff) + chr(lastIndex & 0xff)
                                                        + chr((lastSeqNr >> 8) & 0xff) + chr(lastSeqNr & 0xff)))
                    else: # Not the closing byte
                        word += chr(c)

            else: # No character was read
                if receiving:
                    receiving = False
                    print('WARNING: expected another byte, assuming out of sync')
                    print('NACK ' + str((lastIndex >> 8) & 0xff) + ' ' + str(lastIndex & 0xff) + ' '
                                  + str((lastSeqNr >> 8) & 0xff) + ' ' + str(lastSeqNr & 0xff))
                    ser.write(encode('NACK' + chr((lastIndex >> 8) & 0xff) + chr(lastIndex & 0xff)
                                            + chr((lastSeqNr >> 8) & 0xff) + chr(lastSeqNr & 0xff)))
    except BrokenPipeError:
        actualSnifferTerminated = True
        print('ERROR: Pipe to wireshark is broken')
    except serial.serialutil.SerialException as e:
        actualSnifferTerminated = True
        print('ERROR: Serial error, assuming OpenMote disconnected')


def main():
    global ser
    global output
    global outputIsFile
    global stopSniffingThread
    global actualSnifferTerminated

    parser = argparse.ArgumentParser(description='IEEE 802.15.4 Sniffer')
    parser.add_argument('-c', '--channel', dest='channel', type=int,
                        help='Channel on which the sniffer should start listening (11-26)')
    parser.add_argument('-p', '--port', dest='port',
                        help='Full name of serial port to use. On linux this is e.g. "/dev/ttyUSB0"')
    parser.add_argument('-w', '--wireshark', dest='wireshark_executable', default='wireshark',
                        help='Filename of the wireshark executable (if not just "wireshark" or capturing to file)')
    parser.add_argument('-t', '--pipe', dest='pipe_name', default='fifopipe',
                        help='Name of the temporary pipe to wireshark (when not capturing to file)')
    parser.add_argument('-o', '--output', dest='pcap_file',
                        help='Filename of .pcap file to output. The existence of this parameter decides whether real-time capturing with wireshark is used or whether the sniffer just outputs to a pcap file.')
    parser.add_argument('-f', '--force', action='store_true',
                        help='Overwrite pcap file or pipe when it already exists')
    parser.add_argument('--replace-fcs', action='store_true',
                        help='Replace the normal radio FCS by the TI CC24XX FCS which contains the RSSI and LQI.')
    parser.add_argument('--discard-bad-crc', action='store_true',
                        help='Discard packets with a wrong CRC')
    parser.add_argument('--enable-frame-filtering', action='store_true',
                        help='Discard packets that do not satisfy the third-level filtering requirements as specified in the IEEE 802.15.4 standard. To only apply first level filtering, use the --discard-bad-crc option')
    args = parser.parse_args()

    wiresharkProcess = None
    frameFilteringLevel = 0
    if args.discard_bad_crc:
        frameFilteringLevel = 1
    if args.enable_frame_filtering:
        frameFilteringLevel = 3

    if args.channel != None and (args.channel < 11 or args.channel > 26):
        print('Channel should be between 11 and 26')
        sys.exit(2)

    # If no serial port was provided as parameter, find one now
    if args.port == None:
        args.port = pickSerialPort()
        if args.port == None:
            return

    # Setup the serial connection
    ser = serial.Serial(port     = args.port,
                        baudrate = 460800,
                        parity   = serial.PARITY_NONE,
                        stopbits = serial.STOPBITS_ONE,
                        bytesize = serial.EIGHTBITS,
                        xonxoff  = False,
                        rtscts   = False,
                        dsrdtr   = False,
                        timeout   = 0.25)

    # If no channel was provided as parameter, ask the user on which channel to listen
    if args.channel == None:
        args.channel = pickRadioChannel()
        if args.channel == None:
            sys.exit()

    # Start wireshark when needed
    if args.pcap_file == None:
        print('Creating pipe...')
        createPipe(args.pipe_name, args.force)

        print('Starting wireshark...')
        try:
            nullOutput = open(os.devnull, 'w')
            while True:
                try:
                    if platform != 'Windows':
                        wiresharkProcess = subprocess.Popen([args.wireshark_executable, '-k', '-i', args.pipe_name],
                                                            stdout=nullOutput,
                                                            stderr=nullOutput,
                                                            preexec_fn=os.setsid)
                    else:
                        wiresharkProcess = subprocess.Popen([args.wireshark_executable, '-k', '-i', "\\\\.\\pipe\\" + args.pipe_name],
                                                            stdout=nullOutput,
                                                            stderr=nullOutput,
                                                            creationflags=DETACHED_PROCESS)
                    break
                except FileNotFoundError:
                    print('ERROR: Failed to execute "' + args.wireshark_executable + '"')
                    args.wireshark_executable = str(INPUT('Please provide wireshark executable name: '))
        except KeyboardInterrupt:
            removePipe(args.pipe_name)
            sys.exit()

        print('Waiting for wireshark to be ready...')
        if platform != 'Windows':
            output = open(args.pipe_name, 'wb', buffering=0)
            outputIsFile = True
        else:
            win32pipe.ConnectNamedPipe(output, None)
            outputIsFile = False
        print('Connected to wireshark')

    else: # Write data to pcap file instead of real-time monitoring with wireshark
        if os.path.exists(args.pcap_file):
            if not args.force:
                response = str(INPUT('File ' + args.pcap_file + ' already exists. Delete it and continue? [y/N] '))
                if response == 'y' or response == 'Y':
                    os.remove(args.pcap_file)
                else:
                    sys.exit()
            else:
                os.remove(args.pcap_file)

        output = open(args.pcap_file, 'wb')
        outputIsFile = True

    # Write the global header to the output
    header = bytearray()
    header.extend([0xa1, 0xb2, 0xc3, 0xd4])  # magic number
    header.extend([0, 2]) # major version number
    header.extend([0, 4]) # minor version number
    header.extend([0]*4)  # GMT to local correction
    header.extend([0]*4)  # accuracy of timestamps
    header.extend([0, 0, 0xff, 0xff])  # max length of captured packets, in octets
    header.extend([0, 0, 0, 195])  # 802.15.4 protocol
    if outputIsFile:
        output.write(header)
    else:
        win32file.WriteFile(output, header)

    def cleanup():
        try:
            ser.write(encode('STOP'))
        except:
            pass

        if outputIsFile:
            output.close()
        else:
            win32pipe.DisconnectNamedPipe(pipe)

        if args.pcap_file == None:
            removePipe(args.pipe_name)

    try:
        while True:
            connectToOpenMote(args.channel, frameFilteringLevel)

            stopSniffingThread = False
            sniffingThread = threading.Thread(target=actual_sniffer, args=[args.channel, frameFilteringLevel, args.replace_fcs])
            sniffingThread.start()

            INPUT('Press return key to pause sniffer (and to choose a different channel)\n')
            stopSniffingThread = True
            sniffingThread.join()

            if actualSnifferTerminated:
                break

            if wiresharkProcess != None:
                if wiresharkProcess.poll() != None:
                    print('ERROR: Wireshark no longer running')
                    break

            try:
                ser.write(encode('STOP'))
            except:
                print('ERROR: Failed to inform OpenMote about sniffer being paused')
                break

            args.channel = pickRadioChannel()
            if args.channel == None:
                break
    except KeyboardInterrupt:
        pass
    except:
        cleanup()
        raise

    cleanup()


if __name__ == "__main__":
    main()
