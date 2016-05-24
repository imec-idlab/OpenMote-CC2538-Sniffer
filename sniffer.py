#!/usr/bin/env python

__author__ = 'Bruno Van de Velde (bruno@texus.me)'
__license__ = 'GNU General Public License v2'

import serial
import os
import sys
import glob
import subprocess
import time
import platform
import threading
import argparse
import errno

platform = platform.system()
if platform != 'Windows' and platform != 'Linux' and platform != 'Darwin':
    raise RuntimeError('ERROR: Sniffer does not support platform "' + platform + '"')

if platform == 'Windows':
    from win32process import DETACHED_PROCESS
    import win32pipe
    import win32file

    if (sys.version_info > (3, 0)):
        import winreg
    else:
        import _winreg as winreg

if (sys.version_info > (3, 0)):
    INPUT = input
else:
    INPUT = raw_input


BAUDRATE          = 921600
ACK_THRESHOLD     = 300
SERIAL_TIMEOUT    = 0.3

INDEX_OFFSET      = 2
SEQ_NR_OFFSET     = 4
DATA_OFFSET       = 6

HDLC_FLAG        = 0x7E
HDLC_ESCAPE      = 0x7D
HDLC_ESCAPE_MASK = 0x20

class SerialDataType:
    Packet = 1
    Ack    = 2
    Nack   = 3
    Reset  = 4
    Ready  = 5
    Stop   = 6


stopSniffingThread = False
ser = serial.Serial()
output = None
outputIsFile = True
snifferThreadTerminated = False


def getSerialPortList():
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

    return ports


def selectSerialPortFromList(ports):
    while (True):
        try:
            print('Multiple serial ports were found:')
            for i in range(len(ports)):
                print('- [' + str(i) + '] ' + str(ports[i]))

            selectedPort = int(INPUT('Choose your serial port: '))
            if selectedPort < 0 or selectedPort >= len(ports):
                print('Input number is outside bounds!')
                continue
            else:
                return ports[selectedPort]

        except KeyboardInterrupt:
            return None

        except:
            print('Input parameter is not a number!')
            continue


def pickSerialPort():
    ports = getSerialPortList()

    if len(ports) == 0:
        print('No serial ports found!')
        return None
    elif len(ports) == 1:
        print('Using serial port "' + str(ports[0]) + '"')
        return ports[0]
    else:
        return selectSerialPortFromList(sorted(ports))


def pickRadioChannel():
    channel = 1
    while (channel < 11 or channel > 26) and channel != 0:
        try:
            channel = int(INPUT('Select the IEEE 802.15.4 channel number (11-26, 0 to exit): '))
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
                    return False
            else:
                os.remove(name)

        os.mkfifo(name)
    else:
        global output
        output = win32pipe.CreateNamedPipe('\\\\.\\pipe\\' + name,
                                           win32pipe.PIPE_ACCESS_OUTBOUND,
                                           win32pipe.PIPE_TYPE_MESSAGE | win32pipe.PIPE_WAIT,
                                           1,
                                           65536,
                                           65536,
                                           300,
                                           None)
    return True


def removePipe(name):
    if platform != 'Windows':
        os.remove(name)
    else:
        output.close()


def startWireshark(executableName, pipeName):
    nullOutput = open(os.devnull, 'w')
    while True:
        try:
            if platform != 'Windows':
                return subprocess.Popen([executableName, '-k', '-i', pipeName],
                                         stdout=nullOutput,
                                         stderr=nullOutput,
                                         preexec_fn=os.setsid)
            else:
                return subprocess.Popen([executableName, '-k', '-i', '\\\\.\\pipe\\' + pipeName],
                                         stdout=nullOutput,
                                         stderr=nullOutput,
                                         creationflags=DETACHED_PROCESS)
        except OSError as e:
            print('ERROR: Failed to execute "' + executableName + '". OSError: ' + str(e))
            executableName = str(INPUT('Please provide wireshark executable name: '))


def outputGlobalHeader():
    header = bytearray()
    header.extend([0xa1, 0xb2, 0xc3, 0xd4]) # magic number
    header.extend([0, 2]) # major version number
    header.extend([0, 4]) # minor version number
    header.extend([0]*4) # GMT to local correction
    header.extend([0]*4) # accuracy of timestamps
    header.extend([0, 0, 0xff, 0xff]) # max length of captured packets, in octets
    header.extend([0, 0, 0, 195]) # 802.15.4 protocol
    if outputIsFile:
        output.write(header)
    else:
        win32file.WriteFile(output, header)


def outputPacket(packet):
    ts_sec = int(time.time())
    ts_usec = int((time.time() - ts_sec) * 1000000)

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


def calcRadioCRC(msg):
    crc = 0x0000
    for val in msg:
        crc = precompiled_crc16_table[(crc ^ val) & 0xff] ^ (crc >> 8)
    return crc
    crc ^= 0xFFFF
    return crc ^ 0xffff;


def calcCRC(msg):
    crc = 0xFFFF
    for c in msg:
        crc = precompiled_crc16_table[c ^ ((crc >> 8) & 0xFF)] ^ ((crc << 8) & 0xFFFF)
    return crc;


def decode(msg, quiet=False):
    escaped = False
    result = bytearray()
    for c in msg:
        if c == HDLC_ESCAPE:
            escaped = True
            continue
        else:
            if escaped:
                result.append(c ^ HDLC_ESCAPE_MASK)
                escaped = False
            else:
                result.append(c)

    if len(result) < 4:
        if not quiet:
            print('WARNING: Received message too short')
        return ''

    if calcCRC(result[:-2]) != ((result[-2] << 8) + result[-1]):
        if not quiet:
            print('WARNING: Received message had incorrect serial CRC')
        return ''

    if result[0] != SerialDataType.Packet and result[0] != SerialDataType.Ready:
        if not quiet:
            print('WARNING: Received message had invalid type')
        return ''

    if result[1] != len(result) - 2:
        if not quiet:
            print('WARNING: Received message had incorrect length byte')
        return ''

    if result[0] == SerialDataType.Packet and len(result) < 8:
        if not quiet:
            print('WARNING: Received message too short for type Packet')
        return ''

    return result[:-2]


def encodeByte(byte):
    if byte == HDLC_FLAG or byte == HDLC_ESCAPE:
        return [HDLC_ESCAPE, byte ^ HDLC_ESCAPE_MASK]
    else:
        return [byte]


def encode(msg):
    crc = calcCRC(msg)

    result = bytearray()
    result.append(HDLC_FLAG)
    for c in msg:
        result.extend(encodeByte(c));

    result.extend(encodeByte((crc >> 8) & 0xFF));
    result.extend(encodeByte(crc & 0xFF));

    result.append(HDLC_FLAG)
    return result


def serialWrite(dataType, msg):
    data = bytearray()
    data.append(dataType)
    data.append(len(msg) + 2) # Length of passed message + 2 byte crc
    data.extend(msg)
    ser.write(encode(data))


def serialWriteAck(lastIndex, lastSeqNr):
    serialWrite(SerialDataType.Ack, [(lastIndex >> 8) & 0xff, lastIndex & 0xff,
                                     (lastSeqNr >> 8) & 0xff, lastSeqNr & 0xff])


def serialWriteNack(lastIndex, lastSeqNr):
    serialWrite(SerialDataType.Nack, [(lastIndex >> 8) & 0xff, lastIndex & 0xff,
                                      (lastSeqNr >> 8) & 0xff, lastSeqNr & 0xff])


def serialWriteStop():
    try:
        serialWrite(SerialDataType.Stop, [])
    except:
        pass


class PacketProcessor:
    def __init__(self, discardPacketsWithBadCRC, replaceFCS):
        self.discardPacketsWithBadCRC = discardPacketsWithBadCRC
        self.replaceFCS = replaceFCS
        self.resetVariables()

    def resetVariables(self):
        self.unackedByteCount = 0
        self.lastIndex = 0
        self.lastSeqNr = 0
        self.expectedSeqNr = 0
        self.retransmission = False

    def serialWriteAck(self):
        if self.unackedByteCount >= ACK_THRESHOLD:
            self.unackedByteCount = 0
            serialWriteAck(self.lastIndex, self.lastSeqNr)

    def ackUnackedBytes(self):
        if self.unackedByteCount > 0:
            self.unackedByteCount = ACK_THRESHOLD
            self.serialWriteAck()

    def processPacket(self, msg):
        if len(msg) == 0: # Message was invalid (e.g. wrong serial CRC)
            serialWriteNack(self.lastIndex, self.lastSeqNr)
            return True

        if msg[0] == SerialDataType.Ready:
            print('WARNING: Sniffer reset detected, restarting')
            return False

        # Ignore the packet if it had a wrong sequence number
        receivedSeqNr = (msg[SEQ_NR_OFFSET] << 8) + msg[SEQ_NR_OFFSET+1]
        if self.expectedSeqNr == receivedSeqNr:
            if self.expectedSeqNr == 0xffff:
                self.expectedSeqNr = 0
            else:
                self.expectedSeqNr += 1

            self.retransmission = False

            # Remember the index and sequence number of this packet in case the next one is corrupted
            self.lastIndex = (msg[INDEX_OFFSET] << 8) + msg[INDEX_OFFSET+1]
            self.lastSeqNr = (msg[SEQ_NR_OFFSET] << 8) + msg[SEQ_NR_OFFSET+1]

            # Discard packets with a bad CRC when requested
            if not self.discardPacketsWithBadCRC or msg[-1] & 128 != 0:

                # Recalculate the CRC unless the alternative FCS was requested or the CRC was invalid
                if not self.replaceFCS and msg[-1] & 128 != 0:
                    crc = calcRadioCRC(msg[DATA_OFFSET:-2])
                    msg[-2] = crc & 0xff
                    msg[-1] = (crc >> 8) & 0xff

                # Write Record Header and the packet to output
                outputPacket(msg[DATA_OFFSET:])

            # Send an ACK after enough bytes have been received
            self.unackedByteCount += len(msg)
            self.serialWriteAck()

        else:
            # If the sequence number is higher than expected then tell the sniffer that we are missing something
            if receivedSeqNr > self.expectedSeqNr:
                serialWriteNack(self.lastIndex, self.lastSeqNr)
            else:
                # The OpenMote is retransmitting stuff that we already have, so send an ACK to inform it about this
                if self.retransmission:
                    # If we already noticed it then only send an ACK every few packets
                    self.unackedByteCount += len(msg)
                    self.serialWriteAck()
                else:
                    # If this is the first retransmitted packet then immediately send an ACK
                    self.retransmission = True
                    self.unackedByteCount = ACK_THRESHOLD
                    self.serialWriteAck()

        return True


def connectToOpenMote(channel, quiet = False):
    ser.flushInput()
    ser.flushOutput()

    try:
        # Keep sending RESET packet and discard all bytes until the READY packet arrives
        receiving = False
        for i in range(3):
            if not quiet:
                print('Connecting to OpenMote...')

            serialWrite(SerialDataType.Reset, [channel])
            begin = time.time()
            while time.time() - begin < 1:
                c = ser.read(1)
                if len(c) > 0:
                    c = bytearray(c)[0]  # c is always a single byte, but it was returned as an array
                    if not receiving:
                        if c == HDLC_FLAG:
                            receiving = True
                            msg = bytearray()
                    else:
                        if c == HDLC_FLAG:
                            if len(msg) != 0:
                                receiving = False
                                msg = decode(msg, quiet=True)
                                if len(msg) > 0 and msg[0] == SerialDataType.Ready:
                                    if not quiet:
                                        print('Connected to OpenMote')
                                    return True
                        else:
                            msg.append(c)

    except serial.serialutil.SerialException as e:
        print('ERROR: Serial error. PySerial error: ' + str(e))

    print('ERROR: Failed to connect to OpenMote')
    return False


def snifferThread(channel, discardPacketsWithBadCRC, replaceFCS):
    global snifferThreadTerminated

    msg = bytearray()
    receiving = False
    packetProcessor = PacketProcessor(discardPacketsWithBadCRC, replaceFCS)

    try:
        while not stopSniffingThread:
            c = ser.read(1)
            if len(c) > 0:
                c = bytearray(c)[0]  # c is always a single byte, but it was returned as an array
                if not receiving:
                    receiving = True
                    msg = bytearray()

                    if c != HDLC_FLAG:
                        print('WARNING: encountered unexpected byte, assuming out of sync')
                        msg.append(c)
                else:
                    if c == HDLC_FLAG:
                        if len(msg) == 0:
                            print('WARNING: out of sync detected')
                        else:
                            receiving = False
                            if not packetProcessor.processPacket(decode(msg)):
                                # Something happened with the OpenMote, try to connect again
                                if not connectToOpenMote(channel):
                                    return  # Connection to OpenMote lost, terminate sniffer

                                msg = bytearray()
                                receiving = False
                                packetProcessor.resetVariables()
                                continue

                    else: # Not the closing byte
                        msg.append(c)

            else: # No character was read
                if receiving:
                    receiving = False
                    print('WARNING: expected another byte, assuming out of sync')
                    serialWriteNack(packetProcessor.lastIndex, packetProcessor.lastSeqNr)
                else:
                    # We haven't received any new packets for a moment, if there are still unacked bytes, acknowledge them now
                    packetProcessor.ackUnackedBytes()

    except serial.serialutil.SerialException as e:
        serialWriteStop()
        snifferThreadTerminated = True
        print('ERROR: Serial error, assuming OpenMote disconnected. PySerial error: ' + str(e))

    except IOError as e:
        serialWriteStop()
        snifferThreadTerminated = True
        if e.errno == errno.EPIPE:
            print('ERROR: Pipe to wireshark is broken')
        else:
            print('ERROR: Unknown error. IOError: ' + str(e))


def parseArguments():
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
                        help='Filename of .pcap file to output. The existence of this parameter decides whether real-time capturing with wireshark is used or whether the sniffer just outputs to a pcap file')
    parser.add_argument('-f', '--force', action='store_true',
                        help='Overwrite pcap file or pipe when it already exists')
    parser.add_argument('--replace-fcs', action='store_true',
                        help='Replace the normal radio FCS by the TI CC24XX FCS which contains the RSSI and LQI')
    parser.add_argument('--keep-bad-fcs', action='store_true',
                        help='Don\'t discard packets that have a bad checksum')
    return parser.parse_args()


def main():
    global ser
    global output
    global outputIsFile
    global stopSniffingThread
    global snifferThreadTerminated

    args = parseArguments()

    if args.channel != None and (args.channel < 11 or args.channel > 26):
        print('Channel should be between 11 and 26')
        return

    # If no serial port was provided as parameter, find one now
    if args.port == None:
        args.port = pickSerialPort()
        if args.port == None:
            return

    # Setup the serial connection
    try:
        ser = serial.Serial(port     = args.port,
                            baudrate = BAUDRATE,
                            parity   = serial.PARITY_NONE,
                            stopbits = serial.STOPBITS_ONE,
                            bytesize = serial.EIGHTBITS,
                            xonxoff  = False,
                            rtscts   = False,
                            dsrdtr   = False,
                            timeout  = SERIAL_TIMEOUT)
    except serial.serialutil.SerialException as e:
        print('ERROR: Could not connect to serial port. PySerial error: ' + str(e))
        return

    # If no channel was provided as parameter, ask the user on which channel to listen
    if args.channel == None:
        args.channel = pickRadioChannel()
        if args.channel == None:
            return

    # Test our serial connection before starting wireshark
    print('Testing serial connection...')
    if not connectToOpenMote(args.channel, quiet=True):
        return
    serialWriteStop()

    # Start wireshark when needed
    wiresharkProcess = None
    if args.pcap_file == None:
        print('Creating pipe...')
        if not createPipe(args.pipe_name, args.force):
            return

        print('Starting wireshark...')
        try:
            wiresharkProcess = startWireshark(args.wireshark_executable, args.pipe_name)
        except KeyboardInterrupt:
            removePipe(args.pipe_name)
            return

        print('Waiting for wireshark to be ready...')
        if platform != 'Windows':
            output = open(args.pipe_name, 'wb', buffering=0)
            outputIsFile = True
        else:
            win32pipe.ConnectNamedPipe(output, None)
            outputIsFile = False
        print('Connected to wireshark')

    else: # Write data to pcap file instead of real-time monitoring with wireshark
        print('Creating pcap file...')
        if os.path.exists(args.pcap_file):
            if not args.force:
                response = str(INPUT('File ' + args.pcap_file + ' already exists. Delete it and continue? [y/N] '))
                if response == 'y' or response == 'Y':
                    os.remove(args.pcap_file)
                else:
                    return
            else:
                os.remove(args.pcap_file)

        output = open(args.pcap_file, 'wb')
        outputIsFile = True

    # Write the global header to the output
    outputGlobalHeader()

    # Define a function that should be called if anything goes wrong from this point onwards or when the sniffer quits
    def cleanup():
        serialWriteStop()

        if outputIsFile:
            output.close()
        else:
            win32pipe.DisconnectNamedPipe(output)

        if args.pcap_file == None:
            removePipe(args.pipe_name)

    try:
        while True:
            if not connectToOpenMote(args.channel):
                break

            stopSniffingThread = False
            sniffingThread = threading.Thread(target=snifferThread, args=[args.channel, not args.keep_bad_fcs, args.replace_fcs])
            sniffingThread.start()

            INPUT('Press return key to pause sniffer (and to choose a different channel)\n')
            stopSniffingThread = True
            sniffingThread.join()

            if snifferThreadTerminated:
                break

            # Stop the sniffer when wireshark was already closed
            if wiresharkProcess != None and wiresharkProcess.poll() != None:
                break

            try:
                serialWrite(SerialDataType.Stop, [])
            except:
                print('ERROR: Failed to inform OpenMote about sniffer being paused')
                break

            args.channel = pickRadioChannel()
            if args.channel == None:
                break
    except KeyboardInterrupt:
        stopSniffingThread = True
        sniffingThread.join()
        pass
    except:
        cleanup()
        raise

    cleanup()


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

if __name__ == '__main__':
    main()
