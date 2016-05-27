////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @author     Bruno Van de Velde (bruno@texus.me)
/// @copyright  This file is licensed under the GNU General Public License v2.
///
////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "sniffer_serial_send.hpp"

namespace Sniffer
{
    uint8_t  uartTxBuffer[SERIAL_TX_BUFFER_SIZE];
    uint16_t uartTxBufferLen = 0;

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    void SerialSend::initialize()
    {
        // The first byte in the transmit buffer is always the HDLC_FLAG
        uartTxBuffer[0] = HDLC_FLAG;

        // The second byte in the transmit buffer is a fixed type indicating that we are sending a packet
        uartTxBuffer[1] = SerialDataType::Packet;

        // Tell the host that we are ready to start sniffing
        sendReadyPacket();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    void SerialSend::send()
    {
        // Start at the beginning of the buffer when we have reached the end
        if (buffer[bufferIndexSerialSend] == END_OF_BUFFER_BYTE)
            bufferIndexSerialSend = 0;

        // Check if the length byte is valid
        if ((buffer[bufferIndexSerialSend] > CC2538_RF_MAX_PACKET_LEN + BUFFER_EXTRA_BYTES)
         || (bufferIndexSerialSend + buffer[bufferIndexSerialSend] >= sizeof(buffer)))
        {
            // Something unknown went terribly wrong, reset the sniffer and continue sniffing
            reset(); // disable radio interrupts and reset global variables
            led_red.on(); // indicate that we are no longer lossless
            SerialSend::sendReadyPacket(); // tell the pc that the sequence number count restarted
            IntEnable(INT_RFCORERTX); // re-enable radio interrupts
            CC2538_RF_CSP_ISRXON(); // turn radio back on in receiving mode
            return;
        }

        // Fill the transmit buffer and send it over the UART
        hdlcEncode();
        for (uint16_t i = 0; i < uartTxBufferLen; ++i)
            UARTCharPut(uart.getBase(), uartTxBuffer[i]);

        // Move the uart buffer index
        bufferIndexSerialSend += buffer[bufferIndexSerialSend];

        // When we didn't receive an ACK for some time we must resend packets
        uint16_t dist = 0;
        if (bufferIndexSerialSend > bufferIndexAcked)
            dist = bufferIndexSerialSend - bufferIndexAcked;
        else if (bufferIndexSerialSend < bufferIndexAcked)
            dist = sizeof(buffer) - bufferIndexAcked + bufferIndexSerialSend;

        if (dist > RETRANSMIT_THRESHOLD)
            bufferIndexSerialSend = bufferIndexAcked;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    void SerialSend::sendReadyPacket()
    {
        uint16_t crc = CRC_INIT;
        crc = crcCalculationStep(SerialDataType::Ready, crc);
        crc = crcCalculationStep(2, crc); // length = 2 (our crc bytes)

        UARTCharPut(uart.getBase(), HDLC_FLAG);
        UARTCharPut(uart.getBase(), SerialDataType::Ready);
        UARTCharPut(uart.getBase(), 2); // length = 2 (our crc bytes)
        UARTCharPut(uart.getBase(), (crc >> 8) & 0xFF);
        UARTCharPut(uart.getBase(), (crc >> 0) & 0xFF);
        UARTCharPut(uart.getBase(), HDLC_FLAG);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    inline void SerialSend::hdlcEncode()
    {
        uartTxBufferLen = 2; // The first two bytes are always the same (hdlc flag and type) and are already in the buffer

        // Add the lenght of the data (size of buffer block + 2 byte serial crc)
        // The length byte from the buffer also contained itself but is not copied which is why we need to subtract 1 byte
        uint8_t dataLength = buffer[bufferIndexSerialSend] - 1;
        addByteToHdlc(dataLength + 2);

        // Escape the data and calculate the CRC (first byte from buffer is skipped as this contained the packet length)
        uint16_t crc = CRC_INIT;
        crc = crcCalculationStep(uartTxBuffer[1], crc);
        crc = crcCalculationStep(dataLength + 2, crc);

        for (uint8_t i = 1; i <= dataLength; ++i)
        {
            uint8_t& byte = buffer[bufferIndexSerialSend + i];

            addByteToHdlc(byte);
            crc = crcCalculationStep(byte, crc);
        }

        // Escape the CRC bytes
        addByteToHdlc((crc >> 8) & 0xFF);
        addByteToHdlc(crc & 0xFF);

        // Add the ending HDLC_FLAG byte
        uartTxBuffer[uartTxBufferLen++] = HDLC_FLAG;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////

    inline void SerialSend::addByteToHdlc(uint8_t byte)
    {
        if (byte == HDLC_FLAG || byte == HDLC_ESCAPE)
        {
            uartTxBuffer[uartTxBufferLen++] = HDLC_ESCAPE;
            uartTxBuffer[uartTxBufferLen++] = byte ^ HDLC_ESCAPE_MASK;
        }
        else
            uartTxBuffer[uartTxBufferLen++] = byte;
    }
}
