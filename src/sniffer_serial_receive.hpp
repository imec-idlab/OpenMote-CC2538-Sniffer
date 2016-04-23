////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @author     Bruno Van de Velde (bruno@texus.me)
/// @copyright  This file is licensed under the GNU General Public License v2.
///
////////////////////////////////////////////////////////////////////////////////////////////////////////

void Serial::Receive::uartByteReceived()
{
    uartRxBuffer[uartRxBufferIndexWrite] = uart.readByte();
    if (++uartRxBufferIndexWrite == sizeof(uartRxBuffer))
        uartRxBufferIndexWrite = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

inline void Serial::Receive::serialReceive(uint8_t byte)
{
    // Check if the byte is special (start or end byte)
    if (byte == HDLC_FLAG)
    {
        // Check if the frame is complete
        if (receivingStatus)
            receivedEndByte();
        else // This is the opening byte
            receivedStartByte();
    }
    else // Normal data byte
    {
        // Put the byte in the receive buffer
        if (receivingStatus && messageLen < SERIAL_RX_MAX_MESSAGE_LEN)
            addByteToBuffer(byte);
        else
        {
            // Something went wrong, start retransmitting
            bufferIndexSerialSend = bufferIndexAcked;
            receivingStatus = false;
            messageLen = 0;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

inline void Serial::Receive::receivedStartByte()
{
    receivingStatus = true;
    escaping = false;
    messageLen = 0;
    crc = CRC_INIT;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

inline void Serial::Receive::receivedEndByte()
{
    // Detect out of sync
    if (messageLen == 0)
    {
        receivedStartByte();
        bufferIndexSerialSend = bufferIndexAcked;
        return;
    }

    // You are not supposed to pass here in escaping mode or with a too short message
    bool validMessage = false;
    if (!escaping && (messageLen >= 4)) // minimum packet size = 1 byte type + 1 byte length + 2 byte crc
    {
        if (crc == 0) // Calculating CRC of packet bytes + CRC bytes always results 0 for correct CRC
        {
            // Make sure the length byte is correct (value of length byte does not include type and length bytes)
            if (messageLen == message[1] + 2)
            {
                // Find out what message we received and act acordingly
                validMessage = decodeReceivedMessage();
            }
            else // This should not be possible, start retransmitting
            {
                led_orange.on();
                bufferIndexSerialSend = bufferIndexAcked;
            }
        }
    }

    // Start retransmitting if there was something wrong with the message
    if (!validMessage)
        bufferIndexSerialSend = bufferIndexAcked;

    receivingStatus = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

inline void Serial::Receive::addByteToBuffer(uint8_t byte)
{
    if (byte == HDLC_ESCAPE)
    {
        if (!escaping)
        {
            escaping = true;
        }
        else // Something is wrong
        {
            bufferIndexSerialSend = bufferIndexAcked;
            receivingStatus = false;
        }
    }
    else // The byte is not special
    {
        if (escaping)
        {
            byte = byte ^ HDLC_ESCAPE_MASK;
            escaping = false;
        }

        message[messageLen++] = byte;
        crc = crcCalculationStep(byte, crc);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

inline bool Serial::Receive::decodeReceivedMessage()
{
    if ((message[0] == SerialDataType::Ack) && (message[1] == ACK_MESSAGE_LENGTH))
        receivedACK();
    else if ((message[0] == SerialDataType::Nack) && (message[1] == NACK_MESSAGE_LENGTH))
        receivedNACK();
    else if ((message[0] == SerialDataType::Reset) && (message[1] == RESET_MESSAGE_LENGTH))
        receivedRESET();
    else if ((message[0] == SerialDataType::Stop) && (message[1] == STOP_MESSAGE_LENGTH))
        receivedSTOP();
    else
        return false;

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

inline void Serial::Receive::receivedACK()
{
    uint16_t receivedIndex = readUint16(message, ACK_INDEX_OFFSET);
    uint16_t receivedSeqNr = readUint16(message, ACK_SEQNR_OFFSET);

    // Ignore duplicate messages
    if (previousReceivedIndex == receivedIndex)
        return;

    // Validate the received index which has to lie within the unacked area and make sure the sequence number matches with it
    if (checkReceivedIndexAndSeqNr(receivedIndex, receivedSeqNr))
    {
        // Move the acked index forward
        bufferIndexAcked = receivedIndex + buffer[receivedIndex];

        // Keep track of the last received index
        previousReceivedIndex = receivedIndex;
    }
    else // This should not be possible, start retransmitting
    {
        led_orange.on();
        bufferIndexSerialSend = bufferIndexAcked;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

inline void Serial::Receive::receivedNACK()
{
    uint16_t receivedIndex = readUint16(message, NACK_INDEX_OFFSET);
    uint16_t receivedSeqNr = readUint16(message, NACK_SEQNR_OFFSET);

    // Validate the received index which has to lie within the unacked area or be the same as in the last ACK/NACK
    bool validIndex;
    if (previousReceivedIndex == receivedIndex)
        validIndex = true;
    else
        validIndex = checkReceivedIndexAndSeqNr(receivedIndex, receivedSeqNr);

    if (validIndex)
    {
        // Move the acked index forward (start from beginning again when reaching the end of the buffer)
        uint8_t packetLength = buffer[receivedIndex];
        bufferIndexAcked = receivedIndex + packetLength;
        if (buffer[bufferIndexAcked] == END_OF_BUFFER_BYTE)
            bufferIndexAcked = 0;

        // Resend everything up to the last acked packet
        bufferIndexSerialSend = bufferIndexAcked;

        // Keep track of the last received index
        previousReceivedIndex = receivedIndex;
    }
    else // This should not be possible, start retransmitting
    {
        led_orange.on();
        bufferIndexSerialSend = bufferIndexAcked;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

inline void Serial::Receive::receivedRESET()
{
    // Disable radio interrupts while resetting values
    IntDisable(INT_RFCORERTX);

    // Turn off warning lights
    led_green.on();
    led_yellow.off();
    led_orange.off();
    led_red.off();

    // Empty buffer and reset sequence number
    bufferIndexRadio = 0;
    bufferIndexSerialSend = 0;
    bufferIndexAcked = 0;
    seqNr = 0;

    // Verify that the received channel is within the correct range
    uint8_t channel = message[RESET_CHANNEL_OFFSET];
    if (channel >= 11 && channel <= 26)
    {
        // Set the requested channel
        radio.setChannel(channel);

        // Send the READY message
        Send::sendReadyPacket();

        // Allow new radio packets now
        CC2538_RF_CSP_ISFLUSHRX();
        IntEnable(INT_RFCORERTX);
        CC2538_RF_CSP_ISRXON();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

inline void Serial::Receive::receivedSTOP()
{
    // Disable radio interrupts and clear the radio buffer
    IntDisable(INT_RFCORERTX);
    CC2538_RF_CSP_ISFLUSHRX();

    led_green.off();
    led_yellow.off();
    led_orange.off();
    led_red.off();

    // Empty buffer and reset sequence number
    bufferIndexRadio = 0;
    bufferIndexSerialSend = 0;
    bufferIndexAcked = 0;
    seqNr = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

inline bool Serial::Receive::checkReceivedIndexAndSeqNr(uint16_t receivedIndex, uint16_t receivedSeqNr)
{
    if (receivedIndex >= sizeof(buffer))
        return false;
    else if (bufferIndexAcked > bufferIndexSerialSend) // around end and start of buffer
    {
        if ((receivedIndex < bufferIndexAcked) && (receivedIndex > bufferIndexSerialSend))
            return false;
    }
    else // if (bufferIndexAcked <= bufferIndexSerialSend)
    {
        if ((receivedIndex < bufferIndexAcked) || (receivedIndex > bufferIndexSerialSend))
            return false;
    }

    uint16_t seqNrInBuffer = readUint16(buffer, receivedIndex + BUFFER_SEQNR_OFFSET);

    if (seqNrInBuffer != receivedSeqNr)
        return false;

    return true;
}
