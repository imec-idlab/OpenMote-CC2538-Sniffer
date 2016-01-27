/*================================ include ==================================*/

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "openmote-cc2538.h"

#include "Callback.h"
#include "Serial.h"
#include "uart.h"

/*================================ define ===================================*/

#define HDLC_FLAG           0x7E
#define HDLC_ESCAPE         0x7D
#define HDLC_ESCAPE_MASK    0x20

#define GREEN_LED_TASK_PRIORITY      ( tskIDLE_PRIORITY + 4 )
#define RADIO_RECEIVE_TASK_PRIORITY  ( tskIDLE_PRIORITY + 3 )
#define SERIAL_SEND_TASK_PRIORITY    ( tskIDLE_PRIORITY + 1 )

#define BUFFER_LEN  8000

#define CC2538_RF_MIN_PACKET_LEN   ( 3 )
#define CC2538_RF_MAX_PACKET_LEN   ( 127 )
#define CC2538_RF_RSSI_OFFSET      ( 73 )


/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static void prvSerialTask(void *pvParameters);

static void rxInit(void);
static void rxDone(void);
static void serialByteReceived(void);

static void fastHDLC(uint8_t dataLength);

/*=============================== variables =================================*/

static const uint16_t lut[256] = {
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
};

static PlainCallback rxInitCallback(&rxInit);
static PlainCallback rxDoneCallback(&rxDone);
static PlainCallback serialReceiveCallback(&serialByteReceived);

// TODO: Remove debug byte
static uint16_t uartBufferLen = 0;
static uint8_t uartBuffer[((129+2+1)*2)+2]; // data+crc+DEBUG_BYTE (2x for when each character is escaped) plus begin and end character

static uint8_t receiveUartBuffer[16];
static uint8_t receiveUartBufferIndex = 0;

static uint8_t buffer[BUFFER_LEN];
static uint16_t bufferIndexRadio = 0;
static uint16_t bufferIndexSerialSend = 0;
static uint16_t bufferIndexAcked = 0;

static bool resetRequested = true;
static bool bufferIndexSerialSendRequiresUpdate = true;
static uint16_t bufferIndexSerialSendUpdatedValue = 0;

static uint16_t seqNr = 0;

/*================================= public ==================================*/

int main (void)
{
    // Make sure the buffer only contains zeros
    for (uint32_t i = 0; i < BUFFER_LEN; ++i)
        buffer[i] = 0;

    // Set the TPS62730 in bypass mode (Vin = 3.3V, Iq < 1 uA)
    tps62730.setBypass();

    // Enable erasing the Flash with the user button
    board.enableFlashErase();

    // Enable the IEEE 802.15.4 radio
    radio.setRxCallbacks(&rxInitCallback, &rxDoneCallback);
    radio.enable();
    radio.setChannel(25);
    radio.enableInterrupts();

    // Enable the UART peripheral
    uart.enable(460800/*921600*//*2000000*/, UART_CONFIG, UART_INT_MODE);
    uart.setRxCallback(&serialReceiveCallback);
    uart.enableInterrupts();

    // Enable the FIFO in the UART
///    UARTFIFODisable(base_); //TODO: any influence?

    // Create the sole task which will be executed infinitely
    xTaskCreate(prvSerialTask, (const char *) "Serial", 128, NULL, SERIAL_SEND_TASK_PRIORITY, NULL);

    radio.on();
    radio.receive();

    // Setup interrupts and call our serial task
    led_green.on();
    portDISABLE_INTERRUPTS();
    xPortStartScheduler();
}

/*================================ private ==================================*/

static void serialSend(void)
{
    if ((bufferIndexSerialSend != bufferIndexRadio) && !UARTBusy(uart.getBase()))
    {
        if (buffer[bufferIndexSerialSend] == 0xff)
            bufferIndexSerialSend = 0;

        // Fill the transmit buffer
        if (buffer[bufferIndexSerialSend] <= 132)
            fastHDLC(buffer[bufferIndexSerialSend] - 1);
        else
        {
            led_orange.on();
            return;
        }

        for (uint16_t i = 0; i < uartBufferLen; ++i)
            UARTCharPut(uart.getBase(), uartBuffer[i]);

        // Move the uart buffer index
        bufferIndexSerialSend += buffer[bufferIndexSerialSend];

        // When we didn't receive an ACK for some time we must resend packets
        if (((bufferIndexSerialSend > bufferIndexAcked) && (bufferIndexSerialSend - bufferIndexAcked >= 750))
         || ((bufferIndexSerialSend < bufferIndexAcked) && (BUFFER_LEN - bufferIndexAcked + bufferIndexSerialSend >= 750)))
        {
            led_yellow.on();
            bufferIndexSerialSend = bufferIndexAcked;

            // TODO: Check if we passed radio pointer when jumping back
            // TODO: Why? Acked isn't moved
        }
        else
            led_yellow.off();
    }
}

static void prvSerialTask(void *pvParameters)
{
    // The first byte in the transmit buffer is always the HDLC_FLAG
    uartBuffer[0] = HDLC_FLAG;

    while (true)
    {
        // In case of packet loss, move the serial send index back to resend packets
        if (bufferIndexSerialSendRequiresUpdate)
        {
            bufferIndexSerialSendRequiresUpdate = false;
            bufferIndexSerialSend = bufferIndexSerialSendUpdatedValue;
            bufferIndexAcked = bufferIndexSerialSend; // TODO: Why?

            // TODO: Check if we passed radio pointer when jumping back

            // If a reset was requested, tell the pc that we are ready
            if (resetRequested)
            {
                resetRequested = false;

                UARTCharPut(uart.getBase(), '~');
                UARTCharPut(uart.getBase(), 'R');
                UARTCharPut(uart.getBase(), 'E');
                UARTCharPut(uart.getBase(), 'A');
                UARTCharPut(uart.getBase(), 'D');
                UARTCharPut(uart.getBase(), 'Y');
                UARTCharPut(uart.getBase(), 141);
                UARTCharPut(uart.getBase(), 58);
                UARTCharPut(uart.getBase(), '~');

                seqNr = 0;
            }
        }

        serialSend();
    }
}

static void rxInit(void)
{
    // Turn on the radio LED as the radio is now receiving a packet
    led_red.on();
}

static void rxDone(void)
{
    // Turn off the radio LED as the packet is now received
    led_red.off();

    // Check the packet length (first byte)
    uint8_t packetLength = HWREG(RFCORE_SFR_RFDATA);
    if ((packetLength > CC2538_RF_MAX_PACKET_LEN) || (packetLength < CC2538_RF_MIN_PACKET_LEN))
    {
        radio.receive();
        return;
    }

    uint8_t fullPacketLength = packetLength + 5; // Packet including FCS plus extra bytes that we store

    // The index must never pass the ack index, otherwise we may lose a packet when a NACK comes in
    if (((bufferIndexRadio + fullPacketLength >= BUFFER_LEN)
      && ((bufferIndexAcked > bufferIndexRadio) || (bufferIndexAcked < fullPacketLength)))
     || ((bufferIndexRadio + fullPacketLength < BUFFER_LEN)
      && (bufferIndexAcked > bufferIndexRadio)
      && (bufferIndexAcked < bufferIndexRadio + fullPacketLength)))
    {
        // Indicate that we are no longer lossless and discard this packet
        led_orange.on();
    }
    else // No packet lost, continue normally
    {
        // Increment sequence number but skip the zero after using the highest number
        if (++seqNr == 0)
            seqNr = 1;

        // Check that there is still space behind the last packet
        if (bufferIndexRadio + fullPacketLength >= BUFFER_LEN)
        {
            // Mark that the last part of the buffer as unused and start at the beginning
            buffer[bufferIndexRadio] = 0xff;
            bufferIndexRadio = 0;
        }

        // Put the amount of bytes that we will use (including this length byte) in the buffer
        buffer[bufferIndexRadio] = fullPacketLength;

        // The first two bytes are the index in the buffer right after this packet
        buffer[bufferIndexRadio+1] = (bufferIndexRadio >> 8) & 0xff;
        buffer[bufferIndexRadio+2] = (bufferIndexRadio >> 0) & 0xff;

        // The next two bytes form the sequence number (for verifying if correct packet is still in buffer)
        buffer[bufferIndexRadio+3] = (seqNr >> 8) & 0xff;
        buffer[bufferIndexRadio+4] = (seqNr >> 0) & 0xff;

        // Copy the RX buffer to the buffer (except for the CRC)
        for (uint8_t i = 0; i < packetLength; i++)
            buffer[bufferIndexRadio+5+i] = HWREG(RFCORE_SFR_RFDATA);

        // The next two bytes are the FCS replacement (2 byte CRC was replaced with 1 byte RSSI, 1 bit FCS Valid and 7 bit LQI)
        buffer[bufferIndexRadio+5+packetLength] = (int8_t(HWREG(RFCORE_SFR_RFDATA)) - CC2538_RF_RSSI_OFFSET);
        buffer[bufferIndexRadio+6+packetLength] = HWREG(RFCORE_SFR_RFDATA);

        // Move the buffer index forward
        bufferIndexRadio += fullPacketLength;
    }

    radio.receive();
}

static void serialByteReceived(void)
{
    static uint16_t crc = 0xffff;
    static bool receivingStatus = false;
    static bool escaping = false;

    // Read byte from the UART
    uint8_t byte = uart.readByte();
    if (byte == HDLC_FLAG)
    {
        // Check if the frame is complete
        if (receivingStatus)
        {
            // Detect out of sync
            if (receiveUartBufferIndex == 0)
            {
                escaping = false;
                bufferIndexSerialSendUpdatedValue = bufferIndexAcked;
                bufferIndexSerialSendRequiresUpdate = true;
                return;
            }

            // Something went wrong if we are still in escaping mode or if message is too short, resend everything since last ACK
            if (escaping || (receiveUartBufferIndex < 3))
            {
                receivingStatus = false;
                escaping = false;
                receiveUartBufferIndex = 0;
                bufferIndexSerialSendUpdatedValue = bufferIndexAcked;
                bufferIndexSerialSendRequiresUpdate = true;
                return;
            }

            // Handle the received packet
            {
                if (crc == 0)
                {
                    receiveUartBufferIndex -= 2;

                    if ((receiveUartBufferIndex == 7)
                     && (receiveUartBuffer[0] = 'A')
                     && (receiveUartBuffer[1] = 'C')
                     && (receiveUartBuffer[2] = 'K'))
                    {
                        uint16_t index = (receiveUartBuffer[3] << 8) + receiveUartBuffer[4];
                        uint16_t seqNr = (receiveUartBuffer[5] << 8) + receiveUartBuffer[6];

                        // Make sure the sequence number matches the index (we didn't overwrite the packet already)
                        if ((buffer[index+3] << 8) + buffer[index+4] == seqNr)
                        {
                            // Move the acked index forward
                            bufferIndexAcked = ((buffer[index+1] << 8) + buffer[index+2]) + buffer[index];
                        }
                        else // This should not be possible, drop buffer
                        {
                            led_orange.on();
                            bufferIndexSerialSendUpdatedValue = bufferIndexRadio;
                            bufferIndexSerialSendRequiresUpdate = true;
                        }
                    }
                    else if ((receiveUartBufferIndex == 8)
                     && (receiveUartBuffer[0] = 'N')
                     && (receiveUartBuffer[1] = 'A')
                     && (receiveUartBuffer[2] = 'C')
                     && (receiveUartBuffer[3] = 'K'))
                    {
                        uint16_t index = (receiveUartBuffer[4] << 8) + receiveUartBuffer[5];
                        uint16_t seqNr = (receiveUartBuffer[6] << 8) + receiveUartBuffer[7];

                        // Make sure the sequence number matches the index (we didn't overwrite the packet already)
                        if ((buffer[index+3] << 8) + buffer[index+4] == seqNr)
                        {
                            // Move the acked index forward
                            bufferIndexAcked = ((buffer[index+1] << 8) + buffer[index+2]) + buffer[index];
                            if (buffer[bufferIndexAcked] == 0xff)
                                bufferIndexAcked = 0;

                            // Send everything up to the last acked packet
                            bufferIndexSerialSendUpdatedValue = bufferIndexAcked;
                            bufferIndexSerialSendRequiresUpdate = true;
                        }
                        else // This should not be possible, drop buffer
                        {
                            led_orange.on();
                            bufferIndexSerialSendUpdatedValue = bufferIndexRadio;
                            bufferIndexSerialSendRequiresUpdate = true;
                        }
                    }
                    else if ((receiveUartBufferIndex == 3)
                     && (receiveUartBuffer[0] = 'R')
                     && (receiveUartBuffer[1] = 'S')
                     && (receiveUartBuffer[2] = 'T'))
                    {
                        // Empty buffer and make next packet have sequence number 0
                        bufferIndexSerialSendUpdatedValue = bufferIndexRadio;
                        bufferIndexSerialSendRequiresUpdate = true;
                        resetRequested = true;
                        seqNr = 0;

                        led_yellow.off();
                        led_orange.off();
                    }
                    else // Invalid packet received, resend everything up to the last received ACK
                    {
                        bufferIndexSerialSendUpdatedValue = bufferIndexAcked;
                        bufferIndexSerialSendRequiresUpdate = true;
                    }
                }
                else // Invalid CRC, resend everything up to the last received ACK
                {
                    bufferIndexSerialSendUpdatedValue = bufferIndexAcked;
                    bufferIndexSerialSendRequiresUpdate = true;
                }
            }

            receivingStatus = false;
            receiveUartBufferIndex = 0;
        }
        else // This is the opening byte
        {
            receivingStatus = true;
            crc = 0xffff;
        }

        return;
    }

    // Watch out for buffer overflow
    if (receiveUartBufferIndex >= sizeof(receiveUartBuffer))
    {
        receivingStatus = false;
        escaping = false;
        receiveUartBufferIndex = 0;
        
        bufferIndexSerialSendUpdatedValue = bufferIndexAcked;
        bufferIndexSerialSendRequiresUpdate = true;
        return;
    }

    // TODO: If we pass here with receivingStatus is false then something is wrong -> handle this case

    // Put the byte in the receive buffer
    if (byte == HDLC_ESCAPE)
        escaping = true;
    else
    {
        if (escaping)
        {
            escaping = false;
            receiveUartBuffer[receiveUartBufferIndex++] = byte ^ HDLC_ESCAPE_MASK;
            crc = lut[(byte ^ HDLC_ESCAPE_MASK) ^ (uint8_t)(crc >> 8)] ^ (crc << 8);
        }
        else
        {
            receiveUartBuffer[receiveUartBufferIndex++] = byte;
            crc = lut[byte ^ (uint8_t)(crc >> 8)] ^ (crc << 8);
        }
    }
}

static void fastHDLC(uint8_t dataLength)
{
    // TODO: Remove debugInfo byte
    uint8_t byte;
    uartBufferLen = 2; // Start HDLC_FLAG is already there + 1 DEBUG_INFO BYTE
    uint8_t debugInfo = 0;
    if (bufferIndexAcked > bufferIndexRadio)
        debugInfo = (BUFFER_LEN - (bufferIndexAcked - bufferIndexRadio)) / 50;
    else
        debugInfo = (bufferIndexRadio - bufferIndexAcked) / 50;

    if (debugInfo == HDLC_FLAG || debugInfo == HDLC_ESCAPE)
    {
        uartBuffer[1] = HDLC_ESCAPE;
        uartBuffer[2] = debugInfo ^ HDLC_ESCAPE_MASK;
        uartBufferLen++;
    }
    else
        uartBuffer[1] = debugInfo;

    // Calculate the CRC
    uint16_t crc = 0xffff;
    crc = lut[debugInfo ^ (uint8_t)(crc >> 8)] ^ (crc << 8);
    for (uint8_t i = 0; i < dataLength; ++i)
        crc = lut[buffer[bufferIndexSerialSend + 1 + i] ^ (uint8_t)(crc >> 8)] ^ (crc << 8);

    // Escape the data
    for (uint8_t i = 0; i < dataLength; ++i)
    {
        byte = buffer[bufferIndexSerialSend + 1 + i];
    
        // Check if we are transmitting and HDLC flag or escape byte
        if (byte == HDLC_FLAG || byte == HDLC_ESCAPE)
        {
            // If so, write an HDLC escape symbol and the masked byte to the transmit buffer
            uartBuffer[uartBufferLen++] = HDLC_ESCAPE;
            uartBuffer[uartBufferLen++] = byte ^ HDLC_ESCAPE_MASK;
        }
        else // Just add the byte to the transmit buffer
            uartBuffer[uartBufferLen++] = byte;
    }

    // Escape the first CRC byte
    byte = (crc >> 8) & 0xFF;
    if (byte == HDLC_FLAG || byte == HDLC_ESCAPE)
    {
        uartBuffer[uartBufferLen++] = HDLC_ESCAPE;
        uartBuffer[uartBufferLen++] = byte ^ HDLC_ESCAPE_MASK;
    }
    else
        uartBuffer[uartBufferLen++] = byte;

    // Escape the second CRC byte
    byte = (crc >> 0) & 0xFF;
    if (byte == HDLC_FLAG || byte == HDLC_ESCAPE)
    {
        uartBuffer[uartBufferLen++] = HDLC_ESCAPE;
        uartBuffer[uartBufferLen++] = byte ^ HDLC_ESCAPE_MASK;
    }
    else
        uartBuffer[uartBufferLen++] = byte;

    // Add the ending HDLC_FLAG byte
    uartBuffer[uartBufferLen++] = HDLC_FLAG;
}
