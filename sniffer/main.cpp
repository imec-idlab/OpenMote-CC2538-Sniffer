/*================================ include ==================================*/

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "openmote-cc2538.h"

#include "Callback.h"
#include "Serial.h"
#include "Crc16.h"
#include "uart.h"

/*================================ define ===================================*/

#define HDLC_FLAG           0x7E
#define HDLC_ESCAPE         0x7D
#define HDLC_ESCAPE_MASK    0x20

#define GREEN_LED_TASK_PRIORITY      ( tskIDLE_PRIORITY + 4 )
#define RADIO_RECEIVE_TASK_PRIORITY  ( tskIDLE_PRIORITY + 3 )
#define SERIAL_SEND_TASK_PRIORITY    ( tskIDLE_PRIORITY + 1 )

#define BUFFER_LEN  5000

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static void prvGreenLedTask(void *pvParameters);
static void prvSerialSendTask(void *pvParameters);

static void rxInit(void);
static void rxDone(void);
static void serialByteReceived(void);

static void fastHDLC(uint8_t dataLength);
static void reset(void);

/*=============================== variables =================================*/

static PlainCallback rxInitCallback(&rxInit);
static PlainCallback rxDoneCallback(&rxDone);
static PlainCallback serialReceiveCallback(&serialByteReceived);

// TODO: Recheck uartBuffer size
static uint16_t uartBufferLen = 0;
static uint8_t uartBuffer[((132+2)*2)+2]; // data+crc (2x for when each character is escaped) plus begin and end character

static uint8_t receiveUartBuffer[16];
static uint8_t receiveUartBufferIndex = 0;

static uint8_t buffer[BUFFER_LEN];
static uint16_t bufferIndexRadio = 0;
static uint16_t bufferIndexSerialSend = 0;
static uint16_t bufferIndexAcked = 0;

static bool bufferIndexSerialSendRequiresUpdate = false;
static uint16_t bufferIndexSerialSendUpdatedValue;

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
    radio.setChannel(26);
    radio.enableInterrupts();

    // Enable the UART peripheral
    uart.enable(460800/*921600*//*2000000*/, UART_CONFIG, UART_INT_MODE);
    uart.setRxCallback(&serialReceiveCallback);
    uart.enableInterrupts();

    // Create the blink task
    xTaskCreate(prvGreenLedTask, (const char *) "Green", 128, NULL, GREEN_LED_TASK_PRIORITY, NULL);
    xTaskCreate(prvSerialSendTask, (const char *) "SerialSend", 128, NULL, SERIAL_SEND_TASK_PRIORITY, NULL);

    radio.on();
    radio.receive();

    // Kick the FreeRTOS scheduler
    vTaskStartScheduler();
}

/*================================ private ==================================*/

static void prvGreenLedTask(void *pvParameters)
{
    while (true)
    {
        // Turn off the green LED and keep it for 950 ms
        led_green.off();
        vTaskDelay(950 / portTICK_RATE_MS);

        // Turn on the green LED and keep it for 50 ms
        led_green.on();
        vTaskDelay(50 / portTICK_RATE_MS);
    }
}

static void serialSend(void)
{
    if ((bufferIndexSerialSend != bufferIndexRadio) && !UARTBusy(uart.getBase()))
    {
        if (buffer[bufferIndexSerialSend] == 0xff)
            bufferIndexSerialSend = 0;

        // Fill the transmit buffer
        fastHDLC(buffer[bufferIndexSerialSend]);

        // Pass bytes to UART (TODO: Try to activate FIFO queue in UART to avoid blocking)
        for (uint16_t i = 0; i < uartBufferLen; ++i)
            UARTCharPut(uart.getBase(), uartBuffer[i]);

        // Move the uart buffer index
        bufferIndexSerialSend += buffer[bufferIndexSerialSend];

        // When we didn't receive an ACK for some time we must resend packets
        if (((bufferIndexSerialSend > bufferIndexAcked) && (bufferIndexSerialSend - bufferIndexAcked >= 1000))
         || ((bufferIndexSerialSend < bufferIndexAcked) && (BUFFER_LEN - bufferIndexAcked + bufferIndexSerialSend >= 800)))
        {
            led_yellow.on();
            bufferIndexSerialSend = bufferIndexAcked;
        }
    }
}

static void prvSerialSendTask(void *pvParameters)
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

    static RadioResult result;
    static uint8_t radio_buffer[125];
    static uint8_t radio_len = sizeof(radio_buffer);
    static int8_t rssi;
    static uint8_t lqi;
    static uint8_t crc;
    static uint16_t seqNr = 0;

    // Get a packet from the radio buffer
    radio_len = sizeof(radio_buffer);
    result = radio.getPacket(radio_buffer, &radio_len, &rssi, &lqi, &crc);
    //result = radio.getRawPacket(radio_buffer, &radio_len);
    if (result == RadioResult_Success)
    {
        // Increment sequence number but skip the zero after using the highest number
        if (++seqNr == 0)
            seqNr = 1;

        uint8_t fullPacketLength = radio_len + 8; // Packet size plus extra bytes that we add to it plus this byte

        // The index must never pass the ack index, otherwise we MAY lose a packet when a NACK comes in
        if (((bufferIndexRadio + fullPacketLength >= BUFFER_LEN)
          && ((bufferIndexAcked > bufferIndexRadio) || (bufferIndexAcked < fullPacketLength)))
         || ((bufferIndexRadio + fullPacketLength < BUFFER_LEN)
          && (bufferIndexAcked > bufferIndexRadio)
          && (bufferIndexAcked < bufferIndexRadio + fullPacketLength)))
        {
            reset();
            led_orange.on();
        }
        else // No packet lost, continue normally
        {
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

            // The next three bytes are RSSI, LQI and CRC
            buffer[bufferIndexRadio+5] = rssi;
            buffer[bufferIndexRadio+6] = lqi;
            buffer[bufferIndexRadio+7] = crc;

            // Everything after that is the actual packet
            memcpy(&buffer[bufferIndexRadio+8], radio_buffer, radio_len);

            // Move the buffer index forward
            bufferIndexRadio += fullPacketLength;
        }
    }

    // We are ready to receive the next packet
    radio.receive();
}

static void serialByteReceived(void)
{
    static bool receivingStatus = false;
    static bool escaping = false;

    // Read byte from the UART
    uint8_t byte = uart.readByte();

    if (byte == HDLC_FLAG)
    {
        // Check if the frame is complete
        if (receivingStatus)
        {
            // Something went wrong if we are still in escaping mode or if the message is too short
            if (escaping || (receiveUartBufferIndex < 3))
            {
                receivingStatus = false;
                escaping = false;
                receiveUartBufferIndex = 0;
                led_yellow.on();
            }

            {
                // Calculate the CRC
                Crc16 crcCalculator;
                crcCalculator.init();
                for (uint8_t i = 0; i < receiveUartBufferIndex - 2; ++i)
                    crcCalculator.set(receiveUartBuffer[i]);
                uint16_t crc = crcCalculator.get();

                if ((((crc >> 8) & 0xff) == receiveUartBuffer[receiveUartBufferIndex-2])
                 && ((crc & 0xff) == receiveUartBuffer[receiveUartBufferIndex-1]))
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
                        else // We were not fast enough
                        {
                            reset();
                            led_orange.on();
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
                        else // We were not fast enough
                        {
                            reset();
                            led_orange.on();
                        }
                    }
                    else if ((receiveUartBufferIndex == 3)
                     && (receiveUartBuffer[0] = 'R')
                     && (receiveUartBuffer[1] = 'S')
                     && (receiveUartBuffer[2] = 'T'))
                    {
                        reset();
                    }
                    else // Invalid packet received, send everything up to the last received ACK
                    {
                        bufferIndexSerialSendUpdatedValue = bufferIndexAcked;
                        bufferIndexSerialSendRequiresUpdate = true;
                    }
                }
                else // Invalid CRC, send everything up to the last received ACK
                {
                    bufferIndexSerialSendUpdatedValue = bufferIndexAcked;
                    bufferIndexSerialSendRequiresUpdate = true;
                }
            }

            receivingStatus = false;
            receiveUartBufferIndex = 0;
        }
        else // This is the opening byte
            receivingStatus = true;

        return;
    }

    // Watch out for buffer overflow
    if (receiveUartBufferIndex >= sizeof(receiveUartBuffer))
    {
        receivingStatus = false;
        escaping = false;
        receiveUartBufferIndex = 0;
        led_yellow.on();
    }

    // Put the byte in the receive buffer
    if (byte == HDLC_ESCAPE)
        escaping = true;
    else
    {
        if (escaping)
        {
            escaping = false;
            receiveUartBuffer[receiveUartBufferIndex++] = byte ^ HDLC_ESCAPE_MASK;
        }
        else
            receiveUartBuffer[receiveUartBufferIndex++] = byte;
    }
}

static void fastHDLC(uint8_t dataLength)
{
    // The size can not exceed 130 (125 of packet + 3 bytes with extra data + 2 bytes sequence number)
    ASSERT(dataLength <= 130);

    // TODO: Remove debugInfo byte
    uint8_t byte;
    uartBufferLen = 2; // Start HDLC_FLAG is already there
    uint8_t debugInfo = 0;
    if (bufferIndexAcked > bufferIndexRadio)
        debugInfo = (BUFFER_LEN - (bufferIndexAcked - bufferIndexRadio)) / 25;
    else
        debugInfo = (bufferIndexRadio - bufferIndexAcked) / 25;

    if (debugInfo == HDLC_FLAG || debugInfo == HDLC_ESCAPE)
    {
        uartBuffer[1] = HDLC_ESCAPE;
        uartBuffer[2] = debugInfo ^ HDLC_ESCAPE_MASK;
        uartBufferLen++;
    }
    else
        uartBuffer[1] = debugInfo;

    // Calculate the CRC
    Crc16 crcCalculator;
    crcCalculator.init();
    crcCalculator.set(debugInfo);
    for (uint8_t i = 0; i < dataLength; ++i)
        crcCalculator.set(buffer[bufferIndexSerialSend + 1 + i]);
    uint16_t crc = crcCalculator.get();

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

static void reset(void)
{
    radio.disableInterrupts();
    uart.disableInterrupts();

    led_yellow.off();
    led_orange.off();
    led_green.off();

    // Reset the entire buffer and start over
    for (uint32_t i = 0; i < BUFFER_LEN; ++i)
        buffer[i] = 0;

    bufferIndexRadio = 0;
    bufferIndexSerialSend = 0;
    bufferIndexAcked = 0;

    radio.enableInterrupts();
    uart.enableInterrupts();
}
