/**
 * @file       Serial.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Serial.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

static const uint8_t CRC_LENGTH = 2;    // Length of the CRC

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Serial::Serial(Uart& uart):
    uart_(uart), \
    rxBuffer_(receive_buffer_, sizeof(receive_buffer_)), \
    txBuffer_(transmit_buffer_, sizeof(transmit_buffer_)), \
    hdlc_(rxBuffer_, txBuffer_), \
    rxCallback_(this, &Serial::rxCallback), txCallback_(this, &Serial::txCallback)
{
}

void Serial::init(void)
{
    // Register UART callbacks
    uart_.setRxCallback(&rxCallback_);
    uart_.setTxCallback(&txCallback_);

    // Enable UART interrupts
    uart_.enableInterrupts();

    // Lock the UART receive
    uart_.rxLock();

    // Open the HDLC receive buffer
    hdlc_.rxOpen();
}

void Serial::write(uint8_t* data, uint32_t size)
{
    HdlcResult result = HdlcResult_Ok;
    uint8_t byte;
    bool status;

    // Take the UART lock
    uart_.txLock();

    // Reset the UART transmit buffer
    txBuffer_.reset();

    // Open the HDLC transmit buffer
    result = hdlc_.txOpen();
    if (result != HdlcResult_Ok) goto error;

    // For each byte in the buffer
    result = hdlc_.txPut(data, size);
    if (result != HdlcResult_Ok) goto error;

    // Close the HDLC buffer
    result = hdlc_.txClose();
    if (result != HdlcResult_Ok) goto error;

    // Read first byte from the UART transmit buffer
    status = txBuffer_.read(&byte);
    if (status != true) goto error;

    // Write first byte to the UART
    uart_.writeByte(byte);

    return;

error:
    // Reset the UART buffer
    txBuffer_.reset();

    // Release the UART lock
    uart_.txUnlock();

    return;
}

uint32_t Serial::read(uint8_t* buffer, uint32_t size)
{
    uint8_t data;
    uint32_t length;

    // Lock the UART receive
    uart_.rxLock();

    // Update the length value and account for the CRC bytes
    length = rxBuffer_.getSize() - CRC_LENGTH;

    // Check for buffer overflow
    if (length <= size)
    {
        // Copy all bytes to the buffer except the CRC bytes
        while (size--) {
            rxBuffer_.read(&data);
            *buffer++ = data;
        }

        // Read the CRC bytes from the buffer
        rxBuffer_.read(&data);
        rxBuffer_.read(&data);
    }
    else
    {
        length = 0;
    }

    // Reset the receive buffer
    rxBuffer_.reset();

    // Open the HDLC receive buffer
    hdlc_.rxOpen();

    return length;
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

void Serial::rxCallback(void)
{
    HdlcStatus status;
    HdlcResult result;
    uint8_t byte;

    // Read byte from the UART
    byte = uart_.readByte();

    // Put the byte in the HDLC receive buffer
    result = hdlc_.rxPut(byte);
    if (result == HdlcResult_Error) goto error;

    // Get the HDLC status
    status = hdlc_.getRxStatus();

     // If HDLC frame is completed
    if (status == HdlcStatus_Done)
    {
        // Close the HDLC frame
        result = hdlc_.rxClose();
        if (result == HdlcResult_Error) goto error;

        // Once done, free the UART lock
        uart_.rxUnlockFromInterrupt();
    }

    return;

error:
    // Reset the receive buffer
    rxBuffer_.reset();

    // Open the HDLC receive buffer
    hdlc_.rxOpen();

    return;
}

void Serial::txCallback(void)
{
    uint8_t byte;

    // Read byte from the UART transmit buffer
    if (txBuffer_.read(&byte) == true)
    {
        // Write byte to the UART
        uart_.writeByte(byte);
    }
    else
    {
        // Once done, free the UART lock
        uart_.txUnlockFromInterrupt();
    }
}
