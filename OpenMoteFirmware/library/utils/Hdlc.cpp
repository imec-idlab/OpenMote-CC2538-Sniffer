/**
 * @file       Hdlc.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Hdlc.h"

/*================================ define ===================================*/

static const uint8_t HDLC_FLAG        = 0x7E;
static const uint8_t HDLC_ESCAPE      = 0x7D;
static const uint8_t HDLC_ESCAPE_MASK = 0x20;

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Hdlc::Hdlc(CircularBuffer& rxCircularBuffer, CircularBuffer& txCircularBuffer):
    rxCircularBuffer_(rxCircularBuffer), txCircularBuffer_(txCircularBuffer)
{
}

HdlcResult Hdlc::rxOpen(void)
{
    rxStatus = HdlcStatus_Idle;
    rxLastByte = 0;
    rxIsEscaping = false;
    rxCrc.init();

    return HdlcResult_Ok;
}

HdlcResult Hdlc::rxPut(uint8_t byte)
{
    HdlcResult result = HdlcResult_Ok;

    // Check the current HDLC status
    if (rxStatus == HdlcStatus_Idle &&
        rxLastByte == HDLC_FLAG &&
        byte != HDLC_FLAG) // Start of HDLC frame
    {
        rxStatus = HdlcStatus_Busy;

        // Parse the current byte
        result = rxParse(byte);
    }
    else if (rxStatus == HdlcStatus_Busy &&
             byte != HDLC_FLAG) // Middle of HDLC frame
    {
        // Parse the current byte
        result = rxParse(byte);
    }
    else if (rxStatus == HdlcStatus_Busy &&
             byte == HDLC_FLAG) // End of HDLC frame
    {
        // Finished receiving an HDLC frame
        rxStatus = HdlcStatus_Done;
    }

    // Record the last received byte
    rxLastByte = byte;

    // Return the current status
    return result;
}

HdlcStatus Hdlc::getRxStatus(void){
    return rxStatus;
}

HdlcResult Hdlc::rxClose(void)
{
    HdlcResult result;

    // Return the status of the receive CRC
    result = rxCrc.check() ? HdlcResult_Ok : HdlcResult_Error;

    return result;
}

HdlcResult Hdlc::txOpen(void)
{
    int32_t status;

    // Initialize the transmit CRC module
    txCrc.init();

    // Write the opening HDLC flag to the transmit buffer
    status = txCircularBuffer_.write(HDLC_FLAG);
    if (!status) return HdlcResult_Error;

    return HdlcResult_Ok;
}

HdlcResult Hdlc::txPut(uint8_t byte)
{
    bool status;

    // Push the byte to the transmit CRC module
    txCrc.set(byte);

    // Check if we are transmitting and HDLC flag or escape byte
    if (byte == HDLC_FLAG || byte == HDLC_ESCAPE)
    {
        // If so, write an HDLC escape symbol to the transmit buffer
        status = txCircularBuffer_.write(HDLC_ESCAPE);
        if (!status) return HdlcResult_Error;

        // Transform the current byte
        byte ^= HDLC_ESCAPE_MASK;
    }

    // Write the current byte to the transmit buffer
    status = txCircularBuffer_.write(byte);
    if (!status) return HdlcResult_Error;

    return HdlcResult_Ok;
}

HdlcResult Hdlc::txPut(uint8_t* buffer, int32_t size)
{
    HdlcResult status = HdlcResult_Ok;

    while (size-- && status == HdlcResult_Ok)
    {
        status = txPut(*buffer++);
    }

    return status;
}

HdlcResult Hdlc::txClose(void)
{
    HdlcResult result;
    uint32_t status;
    uint16_t crc;
    uint8_t byte;

    // Get the CRC value
    crc = txCrc.get();

    // Write the CRC value to the transmit buffer
    byte = (crc >> 8) & 0xFF;
    result = txPut(byte);
    if (result != HdlcResult_Ok) return HdlcResult_Error;

    byte = (crc >> 0) & 0xFF;
    result = txPut(byte);
    if (result != HdlcResult_Ok) return HdlcResult_Error;

    // Write the closing HDLC flag to the transmit buffer
    status = txCircularBuffer_.write(HDLC_FLAG);
    if (!status) return HdlcResult_Error;

    return HdlcResult_Ok;
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

HdlcResult Hdlc::rxParse(uint8_t byte)
{
    int32_t status;

    // Check the received byte
    if (byte == HDLC_ESCAPE)
    {
        // Mark as currently escaping
        rxIsEscaping = true;
    }
    else
    {
        if (rxIsEscaping == true) // If we received an escape byte
        {
            // Update the received byte
            byte = byte ^ HDLC_ESCAPE_MASK;

            // Unmark as currently escaping
            rxIsEscaping = false;
        }

        // Write a byte to the receive buffer
        status = rxCircularBuffer_.write(byte);
        if (status == -1) return HdlcResult_Error;

        // Push the byte to the CRC module
        rxCrc.set(byte);
    }

    return HdlcResult_Ok;
}
