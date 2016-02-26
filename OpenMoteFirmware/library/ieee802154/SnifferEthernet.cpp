/**
 * @file       SnifferEthernet.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include <string.h>

#include "SnifferEthernet.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

/*=============================== variables =================================*/

/*================================= public ==================================*/

SnifferEthernet::SnifferEthernet(Board& board, Radio& radio, Ethernet& ethernet):
    SnifferCommon(board, radio), ethernet_(ethernet)
{
}

void SnifferEthernet::init(void)
{
    // Initialize the Ethernet with EUI48
    ethernet_.init(macAddress);
}

void SnifferEthernet::processRadioFrame(void)
{
    RadioResult result;

    // This call blocks until a radio frame is received
    if (mutex.take())
    {
        // Get packet from the radio
        radioBuffer_ptr = radioBuffer;
        radioBuffer_len = sizeof(radioBuffer);
        result = radio_.getPacket(radioBuffer_ptr, &radioBuffer_len, &rssi, &lqi, &crc);

        if (result == RadioResult_Success)
        {
            // Turn off the radio
            radio_.off();

            // Initialize Ethernet frame
            outputBuffer_ptr = outputBuffer;
            outputBuffer_len = sizeof(outputBuffer);
            initFrame(radioBuffer_ptr, radioBuffer_len, rssi, lqi, crc);

            // Transmit the radio frame over Ethernet
            ethernet_.transmitFrame(outputBuffer_ptr, outputBuffer_len);
        }
    }
}

/*================================ private ==================================*/
