/**
 * @file       Ethernet.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Ethernet.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Ethernet::Ethernet(EthernetDevice& ethernetDevice_):
    ethernetDevice(ethernetDevice_), \
    interrupt(this, &Ethernet::interruptHandler), \
    receivedFrames(0), receivedFramesError(0), \
    sentFrames(0), sentFramesError(0)
{
}

void Ethernet::init(uint8_t* mac_address)
{
    ethernetDevice.init(mac_address);
    ethernetDevice.setCallback(&interrupt);
}

void Ethernet::setCallback(Callback* callback_)
{
    callback = callback_;
}

void Ethernet::clearCallback(void)
{
    callback = nullptr;
}

void Ethernet::transmitFrame(uint8_t* frame, uint32_t length)
{
    OperationResult result;

    result = ethernetDevice.transmitFrame(frame, length);

    if (result == ResultSuccess)
    {
        sentFrames++;
    }
    else if (result == ResultError)
    {
        sentFramesError++;
    }
}

void Ethernet::receiveFrame(uint8_t *buffer, uint32_t* length)
{
    OperationResult result;

    result = ethernetDevice.receiveFrame(buffer, length);

    if (result == ResultSuccess)
    {
        receivedFrames++;
    }
    else if (result == ResultError)
    {
        receivedFramesError++;
    }
}

/*=============================== protected =================================*/

void Ethernet::interruptHandler(void)
{
    if (callback != nullptr)
    {
        callback->execute();
    }
}

/*================================ private ==================================*/
