/**
 * @file       Ethernet.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef ETHERNET_H_
#define ETHERNET_H_

#include <stdint.h>

#include "EthernetDevice.h"
#include "EthernetFrame.h"

#include "Callback.h"

class Ethernet;

typedef GenericCallback<Ethernet> EthernetCallback;

class Ethernet
{
public:
    Ethernet(EthernetDevice& ethernetDevice_);
    void init(uint8_t* mac_address);
    void setCallback(Callback* callback_);
    void clearCallback(void);
    void transmitFrame(uint8_t* frame, uint32_t length);
    void receiveFrame(uint8_t* buffer, uint32_t* length);
private:
    void interruptHandler(void);
private:
    EthernetDevice& ethernetDevice;
    EthernetCallback interrupt;

    Callback* callback = nullptr;

    uint32_t receivedFrames;
    uint32_t receivedFramesError;
    uint32_t sentFrames;
    uint32_t sentFramesError;
};

#endif /* ETHERNET_H_ */
