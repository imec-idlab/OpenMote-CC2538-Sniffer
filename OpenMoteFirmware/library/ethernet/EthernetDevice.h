/**
 * @file       EthernetDevice.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef ETHERNET_DEVICE_H_
#define ETHERNET_DEVICE_H_

#include <stdint.h>

#include "Callback.h"

enum OperationResult : int8_t
{
    ResultError   = -1,
    ResultSuccess =  0
};

class EthernetDevice
{
public:
    virtual void init(uint8_t* mac_address) = 0;
    virtual void reset(void) = 0;
    virtual void setCallback(Callback* callback_) = 0;
    virtual void clearCallback(void) = 0;
    virtual OperationResult transmitFrame(uint8_t* data, uint32_t length) = 0;
    virtual OperationResult receiveFrame(uint8_t* buffer, uint32_t* length) = 0;
protected:
    void setMacAddress(uint8_t* mac_address);
protected:
    uint8_t macAddress[6];
};

#endif /* ETHERNET_DEVICE_H_ */
