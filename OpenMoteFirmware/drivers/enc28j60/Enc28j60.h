/**
 * @file       Enc28j60.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */
#ifndef ENC268J60_H_
#define ENC268J60_H_

#include <stdint.h>

#include "Gpio.h"
#include "Spi.h"

#include "Callback.h"
#include "Ethernet.h"

class Enc28j60;

typedef GenericCallback<Enc28j60> Enc28j60Callback;

class Enc28j60 : public EthernetDevice
{
public:
    Enc28j60(Spi& spi, GpioIn& gpio);
    void init(uint8_t* mac_address);
    void reset(void);
    void setCallback(Callback* callback);
    void clearCallback(void);
    OperationResult transmitFrame(uint8_t* data, uint32_t length);
    OperationResult receiveFrame(uint8_t* buffer, uint32_t* length);
protected:
    void interruptHandler(void);
private:
    uint8_t readOperation(uint8_t op, uint8_t address);
    void writeOperation(uint8_t op, uint8_t address, uint8_t data);
    void setBank(uint8_t address);
    uint8_t readRegisterByte (uint8_t address);
    void writeRegisterByte(uint8_t address, uint8_t data);
    uint16_t readRegister(uint8_t address);
    void writeRegister(uint8_t address, uint16_t data);
    uint16_t readPhyByte(uint8_t address);
    void writePhy(uint8_t address, uint16_t data);
    void writeBuffer(const uint8_t* data, uint16_t length);
    void readBuffer(uint8_t* data, uint16_t length);
    bool isLinkUp(void);
private:
    Spi& spi_;
    GpioIn& gpio_;

    Enc28j60Callback interrupt_;
    Callback* callback_;

    uint8_t  currentBank;
    uint32_t nextPacketPtr;
};

#endif /* ENC268J60_H_ */
