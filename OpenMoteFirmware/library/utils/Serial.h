/**
 * @file       Serial.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdint.h>

#include "Uart.h"

#include "CircularBuffer.h"
#include "Hdlc.h"

class Serial;

typedef GenericCallback<Serial> SerialCallback;

class Serial
{
public:
    Serial(Uart& uart);
    void init(void);
    void write(uint8_t* data, uint32_t size);
    uint32_t read(uint8_t* buffer, uint32_t size);
private:
    void rxCallback(void);
    void txCallback(void);
private:
    Uart& uart_;

    uint8_t receive_buffer_[256];
    CircularBuffer rxBuffer_;

    uint8_t transmit_buffer_[256];
    CircularBuffer txBuffer_;

    Hdlc hdlc_;

    SerialCallback rxCallback_;
    SerialCallback txCallback_;
};

#endif /* SERIAL_H_ */
