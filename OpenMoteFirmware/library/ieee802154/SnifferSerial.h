/**
 * @file       SnifferSerial.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef SNIFFER_SERIAL_H_
#define SNIFFER_SERIAL_H_

#include "SnifferCommon.h"
#include "Serial.h"

class SnifferSerial : public SnifferCommon
{
public:
    SnifferSerial(Board& board, Radio& radio, Serial& serial);
    void processRadioFrame(void);
private:
    void initSerialFrame(uint8_t* buffer, uint8_t length);
private:
    Serial& serial_;
};

#endif /* SNIFFER_SERIAL_H_ */
