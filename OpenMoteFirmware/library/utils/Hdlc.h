/**
 * @file       Hdlc.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef HDLC_H_
#define HDLC_H_

#include <stdint.h>

#include "CircularBuffer.h"
#include "Crc16.h"

enum HdlcResult : int32_t
{
    HdlcResult_Error = -1,
    HdlcResult_Ok    =  0,
};

enum HdlcStatus : int32_t
{
    HdlcStatus_Idle  =  1,
    HdlcStatus_Busy  =  2,
    HdlcStatus_Done  =  3,
};

class Hdlc
{
public:
    Hdlc(CircularBuffer& rxCircularBuffer, CircularBuffer& txCircularBuffer);

    HdlcResult rxOpen(void);
    HdlcResult rxPut(uint8_t byte);
    HdlcResult rxClose(void);
    HdlcStatus getRxStatus(void);

    HdlcResult txOpen(void);
    HdlcResult txPut(uint8_t byte);
    HdlcResult txPut(uint8_t* buffer, int32_t size);
    HdlcResult txClose(void);

private:
    HdlcResult rxParse(uint8_t byte);

private:
    CircularBuffer& rxCircularBuffer_;
    CircularBuffer& txCircularBuffer_;

    HdlcStatus rxStatus;
    uint8_t rxLastByte;
    bool rxIsEscaping;

    Crc16 rxCrc;
    Crc16 txCrc;
};

#endif /* HDLC_H_ */
