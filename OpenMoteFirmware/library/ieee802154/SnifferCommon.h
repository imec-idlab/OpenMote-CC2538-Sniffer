/**
 * @file       Sniffer.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef SNIFFER_COMMON_H_
#define SNIFFER_COMMON_H_

#include "Board.h"
#include "Callback.h"
#include "Mutex.h"
#include "Radio.h"

class SnifferCommon;

typedef GenericCallback<SnifferCommon> SnifferCallback;

class SnifferCommon
{
public:
    SnifferCommon(Board& board, Radio& radio);
    void init(void);
    void start(void);
    void stop(void);
    void setChannel(uint8_t channel);
    virtual void processRadioFrame(void) = 0;
    void initFrame(uint8_t* buffer, uint8_t length, int8_t rssi, uint8_t lqi, uint8_t crc);
protected:
    void radioRxInitCallback(void);
    void radioRxDoneCallback(void);
protected:
    Board board_;
    Radio radio_;

    SnifferCallback snifferRadioRxInitCallback_;
    SnifferCallback snifferRadioRxDoneCallback_;

    Mutex mutex;

    uint8_t macAddress[6];
    static const uint8_t broadcastAddress[6];
    static const uint8_t ethernetType[2];

    uint8_t  radioBuffer[128];
    uint8_t* radioBuffer_ptr;
    uint8_t  radioBuffer_len;

    uint8_t  outputBuffer[255];
    uint8_t* outputBuffer_ptr;
    uint32_t outputBuffer_len;

    int8_t  rssi;
    uint8_t lqi;
    uint8_t crc;
};

#endif /* SNIFFER_COMMON_H_ */
