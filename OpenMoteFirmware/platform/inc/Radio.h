/**
 * @file       Radio.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef RADIO_H_
#define RADIO_H_

#include <stdint.h>

#include "Callback.h"

typedef enum
{
    RadioState_Off          = 0x00,
    RadioState_Idle         = 0x01,
    RadioState_ReceiveInit  = 0x02,
    RadioState_Receiving    = 0x03,
    RadioState_ReceiveDone  = 0x04,
    RadioState_TransmitInit = 0x05,
    RadioState_Transmitting = 0x06,
    RadioState_TransmitDone = 0x07
} RadioState;

typedef enum
{
    RadioResult_Error       = -1,
    RadioResult_Success     =  0
} RadioResult;

class Radio
{

friend class InterruptHandler;

public:
    Radio();
    void enable(void);
    void sleep(void);
    void wakeup(void);
    void on(void);
    void off(void);
    void reset(void);
    void setRxCallbacks(Callback* rxInit, Callback* rxDone);
    void setTxCallbacks(Callback* txInit, Callback* txDone);
    void enableInterrupts(void);
    void disableInterrupts(void);
    void setChannel(uint8_t channel);
    void setPower(uint8_t power);
    void transmit(void);
    void receive(void);
    RadioResult loadPacket(uint8_t* data, uint8_t length);
    RadioResult getPacket(uint8_t* buffer, uint8_t* length, int8_t* rssi, uint8_t* lqi, uint8_t* crc);
protected:
    void interruptHandler(void);
    void errorHandler(void);
private:
    volatile RadioState radioState_;

    Callback* rxInit_;
    Callback* rxDone_;
    Callback* txInit_;
    Callback* txDone_;
};

#endif /* RADIO_H_ */
