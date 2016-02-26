/**
 * @file       RadioTimer.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef RADIO_TIMER_H_
#define RADIO_TIMER_H_

#include <stdint.h>

#include "Callback.h"

class RadioTimer
{

friend class InterruptHandler;

public:
    RadioTimer(uint32_t interrupt);
    void start(void);
    void stop(void);
    void restart(void);
    uint32_t sleep(void);
    void wakeup(uint32_t ticks);
    uint32_t getCounter(void);
    void setCounter(uint32_t counter);
    uint32_t getPeriod(void);
    void setPeriod(uint32_t period);
    uint32_t getCompare(void);
    void setCompare(uint32_t compare);
    void setPeriodCallback(Callback* period);
    void clearPeriodCallback(void);
    void setCompareCallback(Callback* compare);
    void clearCompareCallback();
    void enableInterrupts(void);
    void disableInterrupts(void);
protected:
    void interruptHandler(void);
private:
    uint32_t interrupt_;

    Callback* period_;
    Callback* compare_;
};

#endif /* RADIO_H_ */
