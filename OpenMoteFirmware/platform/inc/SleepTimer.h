/**
 * @file       SleepTimer.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef SLEEP_TIMER_H_
#define SLEEP_TIMER_H_

#include <stdint.h>

#include "Callback.h"

class SleepTimer
{

friend class InterruptHandler;

public:
    SleepTimer(uint32_t interrupt);
    void start(uint32_t counts);
    void stop(void);
    uint32_t sleep(void);
    void wakeup(uint32_t ticks);
    uint32_t getCounter(void);
    bool isExpired(uint32_t future);
    void setCallback(Callback* callback);
    void clearCallback(void);
    void enableInterrupts(void);
    void disableInterrupts(void);
protected:
    void interruptHandler(void);
private:
    uint32_t interrupt_;

    Callback* callback_;
};

#endif /* SLEEP_TIMER_H_ */
