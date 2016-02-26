/**
 * @file       Timer.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>

#include "Callback.h"

class Timer
{

friend class InterruptHandler;

public:
    Timer(uint32_t peripheral, uint32_t base, uint32_t source, uint32_t config, uint32_t interrupt, uint32_t interrupt_mode);
    uint32_t getBase(void);
    uint32_t getSource(void);
    void init(uint32_t frequency);
    void start(void);
    void stop(void);
    uint32_t read(void);
    void setCallback(Callback* callback);
    void clearCallback(void);
    void enableInterrupts(void);
    void disableInterrupts(void);
protected:
    void interruptHandler(void);
private:
    uint32_t peripheral_;
    uint32_t base_;
    uint32_t source_;
    uint32_t config_;
    uint32_t interrupt_;
    uint32_t interrupt_mode_;
    uint32_t frequency_;

    Callback* callback_;
};

#endif /* TIMER_H_ */
