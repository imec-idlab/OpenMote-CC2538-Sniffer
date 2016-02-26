/**
 * @file       Board.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef BOARD_H_
#define BOARD_H_

#include <stdint.h>

#include "Callback.h"

enum SleepMode : uint8_t;

class Board {
public:
    Board();
    void reset(void);
    void setSleepMode(SleepMode sleepMode);
    void sleep(void);
    void wakeup(void);
    void enableInterrupts(void);
    void disableInterrupts(void);
    void enableFlashErase(void);
    uint32_t getCurrentTicks(void);
    bool isExpiredTicks(uint32_t futureTicks);
    void getEUI48(uint8_t* address);
    void getEUI64(uint8_t* address);
public:
    static const uint32_t BOARD_TICKS_PER_US;
private:
    SleepMode sleepMode_;
    PlainCallback flashEraseCallback_;
};

#endif /* BOARD_H_ */
