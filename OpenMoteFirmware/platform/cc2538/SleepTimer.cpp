/**
 * @file       SleepTimer.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "SleepTimer.h"
#include "InterruptHandler.h"

#include "cc2538_include.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

SleepTimer::SleepTimer(uint32_t interrupt):
    interrupt_(interrupt)
{
}

void SleepTimer::start(uint32_t counts)
{
    uint32_t current;

    // Get current counter
    current = SleepModeTimerCountGet();

    // Set future timeout
    SleepModeTimerCompareSet(current + counts);
}

void SleepTimer::stop(void)
{
    // Nothing to do here, SleepTimer cannot be stopped
}

uint32_t SleepTimer::sleep(void)
{
    return 0;
}

void SleepTimer::wakeup(uint32_t ticks)
{

}

uint32_t SleepTimer::getCounter(void)
{
    // Get current counter
    return SleepModeTimerCountGet();
}

bool SleepTimer::isExpired(uint32_t future)
{
    uint32_t current;
    int32_t delta;

    // Get current counter
    current = SleepModeTimerCountGet();

    // Calculate delta
    delta = (int32_t) (current - future);

    // Return true if expired
    return (delta < 0);
}

void SleepTimer::setCallback(Callback* callback)
{
    callback_ = callback;
}

void SleepTimer::clearCallback(void)
{
    callback_ = nullptr;
}

void SleepTimer::enableInterrupts(void)
{
    InterruptHandler::getInstance().setInterruptHandler(this);

    IntEnable(interrupt_);
}

void SleepTimer::disableInterrupts(void)
{
    IntDisable(interrupt_);

    InterruptHandler::getInstance().clearInterruptHandler(this);
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

void SleepTimer::interruptHandler(void)
{
    if (callback_ != nullptr)
    {
        callback_->execute();
    }
}
