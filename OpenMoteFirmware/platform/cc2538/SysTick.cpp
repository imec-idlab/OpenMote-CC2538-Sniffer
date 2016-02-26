/**
 * @file       SysTick.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "SysTick.h"
#include "InterruptHandler.h"

#include "cc2538_include.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

SysTick::SysTick(uint32_t period):
    period_(period)
{
}

void SysTick::init(void)
{
    SysTickPeriodSet(period_);
}

void SysTick::start(void)
{
    SysTickEnable();
}

void SysTick::stop(void)
{
    SysTickDisable();
}

void SysTick::setCallback(Callback* callback)
{
    callback_ = callback;
}

void SysTick::clearCallback(void)
{
    callback_ = nullptr;
}

void SysTick::enableInterrupts(void)
{
    InterruptHandler::getInstance().setInterruptHandler(this);

    // Enable SysTick interrupts
    SysTickIntEnable();
}

void SysTick::disableInterrupts(void)
{
    SysTickIntDisable();
}

/*=============================== protected =================================*/

void SysTick::interruptHandler(void)
{
    if (callback_ != nullptr)
    {
        callback_->execute();
    }
}

/*================================ private ==================================*/
