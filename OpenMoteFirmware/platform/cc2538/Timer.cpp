/**
 * @file       Timer.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Timer.h"
#include "InterruptHandler.h"

#include "cc2538_include.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Timer::Timer(uint32_t peripheral, uint32_t base, uint32_t source, uint32_t config, uint32_t interrupt, uint32_t interrupt_mode):
    peripheral_(peripheral), base_(base), source_(source), config_(config), interrupt_(interrupt), interrupt_mode_(interrupt_mode)
{
}

uint32_t Timer::getBase(void)
{
    return base_;
}

uint32_t Timer::getSource(void)
{
    return source_;
}

void Timer::init(uint32_t frequency)
{
    // Store the Timer frequency
    frequency_ = frequency;

    // Disable peripheral previous to configuring it
    TimerDisable(base_, source_);

    // Enable peripheral except in deep sleep modes (e.g. LPM1, LPM2, LPM3)
    SysCtrlPeripheralEnable(peripheral_);
    SysCtrlPeripheralSleepEnable(peripheral_);
    SysCtrlPeripheralDeepSleepDisable(peripheral_);

    // Configure the peripheral
    TimerConfigure(base_, config_);

    // Set the frequency
    TimerLoadSet(base_, source_, frequency_);
}

void Timer::start(void)
{
    TimerEnable(base_, source_);
}

void Timer::stop(void)
{
    TimerDisable(base_, source_);
}

uint32_t Timer::read(void)
{
    return TimerValueGet(base_, source_);
}

void Timer::setCallback(Callback* callback)
{
    callback_ = callback;
}

void Timer::clearCallback(void)
{
    callback_ = nullptr;
}

void Timer::enableInterrupts(void)
{
    InterruptHandler::getInstance().setInterruptHandler(this);

    // Enable Timer interrupts
    TimerIntEnable(base_, interrupt_mode_);

    // Set the Timer interrupt priority
    // IntPrioritySet(interrupt_, (7 << 5));

    // Enable the Timer interrupt
    IntEnable(interrupt_);
}

void Timer::disableInterrupts(void)
{
    // Diisable Timer interrupts
    TimerIntDisable(base_, interrupt_mode_);

    // Disable the Timer interrupt
    IntDisable(interrupt_);
}

/*=============================== protected =================================*/

void Timer::interruptHandler(void)
{
    if (callback_ != nullptr)
    {
        callback_->execute();
    }
}

/*================================ private ==================================*/
