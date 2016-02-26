/**
 * @file       GpioIn.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Gpio.h"
#include "InterruptHandler.h"

#include "cc2538_include.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

GpioIn::GpioIn(uint32_t port, uint8_t pin, uint32_t edge):
    Gpio(port, pin), edge_(edge)
{
    // Set the pin as input
    GPIOPinTypeGPIOInput(port_, pin_);

    // Set the edge of the interrupt
    GPIOIntTypeSet(port_, pin_, edge_);
}

bool GpioIn::read(void)
{
    uint32_t state;
    state = GPIOPinRead(port_, pin_);
    return (bool)state;
}

void GpioIn::setCallback(Callback* callback)
{
    // Save the pointer to the callback function
    callback_ = callback;

    // Get a reference to the interruptHandler object
    InterruptHandler& interruptHandler = InterruptHandler::getInstance();

    // Register to the interruptHandler by passing a pointer to the object
    interruptHandler.setInterruptHandler(this);
}

void GpioIn::clearCallback(void)
{
    // Clear the pointer to the callback function
    callback_ = nullptr;
}

void GpioIn::enableInterrupts(void)
{
    // Clear the interrupt
    GPIOPinIntClear(port_, pin_);

    // Enable the interrupt
    GPIOPinIntEnable(port_, pin_);
}

void GpioIn::disableInterrupts(void)
{
    // Disable the interrupt
    GPIOPinIntDisable(port_, pin_);
}

void GpioIn::interruptHandler(void)
{
    // Call the interrupt handler if it is NOT null
    if (callback_ != nullptr) {
        callback_->execute();
    }
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
