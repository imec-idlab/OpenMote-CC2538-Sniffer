/**
 * @file       GpioOut.cpp
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

#include "cc2538_include.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

GpioOut::GpioOut(uint32_t port, uint8_t pin):
    Gpio(port, pin)
{
    // Set the pin as output
    GPIOPinTypeGPIOOutput(port_, pin_);

    // Set the pin low
    GPIOPinWrite(port_, pin_, 0);
}

void GpioOut::on(void)
{
    // Set the pin high
    GPIOPinWrite(port_, pin_, pin_);
}

void GpioOut::off(void)
{
    // Set the pin low
    GPIOPinWrite(port_, pin_, 0);
}

void GpioOut::toggle(void)
{
    // Read the old status of the pin
    uint32_t status = GPIOPinRead(port_, pin_);

    // Change the status of the pin
    status = (~status) & pin_;

    // Set the new status of the pin
    GPIOPinWrite(port_, pin_, status);
}

uint32_t GpioOut::status(void)
{
    // Read the pin status
    uint32_t status = GPIOPinRead(port_, pin_);

    // Return the pin status
    return (status & pin_);
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
