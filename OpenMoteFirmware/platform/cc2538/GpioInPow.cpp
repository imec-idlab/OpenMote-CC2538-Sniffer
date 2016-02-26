/**
 * @file       GpioInPow.cpp
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

GpioInPow::GpioInPow(uint32_t port, uint8_t pin, uint32_t edge):
    GpioIn(port, pin, edge)
{
    // Set the pin as input
    GPIOPinTypeGPIOInput(port_, pin_);

    // Set the edge of the interrupt
    GPIOPowIntTypeSet(port_, pin_, edge_);

    // Enable the interrupt wakeup capability of the port
    if(port_ == GPIO_A_BASE)
    {
        GPIOIntWakeupEnable(GPIO_IWE_PORT_A);
    }
    else if(port_ == GPIO_B_BASE)
    {
        GPIOIntWakeupEnable(GPIO_IWE_PORT_B);
    }
    else if(port_ == GPIO_C_BASE)
    {
        GPIOIntWakeupEnable(GPIO_IWE_PORT_C);
    }
    else if(port_ == GPIO_D_BASE)
    {
        GPIOIntWakeupEnable(GPIO_IWE_PORT_D);
    }
}

void GpioInPow::enableInterrupts(void)
{
    // Clear the power interrupt
    GPIOPowIntClear(port_, pin_);

    // Enable the power intrrupt
    GPIOPowIntEnable(port_, pin_);
}

void GpioInPow::disableInterrupts(void)
{
    // Enable the power intrrupt
    GPIOPowIntDisable(port_, pin_);
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
