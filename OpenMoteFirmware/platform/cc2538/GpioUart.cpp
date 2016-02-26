/**
 * @file       GpioUart.cpp
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

GpioUart::GpioUart(uint32_t port, uint8_t pin, uint32_t ioc):
    Gpio(port, pin), ioc_(ioc)
{
}

uint32_t GpioUart::getIoc(void)
{
    return ioc_;
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
