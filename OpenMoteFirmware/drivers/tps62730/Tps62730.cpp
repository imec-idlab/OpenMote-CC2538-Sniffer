/**
 * @file       Tps62730.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Tps62730.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Tps62730::Tps62730(GpioOut& bypass, GpioIn& status):
    bypass_(bypass), status_(status)
{
}

void Tps62730::setOn(void)
{
    bypass_.on();
}

void Tps62730::setBypass(void)
{
    bypass_.off();
}

bool Tps62730::getStatus(void)
{
    bool status;
    status = status_.read();
    return status;
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
