/**
 * @file       Watchdog.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Watchdog.h"

#include "cc2538_include.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Watchdog::Watchdog(uint32_t interval):
    interval_(interval)
{
}

void Watchdog::init(void)
{
    WatchdogEnable(interval_);
}

void Watchdog::walk(void)
{
    WatchdogClear();
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
