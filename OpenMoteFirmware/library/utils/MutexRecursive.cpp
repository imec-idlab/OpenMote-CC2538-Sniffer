/**
 * @file       MutexRecursive.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Mutex.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

MutexRecursive::MutexRecursive():
    Mutex()
{
    mutex_ = xSemaphoreCreateRecursiveMutex();
}

MutexRecursive::~MutexRecursive()
{
    vSemaphoreDelete(mutex_);
}

bool MutexRecursive::take(void)
{
    return false;
}

bool MutexRecursive::take(uint32_t milliseconds)
{
    return false;
}

void MutexRecursive::give(void)
{
}

void MutexRecursive::giveFromInterrupt(void)
{
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
