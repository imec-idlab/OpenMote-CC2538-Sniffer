/**
 * @file       Mutex.cpp
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

Mutex::Mutex()
{
    mutex_ = xSemaphoreCreateMutex();
}

Mutex::~Mutex()
{
    vSemaphoreDelete(mutex_);
}

bool Mutex::take(void)
{
    bool status = (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE);
    return status;
}

bool Mutex::take(uint32_t milliseconds)
{
    TickType_t timeout = milliseconds / portTICK_RATE_MS;
    bool status = (xSemaphoreTake(mutex_, timeout) == pdTRUE);
    return status;
}

void Mutex::give(void)
{
    xSemaphoreGive(mutex_);
}

void Mutex::giveFromInterrupt(void)
{
    priorityTaskWoken_ = pdFALSE;
    xSemaphoreGiveFromISR(mutex_, &priorityTaskWoken_);
    portYIELD_FROM_ISR(priorityTaskWoken_);
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
