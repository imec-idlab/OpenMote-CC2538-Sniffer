/**
 * @file       SemaphoreBinary.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Semaphore.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

SemaphoreBinary::SemaphoreBinary(void)
{
    semaphore_ = xSemaphoreCreateBinary();
}

SemaphoreBinary::~SemaphoreBinary(void)
{
}

void SemaphoreBinary::take(void)
{
    xSemaphoreTake(semaphore_, portMAX_DELAY);
}

void SemaphoreBinary::take(uint32_t milliseconds)
{
    TickType_t timeout = milliseconds / portTICK_RATE_MS;
    xSemaphoreTake(semaphore_, timeout);
}

void SemaphoreBinary::give(void)
{
    xSemaphoreGive(semaphore_);
}

void SemaphoreBinary::giveFromInterrupt(void)
{
    priorityTaskWoken_ = pdFALSE;
    xSemaphoreGiveFromISR(semaphore_, &priorityTaskWoken_);
    portYIELD_FROM_ISR(priorityTaskWoken_);
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
