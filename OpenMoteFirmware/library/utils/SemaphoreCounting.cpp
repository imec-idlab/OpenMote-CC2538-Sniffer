/**
 * @file       SemaphoreCounting.cpp
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

SemaphoreCounting::SemaphoreCounting(uint32_t initialCount, uint32_t maxCount)
{
    semaphore_ = xSemaphoreCreateCounting(maxCount, initialCount);
}

SemaphoreCounting::~SemaphoreCounting(void)
{
    vSemaphoreDelete(semaphore_);
}

void SemaphoreCounting::take(void)
{
    xSemaphoreTake(semaphore_, portMAX_DELAY);
}

void SemaphoreCounting::take(uint32_t milliseconds)
{
    TickType_t timeout = milliseconds / portTICK_RATE_MS;
    xSemaphoreTake(semaphore_, timeout);
}

void SemaphoreCounting::give(void)
{
    xSemaphoreGive(semaphore_);
}

void SemaphoreCounting::giveFromInterrupt(void)
{
    priorityTaskWoken_ = pdFALSE;
    xSemaphoreGiveFromISR(semaphore_, &priorityTaskWoken_);
    portYIELD_FROM_ISR(priorityTaskWoken_);
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
