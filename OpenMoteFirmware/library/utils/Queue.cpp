/**
 * @file       Queue.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Queue.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Queue::Queue(uint8_t* buffer, uint32_t length):
    buffer_(buffer), length_(length),
    read_(buffer), write_(buffer)
{
    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == NULL) {
        while(true);
    }
}

void Queue::reset(void)
{
    // Try to acquire the mutex
    if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE)
    {
        empty();

        read_  = buffer_;
        write_ = buffer_;

        xSemaphoreGive(mutex_);
    }
}

int32_t Queue::isEmpty(void)
{
    int32_t scratch;

    // Try to acquire the mutex
    if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE)
    {
        scratch = (read_ == write_);

        xSemaphoreGive(mutex_);

        return scratch;
    }
    else
    {
        return -1;
    }
}

int32_t Queue::isFull(void)
{
    int32_t scratch;

    // Try to acquire the mutex
    if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE)
    {
        scratch = (write_ == (buffer_ + length_));

        xSemaphoreGive(mutex_);

        return scratch;
    }
    else
    {
        return -1;
    }
}

int32_t Queue::getSize(void)
{
    int32_t scratch;

    // Try to acquire the mutex
    if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE)
    {
        scratch = length_;

        xSemaphoreGive(mutex_);

        return scratch;
    }
    else
    {
        return -1;
    }
}

int32_t Queue::getOccupied(void)
{
    int32_t scratch;

    // Try to acquire the mutex
    if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE)
    {
        scratch = (write_ - buffer_);

        xSemaphoreGive(mutex_);

        return scratch;
    }
    else
    {
        return -1;
    }
}

int32_t Queue::getRemaining(void)
{
    int32_t scratch;

    // Try to acquire the mutex
    if (xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE)
    {
        scratch = (buffer_ + length_ - write_);

        xSemaphoreGive(mutex_);

        return scratch;
    }
    else
    {
        return -1;
    }
}

int32_t Queue::read(uint8_t* data)
{
    if (!isEmpty())
    {
        *data = *read_++;
        return 0;
    }

    return -1;
}

int32_t Queue::read(uint8_t* buffer, uint32_t length)
{
    while (length--)
    {
        if (read(buffer++) != 0)
        {
            return -1;
        }
    }

    return 0;
}

int32_t Queue::write(uint8_t data)
{
    if (!isFull())
    {
        *write_++ = data;
        return 0;
    }

    return -1;
}

int32_t Queue::write(const uint8_t *data, uint32_t length)
{
    // For each byte
    while (length--)
    {
        // Try to write the byte
        if (write(*data++) != 0)
        {
            // Return error
            return -1;
        }
    }

    // Return success
    return 0;
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

void Queue::empty(void)
{
    uint8_t* tmp = buffer_;
    while(tmp != (buffer_ + length_)) {
        *tmp++ = 0;
    }
}
