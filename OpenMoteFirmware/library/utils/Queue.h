/**
 * @file       Queue.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */


#ifndef QUEUE_H_
#define QUEUE_H_

#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"

class Queue
{
public:
    Queue(uint8_t* buffer, uint32_t length);
    void reset(void);
    int32_t getSize(void);
    int32_t getOccupied(void);
    int32_t getRemaining(void);
    int32_t isEmpty(void);
    int32_t isFull(void);
    int32_t read(uint8_t* data);
    int32_t read(uint8_t* buffer, uint32_t length);
    int32_t write(uint8_t data);
    int32_t write(const uint8_t* data, uint32_t length);
private:
    void empty(void);
private:
    SemaphoreHandle_t mutex_;
    uint8_t* buffer_;
    int32_t  length_;
    uint8_t* read_;
    uint8_t* write_;
};

#endif /* CIRCULAR_BUFFER_H_ */
