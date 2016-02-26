/**
 * @file       EthernetFrame.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "EthernetFrame.h"

/*================================ define ===================================*/

static const uint32_t MAX_ETHERNET_LENGTH = 1518;

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

EthernetFrame::EthernetFrame(uint8_t* buffer, uint32_t length):
    buffer_ptr(buffer), buffer_len(length),
    destination(&buffer[0]), source(&buffer[6]), type((uint16_t*) &buffer[12]), payload(&buffer[14])
{
}

void EthernetFrame::reset(void)
{
    buffer_len = 1518;
}

uint8_t* EthernetFrame::getBuffer(void)
{
    return buffer_ptr;
}

uint32_t EthernetFrame::getLength(void)
{
    return buffer_len;
}

void EthernetFrame::setLength(uint32_t length)
{
    buffer_len = length;
}

bool EthernetFrame::isValid()
{
    return false;
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
