/**
 * @file       EthernetDevice.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "EthernetDevice.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

/*=============================== protected =================================*/

void EthernetDevice::setMacAddress(uint8_t* mac_address)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        macAddress[i] = mac_address[i];
    }
}

/*================================ private ==================================*/
