/**
 * @file       EthernetFrame.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef ETHERNET_FRAME_H_
#define ETHERNET_FRAME_H_

#include <stdint.h>

class EthernetFrame
{
public:
    EthernetFrame(uint8_t* buffer, uint32_t length);
    void reset(void);
    uint8_t* getBuffer(void);
    uint32_t getLength(void);
    void setLength(uint32_t length);
    bool isValid(void);
private:
    uint8_t* buffer_ptr;
    uint32_t buffer_len;

    uint8_t* destination;
    uint8_t* source;
    uint16_t* type;
    uint8_t* payload;
};

#endif /* ETHERNET_FRAME_H_ */
