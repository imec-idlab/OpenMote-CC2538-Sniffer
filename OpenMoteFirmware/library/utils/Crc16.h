/**
 * @file       Crc16.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef CRC16_H_
#define CRC16_H_

#include <stdint.h>

class Crc16
{
public:
    Crc16();
    void init(void);
    uint16_t get(void);
    void set(uint8_t byte);
    bool check(void);
private:
    uint16_t crc;
};

#endif /* CRC16_H_ */
