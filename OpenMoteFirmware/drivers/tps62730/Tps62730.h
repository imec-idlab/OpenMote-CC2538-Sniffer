/**
 * @file       Tps62730.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef TPS62730_H_
#define TPS62730_H_

#include <stdint.h>

#include "Gpio.h"

class Tps62730
{

public:
    Tps62730(GpioOut& bypass, GpioIn& status);
    void setOn(void);
    void setBypass(void);
    bool getStatus(void);
private:
    GpioOut& bypass_;
    GpioIn& status_;
};

#endif /* ADXL346_H_ */
