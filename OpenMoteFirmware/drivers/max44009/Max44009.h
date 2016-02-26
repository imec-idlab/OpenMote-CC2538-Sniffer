/**
 * @file       Max44009.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef MAX44009_H_
#define MAX44009_H_

#include <stdint.h>

#include "I2c.h"
#include "Gpio.h"

#include "Callback.h"
#include "Sensor.h"

class I2c;
class GpioIn;

class Max44009: public Sensor
{
public:
    Max44009(I2c& i2c, GpioIn& gpio);
    bool enable(void);
    bool suspend(void){return false;}
    bool wakeup(void){return false;}
    bool reset(void);
    bool isPresent(void);
    void setCallback(Callback* callback);
    void clearCallback(void);
    bool readLux(void);
    float getLux(void);
    uint16_t getLuxRaw(void);
private:
    I2c& i2c_;
    GpioIn& gpio_;

    uint8_t exponent;
    uint8_t mantissa;
};

#endif /* MAX44009_H_ */
