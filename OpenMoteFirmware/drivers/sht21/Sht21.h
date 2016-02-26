/**
 * @file       Sht21.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef SHT21_H_
#define SHT21_H_

#include <stdint.h>

#include "I2c.h"

#include "Sensor.h"

class I2c;

class Sht21: public Sensor
{
public:
    Sht21(I2c& i2c);
    bool enable(void);
    bool suspend(void){return false;}
    bool wakeup(void){return false;}
    bool reset(void);
    bool isPresent(void);
    bool readTemperature(void);
    bool readHumidity(void);
    float getTemperature(void);
    uint16_t getTemperatureRaw(void);
    float getHumidity(void);
    uint16_t getHumidityRaw(void);
private:
    void isInitialized(void);
private:
    I2c& i2c_;

    uint16_t temperature;
    uint16_t humidity;
};

#endif /* SHT21_H_ */
