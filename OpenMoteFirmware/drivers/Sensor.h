/**
 * @file       Sensor.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef SENSOR_H_
#define SENSOR_H_

class Sensor {
public:
    virtual bool enable(void) = 0;
    virtual bool suspend(void) = 0;
    virtual bool wakeup(void) = 0;
    virtual bool reset(void) = 0;
    virtual bool isPresent(void) = 0;
};

#endif /* SENSOR_H_ */
