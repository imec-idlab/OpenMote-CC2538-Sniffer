/**
 * @file       Max44009.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Max44009.h"

/*================================ define ===================================*/

/* ADDRESS AND NOT_FOUND VALUE */
#define MAX44009_ADDRESS                    ( 0x4A )

/* REGISTER ADDRESSES */
#define MAX44009_INT_STATUS_ADDR            ( 0x00 )    // R
#define MAX44009_INT_ENABLE_ADDR            ( 0x01 )    // R/W
#define MAX44009_CONFIG_ADDR                ( 0x02 )    // R/W
#define MAX44009_LUX_HIGH_ADDR              ( 0x03 )    // R
#define MAX44009_LUX_LOW_ADDR               ( 0x04 )    // R
#define MAX44009_THR_HIGH_ADDR              ( 0x05 )    // R/W
#define MAX44009_THR_LOW_ADDR               ( 0x06 )    // R/W
#define MAX44009_THR_TIMER_ADDR             ( 0x07 )    // R/W

/* INTERRUPT VALUES */
#define MAX44009_INT_STATUS_OFF             ( 0x00 )
#define MAX44009_INT_STATUS_ON              ( 0x01 )
#define MAX44009_INT_DISABLED               ( 0x00 )
#define MAX44009_INT_ENABLED                ( 0x01 )

/* CONFIGURATION VALUES */
#define MAX44009_CONFIG_DEFAULT             ( 0 << 7 )
#define MAX44009_CONFIG_CONTINUOUS          ( 1 << 7 )
#define MAX44009_CONFIG_AUTO                ( 0 << 6 )
#define MAX44009_CONFIG_MANUAL              ( 1 << 6 )
#define MAX44009_CONFIG_CDR_NORMAL          ( 0 << 3 )
#define MAX44009_CONFIG_CDR_DIVIDED         ( 1 << 3 )
#define MAX44009_CONFIG_INTEGRATION_800ms   ( 0 << 0 )
#define MAX44009_CONFIG_INTEGRATION_400ms   ( 1 << 0 )
#define MAX44009_CONFIG_INTEGRATION_200ms   ( 2 << 0 )
#define MAX44009_CONFIG_INTEGRATION_100ms   ( 3 << 0 )
#define MAX44009_CONFIG_INTEGRATION_50ms    ( 4 << 0 )
#define MAX44009_CONFIG_INTEGRATION_25ms    ( 5 << 0 )
#define MAX44009_CONFIG_INTEGRATION_12ms    ( 6 << 0 )
#define MAX44009_CONFIG_INTEGRATION_6ms     ( 7 << 0 )

/* DEFAULT CONFIGURATION */
#define MAX44009_DEFAULT_CONFIGURATION      ( MAX44009_CONFIG_DEFAULT | \
                                              MAX44009_CONFIG_AUTO | \
                                              MAX44009_CONFIG_CDR_NORMAL | \
                                              MAX44009_CONFIG_INTEGRATION_100ms )

/* USER CONFIGURATION */
#define MAX44009_USER_CONFIGURATION         ( MAX44009_CONFIG_DEFAULT | \
                                              MAX44009_CONFIG_MANUAL | \
                                              MAX44009_CONFIG_CDR_NORMAL | \
                                              MAX44009_CONFIG_INTEGRATION_100ms )

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Max44009::Max44009(I2c& i2c, GpioIn& gpio):
    i2c_(i2c), gpio_(gpio)
{
}

bool Max44009::enable(void)
{
    uint8_t max44009_address[5] = {MAX44009_INT_ENABLE_ADDR, MAX44009_CONFIG_ADDR, \
                                   MAX44009_THR_HIGH_ADDR, MAX44009_THR_LOW_ADDR, \
                                   MAX44009_THR_TIMER_ADDR};
    uint8_t max44009_value[5]   = {MAX44009_INT_STATUS_ON, MAX44009_USER_CONFIGURATION, \
                                   0xFF, 0x00, 0xFF};
    uint8_t max44009_data[2];
    bool status;

    // Lock access to I2C
    i2c_.lock();

    // Iterate for all addresses and value
    for (uint8_t i = 0; i < sizeof(max44009_address); i++)
    {
        // Prepare the data vector
        max44009_data[0] = max44009_address[i];
        max44009_data[1] = max44009_value[i];

        // Write the configuration value in each register
        status = i2c_.writeByte(MAX44009_ADDRESS, max44009_data, sizeof(max44009_data));
        if (status == false) goto error;
    }

    // Release access to I2C
    i2c_.unlock();

    return true;

error:
    // Free access to I2C
    i2c_.unlock();
    return false;
}

bool Max44009::reset(void)
{
    uint8_t max44009_address[5] = {MAX44009_INT_ENABLE_ADDR, MAX44009_CONFIG_ADDR, \
                                   MAX44009_THR_HIGH_ADDR, MAX44009_THR_LOW_ADDR, \
                                   MAX44009_THR_TIMER_ADDR};
    uint8_t max44009_value[5]   = {MAX44009_INT_STATUS_OFF, MAX44009_DEFAULT_CONFIGURATION, \
                                   0xFF, 0x00, 0xFF};
    uint8_t max44009_data[2];
    bool status;

    // Lock access to I2C
    i2c_.lock();

    // Iterate for all addresses and value
    for (uint8_t i = 0; i < sizeof(max44009_address); i++)
    {
        // Prepare the data vector
        max44009_data[0] = max44009_address[i];
        max44009_data[1] = max44009_value[i];

        // Write the configuration value in each register
        status = i2c_.writeByte(MAX44009_ADDRESS, max44009_data, sizeof(max44009_data));
        if (status == false) goto error;
    }

    // Release access to I2C
    i2c_.unlock();

    return true;

error:
    // Release access to I2C
    i2c_.unlock();
    return false;
}

void Max44009::setCallback(Callback* callback)
{
    gpio_.setCallback(callback);
    gpio_.enableInterrupts();
}

void Max44009::clearCallback(void)
{
    gpio_.clearCallback();
    gpio_.disableInterrupts();
}

bool Max44009::isPresent(void)
{
    uint8_t isPresent;
    bool status;

    // Lock access to I2C
    i2c_.lock();

    // Write the configuration address
    status = i2c_.writeByte(MAX44009_ADDRESS, MAX44009_CONFIG_ADDR);
    if (status == false) goto error;

    // Read the configuration register
    status = i2c_.readByte(MAX44009_ADDRESS, &isPresent);
    if (status == false) goto error;

    // Release access to I2C
    i2c_.unlock();

    // Return true if sensor is present
    return (isPresent == MAX44009_DEFAULT_CONFIGURATION ||
            isPresent == MAX44009_USER_CONFIGURATION);
error:
    // Release access to I2C
    i2c_.unlock();
    return false;
}

bool Max44009::readLux(void)
{
    uint8_t max44009_data[2];
    bool status;

    // Lock access to I2C
    i2c_.lock();

    // Write the high data register address
    status = i2c_.writeByte(MAX44009_ADDRESS, MAX44009_LUX_HIGH_ADDR);
    if (status == false) goto error;

    // Read the high data register value
    status = i2c_.readByte(MAX44009_ADDRESS, &max44009_data[0]);
    if (status == false) goto error;

    // Write the low data register address
    status = i2c_.writeByte(MAX44009_ADDRESS, MAX44009_LUX_LOW_ADDR);
    if (status == false) goto error;

    // Read the low data register address
    status = i2c_.readByte(MAX44009_ADDRESS, &max44009_data[1]);
    if (status == false) goto error;

    // Release access to I2C
    i2c_.unlock();

    // Convert MAX44009 exponent
    exponent = ((max44009_data[0] >> 4) & 0x0F);
    exponent = (exponent == 0x0F ? exponent & 0x0E : exponent);

    // Convert MAX44009 mantissa
    mantissa = ((max44009_data[0] & 0x0F) << 4) | (max44009_data[1] & 0x0F);

    return true;

error:
    // Release access to I2C
    i2c_.unlock();
    return false;
}

float Max44009::getLux(void)
{
    float lux = 0.045;
    lux *= 2^exponent * mantissa;
    return lux;
}

uint16_t Max44009::getLuxRaw(void)
{
    uint16_t lux;
    lux = ((uint16_t)exponent << 8) | ((uint16_t) mantissa);
    return lux;
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
