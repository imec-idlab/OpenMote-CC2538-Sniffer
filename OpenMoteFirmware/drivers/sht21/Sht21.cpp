/**
 * @file       Sht21.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Sht21.h"

/*================================ define ===================================*/

#define SHT21_ADDRESS                   ( 0x40 )
#define SHT21_DELAY_MS                  ( 20 )

#define SHT21_USER_REG_READ             ( 0xE7 )
#define SHT21_USER_REG_WRITE            ( 0xE6 )
#define SHT21_USER_REG_RESERVED_BITS    ( 0x38 )

#define SHT21_RESOLUTION_12b_14b        ( (0 << 7) | (0 << 0) )
#define SHT21_RESOLUTION_8b_12b         ( (0 << 7) | (1 << 0) )
#define SHT21_RESOLUTION_10b_13b        ( (1 << 7) | (0 << 0) )
#define SHT21_RESOLUTION_11b_11b        ( (1 << 7) | (1 << 0) )
#define SHT21_BATTERY_ABOVE_2V25        ( 0 << 6 )
#define SHT21_BATTERY_BELOW_2V25        ( 1 << 6 )
#define SHT21_ONCHIP_HEATER_ENABLE      ( 1 << 2 )
#define SHT21_ONCHIP_HEATER_DISABLE     ( 0 << 2 )
#define SHT21_OTP_RELOAD_ENABLE         ( 0 << 1 )
#define SHT21_OTP_RELOAD_DISABLE        ( 1 << 1 )

#define SHT21_TEMPERATURE_HM_CMD        ( 0xE3 )
#define SHT21_HUMIDITY_HM_CMD           ( 0xE5 )
#define SHT21_TEMPERATURE_NHM_CMD       ( 0xF3 )
#define SHT21_HUMIDITY_NHM_CMD          ( 0xF5 )
#define SHT21_RESET_CMD                 ( 0xFE )

#define SHT21_STATUS_MASK               ( 0xFC )

#define SHT21_DEFAULT_CONFIG            ( SHT21_RESOLUTION_12b_14b | \
                                          SHT21_ONCHIP_HEATER_DISABLE | \
                                          SHT21_BATTERY_ABOVE_2V25 | \
                                          SHT21_OTP_RELOAD_DISABLE )

#define SHT21_USER_CONFIG               ( SHT21_RESOLUTION_8b_12b | \
                                          SHT21_ONCHIP_HEATER_DISABLE | \
                                          SHT21_BATTERY_ABOVE_2V25 | \
                                          SHT21_OTP_RELOAD_DISABLE )

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Sht21::Sht21(I2c& i2c):
    i2c_(i2c)
{
}

bool Sht21::enable(void)
{
    uint8_t config[2];
    bool status;

    // Check if the sensor has been initialized
    isInitialized();

    // Setup the configuration vector, the first position holds address
    // and the second position holds the actual configuration
    config[0] = SHT21_USER_REG_WRITE;
    config[1] = 0;

    // Obtain the mutex of the I2C driver
    i2c_.lock();

    // Write the configuration register address
    status = i2c_.writeByte(SHT21_ADDRESS, SHT21_USER_REG_READ);
    if (status == false) goto error;

    // Read the current configuration (see datasheet pag. 9, fig. 18)
    status = i2c_.readByte(SHT21_ADDRESS, &config[1]);
    if (status == false) goto error;

    // Clean all the configuration bits except those reserved
    config[1] &= SHT21_USER_REG_RESERVED_BITS;

    // Set the configuration bits without changing those reserved
    config[1] |= SHT21_USER_CONFIG;

    // Write the user configuration
    status = i2c_.writeByte(SHT21_ADDRESS, config, sizeof(config));
    if (status == false) goto error;

    // Release the mutex of the I2C driver
    i2c_.unlock();

    return true;

error:
    // Release the mutex of the I2C driver
    i2c_.unlock();
    return false;
}

bool Sht21::reset(void)
{
    bool status;

    // Obtain the mutex of the I2C driver
    i2c_.lock();

    // Send a soft-reset command (see datasheet pag. 9, fig. 17)
    status = i2c_.writeByte(SHT21_ADDRESS, SHT21_RESET_CMD);
    if (status == false) goto error;

    // Release the mutex of the I2C driver
    i2c_.unlock();

    return true;

error:
    // Release the mutex of the I2C driver
    i2c_.unlock();
    return false;
}

bool Sht21::isPresent(void)
{
    uint8_t isPresent;
    bool status;

    // Check if the sensor has been initialized
    isInitialized();

    // Obtain the mutex of the I2C driver
    i2c_.lock();

    //  Write the configuration register address
    status = i2c_.writeByte(SHT21_ADDRESS, SHT21_USER_REG_READ);
    if (status == false) goto error;

    // Read the current configuration
    status = i2c_.readByte(SHT21_ADDRESS, &isPresent);
    if (status == false) goto error;

    // Clear the reserved bits (see datasheet pag. 9, tab. 8)
    isPresent &= ~SHT21_USER_REG_RESERVED_BITS;

    // Release the mutex of the I2C driver
    i2c_.unlock();

    // Return true if sensor is present
    return (isPresent == SHT21_DEFAULT_CONFIG ||
            isPresent == SHT21_USER_CONFIG);

error:
    // Release the mutex of the I2C driver
    i2c_.unlock();
    return false;
}

bool Sht21::readTemperature(void)
{
    uint8_t sht21_temperature[2];
    bool status;

    // Obtain the mutex of the I2C driver
    i2c_.lock();

    // Write the read temperature command
    status = i2c_.writeByte(SHT21_ADDRESS, SHT21_TEMPERATURE_HM_CMD);
    if (status == false) goto error;

    // Read the current temperature (see datasheet pag. 8, fig. 15)
    status = i2c_.readByte(SHT21_ADDRESS, sht21_temperature, sizeof(sht21_temperature));
    if (status == false) goto error;

    // Release the mutex of the I2C driver
    i2c_.unlock();

    // Update the temperature
    temperature = (sht21_temperature[0] << 8) | (sht21_temperature[1] & SHT21_STATUS_MASK);

    return true;

error:
    // Release the mutex of the I2C driver
    i2c_.unlock();
    return false;
}

bool Sht21::readHumidity(void)
{
    uint8_t sht21_humidity[2];
    bool status;

    // Obtain the mutex of the I2C driver
    i2c_.lock();

    // Write the read humidity command
    status = i2c_.writeByte(SHT21_ADDRESS, SHT21_HUMIDITY_HM_CMD);
    if (status == false) goto error;

    // Read the current humidity (see datasheet  pag. 8, fig. 15)
    status = i2c_.readByte(SHT21_ADDRESS, sht21_humidity, sizeof(sht21_humidity));
    if (status == false) goto error;

    // Release the mutex of the I2C driver
    i2c_.unlock();

    // Update the humidity
    humidity = (sht21_humidity[0] << 8) | (sht21_humidity[1] & SHT21_STATUS_MASK);

    return true;

error:
    // Release the mutex of the I2C driver
    i2c_.unlock();
    return false;
}

float Sht21::getTemperature(void)
{
    float result;
    result = -46.85;
    result += 175.72 * temperature / 65536;
    return result;
}

uint16_t Sht21::getTemperatureRaw(void)
{
    return temperature;
}

uint16_t Sht21::getHumidityRaw(void)
{
    return humidity;
}

float Sht21::getHumidity(void)
{
    float result;
    result = -6.0;
    result += 125.0 * humidity / 65536;
    return result;
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

void Sht21::isInitialized(void)
{
    static bool isInitialized = false;

    // If sensor is not initialized
    if (!isInitialized)
    {
        // Wait until sensor is available
        vTaskDelay(SHT21_DELAY_MS / portTICK_RATE_MS);

        // The sensor is now initialized
        isInitialized = true;
    }
}
