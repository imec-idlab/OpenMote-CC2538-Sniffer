/**
 * @file       Adxl346.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Adxl346.h"

/*================================ define ===================================*/

/* ADDRESS AND IDENTIFIER */
#define ADXL346_ADDRESS                     ( 0x53 )
#define ADXL346_DEVID_VALUE                 ( 0xE6 )

/* REGISTER ADDRESSES */
#define ADXL346_DEVID_ADDR                  ( 0x00 )
#define ADXL346_THRES_TAP_ADDR              ( 0x1D )
#define ADXL346_OFSX_ADDR                   ( 0x1E )
#define ADXL346_OFSY_ADDR                   ( 0x1F )
#define ADXL346_OFSZ_ADDR                   ( 0x20 )
#define ADXL346_DUR_ADDR                    ( 0x21 )
#define ADXL346_LATENT_ADDR                 ( 0x22 )
#define ADXL346_WINDOW_ADDR                 ( 0x23 )
#define ADXL346_THRESH_ACT_ADDR             ( 0x24 )
#define ADXL346_THRESH_INACT_ADDR           ( 0x25 )
#define ADXL346_TIME_INACT_ADDR             ( 0x26 )
#define ADXL346_ACT_INACT_CTL_ADDR          ( 0x27 )
#define ADXL346_THRESH_FF_ADDR              ( 0x28 )
#define ADXL346_TIME_FF_ADDR                ( 0x29 )
#define ADXL346_TAP_AXES_ADDR               ( 0x2A )
#define ADXL346_ACT_TAP_STATUS_ADDR         ( 0x2B )
#define ADXL346_BW_RATE_ADDR                ( 0x2C )
#define ADXL346_POWER_CTL_ADDR              ( 0x2D )
#define ADXL346_INT_ENABLE_ADDR             ( 0x2E )
#define ADXL346_INT_MAP_ADDR                ( 0x2F )
#define ADXL346_INT_SOURCE_ADDR             ( 0x30 )
#define ADXL346_DATA_FORMAT_ADDR            ( 0x31 )
#define ADXL346_DATAX0_ADDR                 ( 0x32 )
#define ADXL346_DATAX1_ADDR                 ( 0x33 )
#define ADXL346_DATAY0_ADDR                 ( 0x34 )
#define ADXL346_DATAY1_ADDR                 ( 0x35 )
#define ADXL346_DATAZ0_ADDR                 ( 0x36 )
#define ADXL346_DATAZ1_ADDR                 ( 0x37 )
#define ADXL346_FIFO_CTL_ADDR               ( 0x38 )
#define ADXL346_FIFO_STATUS_ADDR            ( 0x39 )
#define ADXL346_TAP_SIGN_ADDR               ( 0x3A )
#define ADXL346_ORIENT_CONF_ADDR            ( 0x3B )
#define ADXL346_ORIENT_ADDR                 ( 0x3C )

/* INT_ENABLE/INT_MAP/INT_SOURCE */
#define ADXL346_INT_ENABLE_DATA_READY      ( 1 << 7 )
#define ADXL346_INT_ENABLE_SINGLE_TAP      ( 1 << 6 )
#define ADXL346_INT_ENABLE_DOUBLE_TAP      ( 1 << 5 )
#define ADXL346_INT_ENABLE_ACTIVITY        ( 1 << 4 )
#define ADXL346_INT_ENABLE_INACTIVITY      ( 1 << 3 )
#define ADXL346_INT_ENABLE_FREE_FALL       ( 1 << 2 )
#define ADXL346_INT_ENABLE_WATERMARK       ( 1 << 1 )
#define ADXL346_INT_ENABLE_OVERRUN         ( 1 << 0 )

/* ACT_INACT_CONTROL */
#define ADXL346_ACT_INACT_CTL_ACT_ACDC     ( 1 << 7 )
#define ADXL346_ACT_INACT_CTL_ACT_X_EN     ( 1 << 6 )
#define ADXL346_ACT_INACT_CTL_ACT_Y_EN     ( 1 << 5 )
#define ADXL346_ACT_INACT_CTL_ACT_Z_EN     ( 1 << 4 )
#define ADXL346_ACT_INACT_CTL_INACT_ACDC   ( 1 << 3 )
#define ADXL346_ACT_INACT_CTL_INACT_X_EN   ( 1 << 2 )
#define ADXL346_ACT_INACT_CTL_INACT_Y_EN   ( 1 << 1 )
#define ADXL346_ACT_INACT_CTL_INACT_Z_EN   ( 1 << 0 )

/* TAP_AXES */
#define ADXL346_TAP_AXES_SUPPRESS           ( 1 << 3 )
#define ADXL346_TAP_AXES_TAP_X_EN           ( 1 << 2 )
#define ADXL346_TAP_AXES_TAP_Y_EN           ( 1 << 1 )
#define ADXL346_TAP_AXES_TAP_Z_EN           ( 1 << 0 )

/* ACT_TAP_STATUS */
#define ADXL346_ACT_TAP_STATUS_ACT_X_SRC    ( 1 << 6 )
#define ADXL346_ACT_TAP_STATUS_ACT_Y_SRC    ( 1 << 5 )
#define ADXL346_ACT_TAP_STATUS_ACT_Z_SRC    ( 1 << 4 )
#define ADXL346_ACT_TAP_STATUS_ASLEEP       ( 1 << 3 )
#define ADXL346_ACT_TAP_STATUS_TAP_X_SRC    ( 1 << 2 )
#define ADXL346_ACT_TAP_STATUS_TAP_Y_SRC    ( 1 << 1 )
#define ADXL346_ACT_TAP_STATUS_TAP_Z_SRC    ( 1 << 0 )

/* BW_RATE */
#define ADXL346_BW_RATE_LOW_POWER           ( 1 << 4 )
#define ADXL346_BW_RATE_RATE(x)             ( (x) & 0x0F)

/* POWER CONTROL */
#define ADXL346_POWER_CTL_LINK              ( 1 << 5 )
#define ADXL346_POWER_CTL_AUTO_SLEEP        ( 1 << 4 )
#define ADXL346_POWER_CTL_MEASURE           ( 1 << 3 )
#define ADXL346_POWER_CTL_SLEEP             ( 1 << 2 )
#define ADXL346_POWER_CTL_WAKEUP(x)         ( (x) & 0x03 )

/* DATA_FORMAT */
#define ADXL346_DATA_FORMAT_SELF_TEST       ( 1 << 7 )
#define ADXL346_DATA_FORMAT_SPI             ( 1 << 6 )
#define ADXL346_DATA_FORMAT_INT_INVERT      ( 1 << 5 )
#define ADXL346_DATA_FORMAT_FULL_RES        ( 1 << 3 )
#define ADXL346_DATA_FORMAT_JUSTIFY         ( 1 << 2 )
#define ADXL346_DATA_FORMAT_RANGE_PM_2g     ( 0 )
#define ADXL346_DATA_FORMAT_RANGE_PM_4g     ( 1 )
#define ADXL346_DATA_FORMAT_RANGE_PM_8g     ( 2 )
#define ADXL346_DATA_FORMAT_RANGE_PM_16g    ( 3 )

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Adxl346::Adxl346(I2c& i2c, GpioIn& gpio):
    i2c_(i2c), gpio_(gpio)
{
}

bool Adxl346::enable(void)
{
    uint8_t config[2];
    bool status;

    // Lock access to I2C
    i2c_.lock();

    // Write the bandwidth register
    // Low-power mode, 25 Hz, 27 uA
    config[0] = ADXL346_BW_RATE_ADDR;
    config[1] = (ADXL346_BW_RATE_LOW_POWER | ADXL346_BW_RATE_RATE(8));
    status = i2c_.writeByte(ADXL346_ADDRESS, config, sizeof(config));
    if (status == false) goto error;

    // Write the format register
    // +/-16g range, 4 mg/LSB
    config[0] = ADXL346_DATA_FORMAT_ADDR;
    config[1] = (ADXL346_DATA_FORMAT_FULL_RES | ADXL346_DATA_FORMAT_RANGE_PM_16g);
    status = i2c_.writeByte(ADXL346_ADDRESS, config, sizeof(config));
    if (status == false) goto error;

    // Write the power register
    config[0] = ADXL346_POWER_CTL_ADDR;
    config[1] |= ADXL346_POWER_CTL_MEASURE;
    status = i2c_.writeByte(ADXL346_ADDRESS, config, sizeof(config));
    if (status == false) goto error;

    // Release access to I2C
    i2c_.unlock();

    return true;

error:
    // Release access to I2C
    i2c_.unlock();
    return false;
}

bool Adxl346::suspend(void)
{
    uint8_t config[2];
    bool status;

    // Lock access to I2C
    i2c_.lock();

    // Read the power control register
    status = i2c_.readByte(ADXL346_POWER_CTL_ADDR, &config[1]);
    if (status == false) goto error;

    // Write the power control register (clear measure bit and set sleep bit)
    config[0]  = ADXL346_POWER_CTL_ADDR;
    config[1] &= ~ADXL346_POWER_CTL_MEASURE;
    config[1] |= ADXL346_POWER_CTL_SLEEP;
    status = i2c_.writeByte(ADXL346_ADDRESS, config, sizeof(config));
    if (status == false) goto error;

    // Release access to I2C
    i2c_.unlock();

    return true;

error:
    // Release access to I2C
    i2c_.unlock();
    return false;
}

bool Adxl346::wakeup(void)
{
    uint8_t config[2];
    bool status;

    // Lock access to I2C
    i2c_.lock();

    // Read the power control register
    status = i2c_.readByte(ADXL346_POWER_CTL_ADDR, &config[1]);
    if (status == false) goto error;

    // Write the power control register (clear sleep bit)
    config[0]  = ADXL346_POWER_CTL_ADDR;
    config[1] &= ~ADXL346_POWER_CTL_SLEEP;
    status = i2c_.writeByte(ADXL346_ADDRESS, config, sizeof(config));
    if (status == false) goto error;

    // Write the power control register (set measure bit)
    config[0]  = ADXL346_POWER_CTL_ADDR;
    config[1] |= ADXL346_POWER_CTL_MEASURE;
    status = i2c_.writeByte(ADXL346_ADDRESS, config, sizeof(config));
    if (status == false) goto error;

    // Release access to I2C
    i2c_.unlock();

    return true;

error:
    // Release access to I2C
    i2c_.unlock();
    return false;
}

bool Adxl346::reset(void)
{
    return false;
}

bool Adxl346::isPresent(void)
{
    uint8_t isPresent;
    bool status;

    // Lock access to I2C
    i2c_.lock();

    // Write the device identification address
    status = i2c_.writeByte(ADXL346_ADDRESS, ADXL346_DEVID_ADDR);
    if (status == false) goto error;

    // Read the device identification value
    status = i2c_.readByte(ADXL346_ADDRESS, &isPresent);
    if (status == false) goto error;

    // Release access to I2C
    i2c_.unlock();

    // Return true if sensor is present
    return (isPresent == ADXL346_DEVID_VALUE);

error:
    // Release access to I2C
    i2c_.unlock();
    return false;
}

bool Adxl346::selfTest(bool test)
{
    uint8_t config[2];
    bool status;

    // Lock access to I2C
    i2c_.lock();

    // Read the data format register
    status = i2c_.readByte(ADXL346_DATA_FORMAT_ADDR, &config[1]);
    if (status == false) goto error;

    // Write the data format register
    config[0]  = ADXL346_DATA_FORMAT_ADDR;
    if (test) config[1] |= ADXL346_DATA_FORMAT_SELF_TEST;
    else      config[1] &= ~ADXL346_DATA_FORMAT_SELF_TEST;

    status = i2c_.writeByte(ADXL346_ADDRESS, config, sizeof(config));
    if (status == false) goto error;

    return true;

error:
    // Release access to I2C
    i2c_.unlock();
    return false;
}

void Adxl346::setCallback(Callback* callback)
{
    gpio_.setCallback(callback);
    gpio_.enableInterrupts();
}

void Adxl346::clearCallback(void)
{
    gpio_.clearCallback();
    gpio_.disableInterrupts();
}

bool Adxl346::readSample(uint16_t* x, uint16_t* y, uint16_t* z)
{
    uint16_t acceleration[3];
    uint8_t  address[6] = {ADXL346_DATAX0_ADDR, ADXL346_DATAX1_ADDR,
                           ADXL346_DATAY0_ADDR, ADXL346_DATAY1_ADDR,
                           ADXL346_DATAZ0_ADDR, ADXL346_DATAZ1_ADDR};
    uint8_t scratch[2];
    bool status;

    // Iterate for all addresses, each direction has two addresses
    for (uint8_t i = 0; i < sizeof(address); i += 2)
    {

        // Lock access to I2C
        i2c_.lock();

        // I2C write register address
        status = i2c_.writeByte(ADXL346_ADDRESS, address[i + 0]);
        if (status == false) goto error;

        // I2C read acceleration value
        status = i2c_.readByte(ADXL346_ADDRESS, &scratch[0]);
        if (status == false) goto error;

        // Release access to I2C
        i2c_.unlock();

        // Lock access to I2C
        i2c_.lock();

        // I2C write register address
        status = i2c_.writeByte(ADXL346_ADDRESS, address[i + 1]);
        if (status == false) goto error;

        // I2C read acceleration value
        status = i2c_.readByte(ADXL346_ADDRESS, &scratch[1]);
        if (status == false) goto error;

        // Release access to I2C
        i2c_.unlock();

        // Convert ADXL346 data
        acceleration[i>>1] = (scratch[1] << 8) | scratch[0];
    }

    // Update acceleration variables
    *x = acceleration[0];
    *y = acceleration[1];
    *z = acceleration[2];

    return true;

error:
    // Release access to I2C
    i2c_.unlock();
    return false;
}

float Adxl346::convertAcceleration(int16_t acceleration)
{
    float result = 4.0;
    result *= (acceleration & 0x9FFF);
    return result;
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
