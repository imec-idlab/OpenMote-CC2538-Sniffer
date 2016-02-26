/**
 * @file       I2c.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Board.h"
#include "I2c.h"

#include "cc2538_include.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

extern Board board;

const uint32_t I2C_MAX_DELAY_US    = 100000;
const uint32_t I2C_MAX_DELAY_TICKS = I2C_MAX_DELAY_US / Board::BOARD_TICKS_PER_US;

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

I2c::I2c(uint32_t peripheral, GpioI2c& scl, GpioI2c& sda):
    peripheral_(peripheral), scl_(scl), sda_(sda)
{
}

void I2c::enable(uint32_t clock)
{
    bool status;

    // Update I2C clock
    clock_ = clock;

    // Enable peripheral except in deep sleep modes (e.g. LPM1, LPM2, LPM3)
    SysCtrlPeripheralEnable(peripheral_);
    SysCtrlPeripheralSleepEnable(peripheral_);
    SysCtrlPeripheralDeepSleepDisable(peripheral_);

    // Reset peripheral previous to configuring it
    SysCtrlPeripheralReset(peripheral_);

    // Configure the SCL pin
    GPIOPinTypeI2C(scl_.getPort(), scl_.getPin());
    IOCPinConfigPeriphInput(scl_.getPort(), scl_.getPin(), IOC_I2CMSSCL);
    IOCPinConfigPeriphOutput(scl_.getPort(), scl_.getPin(), IOC_MUX_OUT_SEL_I2C_CMSSCL);

    // Configure the SDA pin
    GPIOPinTypeI2C(sda_.getPort(), sda_.getPin());
    IOCPinConfigPeriphInput(sda_.getPort(), sda_.getPin(), IOC_I2CMSSDA);
    IOCPinConfigPeriphOutput(sda_.getPort(), sda_.getPin(), IOC_MUX_OUT_SEL_I2C_CMSSDA);

    // Configure the I2C clock
    status = ((clock_ == 100000) ? false : true);
    I2CMasterInitExpClk(SysCtrlClockGet(), status);

    // Enable the I2C module as master
    I2CMasterEnable();
}

void I2c::sleep(void)
{
    // Disable the I2C module
    I2CMasterDisable();

    // Configure GPIOs as output
    GPIOPinTypeGPIOOutput(scl_.getPort(), scl_.getPin());
    GPIOPinTypeGPIOOutput(sda_.getPort(), sda_.getPin());

    // Set GPIOs to low
    GPIOPinWrite(scl_.getPort(), scl_.getPin(), 0);
    GPIOPinWrite(sda_.getPort(), sda_.getPin(), 0);
}

void I2c::wakeup(void)
{
    // Reenable the I2C peripheral
    enable(clock_);
}

bool I2c::readByte(uint8_t address, uint8_t* buffer)
{
    uint32_t delayTicks = I2C_MAX_DELAY_TICKS;

    // Read operation
    I2CMasterSlaveAddrSet(address, true);

    // Single read operation
    I2CMasterControl(I2C_MASTER_CMD_SINGLE_RECEIVE);

    // Calculate timeout
    delayTicks += board.getCurrentTicks();

    // Wait until complete or timeout
    while (I2CMasterBusy())
    {
        // Check timeout status and return if expired
        if (board.isExpiredTicks(delayTicks)) return false;
    }

    // Read data from I2C
    *buffer = I2CMasterDataGet();

    return true;
}

bool I2c::readByte(uint8_t address, uint8_t* buffer, uint8_t size)
{
    uint32_t delayTicks = I2C_MAX_DELAY_TICKS;

    // Read operation
    I2CMasterSlaveAddrSet(address, true);

    // Burst read operation
    I2CMasterControl(I2C_MASTER_CMD_BURST_RECEIVE_START);

    // Calculate timeout
    delayTicks += board.getCurrentTicks();

    // Iterate overall all bytes
    while (size)
    {
        // Wait until complete or timeout
        while (I2CMasterBusy())
        {
            // Check timeout status and return if expired
            if (board.isExpiredTicks(delayTicks)) return false;
        }

        // Read data from I2C
        *buffer++ = I2CMasterDataGet();
        size--;

        // Check if it's the last byte
        if (size == 1) I2CMasterControl(I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        else           I2CMasterControl(I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    }

    return true;
}

bool I2c::writeByte(uint8_t address, uint8_t byte)
{
    uint32_t delayTicks = I2C_MAX_DELAY_TICKS;

    // Write operation
    I2CMasterSlaveAddrSet(address, false);

    // Write data to I2C
    I2CMasterDataPut(byte);

    // Single write operation
    I2CMasterControl(I2C_MASTER_CMD_SINGLE_SEND);

    // Calculate timeout
    delayTicks += board.getCurrentTicks();

    // Wait until complete or timeout
    while (I2CMasterBusy())
    {
        // Check timeout status and return if expired
        if (board.isExpiredTicks(delayTicks)) return false;
    }

    return true;
}

bool I2c::writeByte(uint8_t address, uint8_t* buffer, uint8_t size)
{
    uint32_t delayTicks = I2C_MAX_DELAY_TICKS;

    // Write operation
    I2CMasterSlaveAddrSet(address, false);

    // Write data to I2C
    I2CMasterDataPut(*buffer++);
    size--;

    // Burst write operation
    I2CMasterControl(I2C_MASTER_CMD_BURST_SEND_START);

    // Calculate timeout
    delayTicks += board.getCurrentTicks();

    // Wait until complete or timeout
    while (I2CMasterBusy())
    {
        // Check timeout status and return if expired
        if (board.isExpiredTicks(delayTicks)) return false;
    }

    while (size)
    {
        // Write data to I2C
        I2CMasterDataPut(*buffer++);
        size--;

        // Check if it's the last byte
        if (size == 0) I2CMasterControl(I2C_MASTER_CMD_BURST_SEND_FINISH);
        else           I2CMasterControl(I2C_MASTER_CMD_BURST_SEND_CONT);

        // Wait until complete or timeout
        while (I2CMasterBusy())
        {
            // Check timeout status and return if expired
            if (board.isExpiredTicks(delayTicks)) return false;
        }
    }

    return true;
}

/*=============================== protected =================================*/

void I2c::interruptHandler(void)
{
}

/*================================ private ==================================*/
