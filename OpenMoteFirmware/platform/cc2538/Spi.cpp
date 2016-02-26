/**
 * @file       Spi.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Spi.h"
#include "InterruptHandler.h"

#include "cc2538_include.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Spi::Spi(uint32_t peripheral, uint32_t base, uint32_t clock, \
         GpioSpi& miso, GpioSpi& mosi, GpioSpi& clk, GpioSpi& ncs):
        peripheral_(peripheral), base_(base), clock_(clock), \
        miso_(miso), mosi_(mosi), clk_(clk), ncs_(ncs)
{
}

uint32_t Spi::getBase(void)
{
    return base_;
}

void Spi::enable(uint32_t mode, uint32_t protocol, uint32_t datawidth, uint32_t baudrate)
{
    // Store SPI mode, protoco, baudrate and datawidth
    mode_      = mode;
    protocol_  = protocol;
    baudrate_  = baudrate;
    datawidth_ = datawidth;

    // Enable peripheral except in deep sleep modes (e.g. LPM1, LPM2, LPM3)
    SysCtrlPeripheralEnable(peripheral_);
    SysCtrlPeripheralSleepEnable(peripheral_);
    SysCtrlPeripheralDeepSleepDisable(peripheral_);

    // Reset peripheral previous to configuring it
    SSIDisable(base_);

    // Set IO clock as SPI0 clock source
    SSIClockSourceSet(base_, clock_);

    // Configure the MISO, MOSI, CLK and nCS pins as peripheral
    IOCPinConfigPeriphInput(miso_.getPort(), miso_.getPin(), miso_.getIoc());
    IOCPinConfigPeriphOutput(mosi_.getPort(), mosi_.getPin(), mosi_.getIoc());
    IOCPinConfigPeriphOutput(clk_.getPort(), clk_.getPin(), clk_.getIoc());
    // IOCPinConfigPeriphOutput(ncs_.getPort(), ncs_.getPin(), ncs_.getIoc());

    // Configure MISO, MOSI, CLK and nCS GPIOs
    GPIOPinTypeSSI(miso_.getPort(), miso_.getPin());
    GPIOPinTypeSSI(mosi_.getPort(), mosi_.getPin());
    GPIOPinTypeSSI(clk_.getPort(), clk_.getPin());
    // GPIOPinTypeSSI(ncs_.getPort(), ncs_.getPin());

    // Configure the SPI0 clock
    SSIConfigSetExpClk(base_, SysCtrlIOClockGet(), protocol_, \
                       mode_, baudrate_, datawidth_);

    // Enable the SPI0 module
    SSIEnable(base_);
}

void Spi::sleep(void)
{
    SSIDisable(base_);

    // Configure the MISO, MOSI, CLK and nCS pins as output
    GPIOPinTypeGPIOOutput(miso_.getPort(), miso_.getPin());
    GPIOPinTypeGPIOOutput(mosi_.getPort(), mosi_.getPin());
    GPIOPinTypeGPIOOutput(clk_.getPort(), clk_.getPin());
    // GPIOPinTypeGPIOOutput(ncs_.getPort(), ncs_.getPin());

    //
    GPIOPinWrite(miso_.getPort(), miso_.getPin(), 0);
    GPIOPinWrite(mosi_.getPort(), mosi_.getPin(), 0);
    GPIOPinWrite(clk_.getPort(), clk_.getPin(), 0);
    // GPIOPinWrite(ncs_.getPort(), ncs_.getPin(), 0);

}

void Spi::wakeup(void)
{
    enable(mode_, protocol_, datawidth_, baudrate_);
}

void Spi::setRxCallback(Callback* callback)
{
    rx_callback_ = callback;
}

void Spi::setTxCallback(Callback* callback)
{
    tx_callback_ = callback;
}

void Spi::enableInterrupts(void)
{
    // Register the interrupt handler
    InterruptHandler::getInstance().setInterruptHandler(this);

    // Enable the SPI interrupt
    SSIIntEnable(base_, (SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR));

    // Enable the SPI interrupt
    IntEnable(interrupt_);
}

void Spi::disableInterrupts(void)
{
    // Disable the SPI interrupt
    SSIIntDisable(base_, (SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR));

    // Disable the SPI interrupt
    IntDisable(interrupt_);
}

void Spi::select(void)
{
    if (protocol_ == SSI_FRF_MOTO_MODE_0 ||
        protocol_ == SSI_FRF_MOTO_MODE_1)
    {
        ncs_.low();
    }
    else
    {
        ncs_.high();
    }
}

void Spi::deselect(void)
{
    if (protocol_ == SSI_FRF_MOTO_MODE_0 ||
        protocol_ == SSI_FRF_MOTO_MODE_1)
    {
        ncs_.high();
    }
    else
    {
        ncs_.low();
    }
}

uint8_t Spi::readByte(void)
{
    uint32_t byte;

    // Push a byte
    SSIDataPut(base_, 0x00);

    // Wait until it is complete
    while(SSIBusy(base_))
        ;

    // Read a byte
    SSIDataGet(base_, &byte);

    return (uint8_t)(byte & 0xFF);
}

uint32_t Spi::readByte(uint8_t* buffer, uint32_t length)
{
    uint32_t data;

    for (uint32_t i =  0; i < length; i++)
    {
        // Push a byte
        SSIDataPut(base_, 0x00);

        // Wait until it is complete
        while(SSIBusy(base_))
            ;

        // Read a byte
        SSIDataGet(base_, &data);

        *buffer++ = (uint8_t) data;
    }
    return 0;
}

void Spi::writeByte(uint8_t byte)
{
    uint32_t data;

    // Push a byte
    SSIDataPut(base_, byte);

    // Wait until it is complete
    while(SSIBusy(base_))
        ;

    // Read a byte
    SSIDataGet(base_, &data);
}

uint32_t Spi::writeByte(uint8_t* buffer, uint32_t length)
{
    uint32_t data;

    for (uint32_t i = 0; i < length; i++)
    {
        // Push a byte
        SSIDataPut(base_, *buffer++);

        // Wait until it is complete
        while(SSIBusy(base_))
            ;

        // Read a byte
        SSIDataGet(base_, &data);
    }

    return 0;
}

/*=============================== protected =================================*/

void Spi::interruptHandler(void)
{
    uint32_t status;

    // Read interrupt source
    status = SSIIntStatus(base_, true);

    // Clear SPI interrupt in the NVIC
    IntPendClear(interrupt_);

    // Process TX interrupt
    if (status & SSI_TXFF) {
        interruptHandlerTx();
    }

    // Process RX interrupt
    if (status & SSI_RXFF) {
        interruptHandlerRx();
    }
}

/*================================ private ==================================*/

void Spi::interruptHandlerRx(void)
{
    if (tx_callback_ != nullptr)
    {
        tx_callback_->execute();
    }
}

void Spi::interruptHandlerTx(void)
{
    if (rx_callback_ != nullptr)
    {
        rx_callback_->execute();
    }
}
