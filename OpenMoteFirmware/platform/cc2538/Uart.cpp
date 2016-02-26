/**
 * @file       Uart.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Uart.h"
#include "InterruptHandler.h"

#include "cc2538_include.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Uart::Uart(uint32_t peripheral, uint32_t base, uint32_t clock, uint32_t interrupt, GpioUart& rx, GpioUart& tx):
    peripheral_(peripheral), base_(base), clock_(clock), interrupt_(interrupt), rx_(rx), tx_(tx)
{
}

uint32_t Uart::getBase(void)
{
    return base_;
}

void Uart::enable(uint32_t baudrate, uint32_t config, uint32_t mode)
{
    // Store the UART baudrate, configuration and mode
    baudrate_ = baudrate;
    config_   = config;
    mode_     = mode;

    // Enable peripheral except in deep sleep modes (e.g. LPM1, LPM2, LPM3)
    SysCtrlPeripheralEnable(peripheral_);
    SysCtrlPeripheralSleepEnable(peripheral_);
    SysCtrlPeripheralDeepSleepDisable(peripheral_);

    // Disable peripheral previous to configuring it
    UARTDisable(peripheral_);

    // Set IO clock as UART clock source
    UARTClockSourceSet(base_, clock_);

    // Configure the UART RX and TX pins
    IOCPinConfigPeriphInput(rx_.getPort(), rx_.getPin(), rx_.getIoc());
    IOCPinConfigPeriphOutput(tx_.getPort(), tx_.getPin(), tx_.getIoc());

    // Configure the UART GPIOs
    GPIOPinTypeUARTInput(rx_.getPort(), rx_.getPin());
    GPIOPinTypeUARTOutput(tx_.getPort(), tx_.getPin());

    // Configure the UART
    UARTConfigSetExpClk(base_, SysCtrlIOClockGet(), baudrate_, config_);

    // Disable FIFO as we only use a one-byte buffer
    UARTFIFODisable(base_);

    // Raise an interrupt at the end of transmission
    UARTTxIntModeSet(base_, mode_);

    // Enable UART hardware
    UARTEnable(base_);
}

void Uart::sleep(void)
{
    // Wait until UART is not busy
    while(UARTBusy(base_))
        ;

    // Disable UART hardware
    UARTDisable(base_);

    // Configure the pins as outputs
    GPIOPinTypeGPIOOutput(rx_.getPort(), rx_.getPin());
    GPIOPinTypeGPIOOutput(tx_.getPort(), tx_.getPin());

    // Pull the pins to ground
    GPIOPinWrite(rx_.getPort(), rx_.getPin(), 0);
    GPIOPinWrite(tx_.getPort(), tx_.getPin(), 0);
}

void Uart::wakeup(void)
{
    // Re-enable the UART interface
    enable(baudrate_, config_, mode_);
}

void Uart::setRxCallback(Callback* callback)
{
    rx_callback_ = callback;
}

void Uart::setTxCallback(Callback* callback)
{
    tx_callback_ = callback;
}

void Uart::enableInterrupts(void)
{
    // Register the interrupt handler
    InterruptHandler::getInstance().setInterruptHandler(this);

    // Enable the UART RX, TX and RX timeout interrupts
    UARTIntEnable(base_, UART_INT_RX | UART_INT_TX | UART_INT_RT);

    // Set the UART interrupt priority
    IntPrioritySet(interrupt_, (7 << 5));

    // Enable the UART interrupt
    IntEnable(interrupt_);
}

void Uart::disableInterrupts(void)
{
    // Disable the UART RX, TX and RX timeout interrupts
    UARTIntDisable(base_, UART_INT_RX | UART_INT_TX | UART_INT_RT);

    // Disable the UART interrupt
    IntDisable(interrupt_);
}

uint8_t Uart::readByte(void)
{
    int32_t byte;
    byte = UARTCharGetNonBlocking(base_);
    return (uint8_t)(byte & 0xFF);
}

uint32_t Uart::readByte(uint8_t * buffer, uint32_t length)
{
    uint32_t data;
    for (uint32_t i = 0; i < length; i++)
    {
        data = UARTCharGet(base_);
        *buffer++ = (uint8_t)data;
    }

    // Wait until it is complete
    while(UARTBusy(base_))
        ;

    return 0;
}

void Uart::writeByte(uint8_t byte)
{
    UARTCharPutNonBlocking(base_, byte);
}

uint32_t Uart::writeByte(uint8_t * buffer, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        UARTCharPut(base_, *buffer++);
    }

    // Wait until it is complete
    while(UARTBusy(base_))
        ;

    return 0;
}

/*=============================== protected =================================*/

void Uart::interruptHandler(void)
{
    uint32_t status;

    // Read interrupt source
    status = UARTIntStatus(base_, true);

    // Clear UART interrupt in the NVIC
    IntPendClear(interrupt_);

    // Process TX interrupt
    if (status & UART_INT_TX)
    {
        UARTIntClear(base_, UART_INT_TX);
        interruptHandlerTx();
    }

    // Process RX interrupt
    if (status & UART_INT_RX ||
        status & UART_INT_RT)
    {
        UARTIntClear(base_, UART_INT_RX | UART_INT_RT);
        interruptHandlerRx();
    }
}

/*================================ private ==================================*/

void Uart::interruptHandlerRx(void)
{
    if (rx_callback_ != nullptr)
    {
        rx_callback_->execute();
    }
}

void Uart::interruptHandlerTx(void)
{
    if (tx_callback_ != nullptr)
    {
        tx_callback_->execute();
    }
}
