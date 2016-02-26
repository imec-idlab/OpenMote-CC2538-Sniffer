/**
 * @file       Uart.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>

#include "Gpio.h"
#include "Callback.h"
#include "Mutex.h"

class Gpio;

class Uart
{

friend class InterruptHandler;

public:
    Uart(uint32_t peripheral, uint32_t base, uint32_t clock, uint32_t interrupt, GpioUart& rx, GpioUart& tx);
    uint32_t getBase(void);
    void rxUnlockFromInterrupt(void) {rxMutex_.giveFromInterrupt();}
    void txUnlockFromInterrupt(void) {txMutex_.giveFromInterrupt();}
    void enable(uint32_t baudrate, uint32_t config, uint32_t mode);
    void sleep(void);
    void wakeup(void);
    void setRxCallback(Callback* callback);
    void setTxCallback(Callback* callback);
    void enableInterrupts(void);
    void disableInterrupts(void);
    void rxLock(void) {rxMutex_.take();}
    void txLock(void) {txMutex_.take();}
    void rxUnlock(void) {rxMutex_.give();}
    void txUnlock(void) {txMutex_.give();}
    uint8_t readByte(void);
    uint32_t readByte(uint8_t* buffer, uint32_t length);
    void writeByte(uint8_t byte);
    uint32_t writeByte(uint8_t* buffer, uint32_t length);
protected:
    void interruptHandler(void);
private:
    void interruptHandlerRx(void);
    void interruptHandlerTx(void);
private:
    uint32_t peripheral_;
    uint32_t base_;
    uint32_t clock_;
    uint32_t interrupt_;
    uint32_t mode_;
    uint32_t config_;
    uint32_t baudrate_;

    Mutex rxMutex_;
    Mutex txMutex_;

    GpioUart& rx_;
    GpioUart& tx_;

    Callback* rx_callback_;
    Callback* tx_callback_;
};

#endif /* UART_H_ */
