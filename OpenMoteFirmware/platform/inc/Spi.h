/**
 * @file       Spi.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>

#include "Gpio.h"
#include "Callback.h"
#include "Mutex.h"

class Gpio;

class Spi
{

friend class InterruptHandler;

public:
    Spi(uint32_t peripheral, uint32_t base, uint32_t clock, \
        GpioSpi& miso, GpioSpi& mosi, GpioSpi& clk, GpioSpi& ncs);
    uint32_t getBase(void);
    void enable(uint32_t mode, uint32_t protocol, uint32_t datawidth, uint32_t baudrate);
    void sleep(void);
    void wakeup(void);
    void setRxCallback(Callback* callback);
    void setTxCallback(Callback* callback);
    void enableInterrupts(void);
    void disableInterrupts(void);
    void select(void);
    void deselect(void);
    uint8_t readByte(void);
    uint32_t readByte(uint8_t * buffer, uint32_t length);
    void writeByte(uint8_t byte);
    uint32_t writeByte(uint8_t * buffer, uint32_t length);
protected:
    void interruptHandler(void);
private:
    void interruptHandlerRx();
    void interruptHandlerTx();
private:
    uint32_t peripheral_;
    uint32_t base_;
    uint32_t clock_;
    uint32_t interrupt_;
    uint32_t mode_;
    uint32_t protocol_;
    uint32_t baudrate_;
    uint32_t datawidth_;

    GpioSpi& miso_;
    GpioSpi& mosi_;
    GpioSpi& clk_;
    GpioSpi& ncs_;

    Callback* rx_callback_;
    Callback* tx_callback_;
};

#endif /* SPI_H_ */
