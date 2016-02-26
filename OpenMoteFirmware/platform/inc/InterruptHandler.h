/**
 * @file       InterruptHandler.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef INTERRUPT_HANDLER_H_
#define INTERRUPT_HANDLER_H_

class GpioIn;
class GpioInPow;
class Timer;
class Uart;
class I2c;
class Spi;
class Radio;
class SysTick;
class SleepTimer;
class RadioTimer;

class InterruptHandler {

public:
    static InterruptHandler& getInstance(void);
    static void setInterruptHandler(GpioIn* gpio);
    static void clearInterruptHandler(GpioIn* gpio);
    static void setInterruptHandler(Timer* timer);
    static void clearInterruptHandler(Timer* timer);
    static void setInterruptHandler(Uart* uart);
    static void clearInterruptHandler(Uart* uart);
    static void setInterruptHandler(I2c* i2c);
    static void clearInterruptHandler(I2c* i2c);
    static void setInterruptHandler(Spi* spi);
    static void clearInterruptHandler(Spi* spi);
    static void setInterruptHandler(SysTick* systick);
    static void clearInterruptHandler(SysTick* systick);
    static void setInterruptHandler(Radio* radio);
    static void clearInterruptHandler(Radio* radio);
    static void setInterruptHandler(SleepTimer* sleepTimer);
    static void clearInterruptHandler(SleepTimer* sleepTimer);
    static void setInterruptHandler(RadioTimer* radioTimer);
    static void clearInterruptHandler(RadioTimer* radioTimer);
private:
    InterruptHandler();
    static inline void GPIOA_InterruptHandler(void);
    static inline void GPIOB_InterruptHandler(void);
    static inline void GPIOC_InterruptHandler(void);
    static inline void GPIOD_InterruptHandler(void);
    static inline void TIMER0_InterruptHandler(void);
    static inline void TIMER1_InterruptHandler(void);
    static inline void TIMER2_InterruptHandler(void);
    static inline void TIMER3_InterruptHandler(void);
    static inline void UART0_InterruptHandler(void);
    static inline void UART1_InterruptHandler(void);
    static inline void I2C_InterruptHandler(void);
    static inline void SPI0_InterruptHandler(void);
    static inline void SPI1_InterruptHandler(void);
    static inline void SysTick_InterruptHandler(void);
    static inline void RFCore_InterruptHandler(void);
    static inline void RFError_InterruptHandler(void);
    static inline void SleepTimer_InterruptHandler(void);
    static inline void RadioTimer_InterruptHandler(void);
private:
    static InterruptHandler instance_;
    static GpioIn* GPIOA_interruptVector_[8];
    static GpioIn* GPIOB_interruptVector_[8];
    static GpioIn* GPIOC_interruptVector_[8];
    static GpioIn* GPIOD_interruptVector_[8];
    static Timer* TIMER0_interruptVector_[2];
    static Timer* TIMER1_interruptVector_[2];
    static Timer* TIMER2_interruptVector_[2];
    static Timer* TIMER3_interruptVector_[2];
    static Uart* UART0_interruptVector_;
    static Uart* UART1_interruptVector_;
    static I2c* I2C_interruptVector_;
    static Spi* SPI0_interruptVector_;
    static Spi* SPI1_interruptVector_;
    static SysTick* SysTick_interruptVector_;
    static Radio* Radio_interruptVector_;
    static SleepTimer* SleepTimer_interruptVector_;
    static RadioTimer* RadioTimer_interruptVector_;
};

#endif /* INTERRUPT_HANDLER_H */
