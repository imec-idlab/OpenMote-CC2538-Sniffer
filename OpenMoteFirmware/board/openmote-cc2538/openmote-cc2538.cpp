/**
 * @file       openmote-cc2538.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "openmote-cc2538.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

// Board management
Board board;
Watchdog watchdog(WATCHDOG_INTERVAL);

// Step-down DC/DC converter
GpioOut bypass(TPS62730_PORT, TPS62730_BYPASS_PIN);
GpioIn  status(TPS62730_PORT, TPS62730_STATUS_PIN, TPS62730_STATUS_EDGE);
Tps62730 tps62730(bypass, status);

// Debug pins
GpioOut debug_ad0(GPIO_DEBUG_AD0_PORT, GPIO_DEBUG_AD0_PIN);
GpioOut debug_ad1(GPIO_DEBUG_AD1_PORT, GPIO_DEBUG_AD1_PIN);
GpioOut debug_ad2(GPIO_DEBUG_AD2_PORT, GPIO_DEBUG_AD2_PIN);

// Leds
GpioOut led_green(LED_GREEN_PORT, LED_GREEN_PIN);
GpioOut led_orange(LED_ORANGE_PORT, LED_ORANGE_PIN);
GpioOut led_red(LED_RED_PORT, LED_RED_PIN);
GpioOut led_yellow(LED_YELLOW_PORT, LED_YELLOW_PIN);

// Antenna
GpioOut antenna_external(ANTENNA_EXTERNAL_PORT, ANTENNA_EXTERNAL_PIN);
GpioOut antenna_internal(ANTENNA_INTERNAL_PORT, ANTENNA_INTERNAL_PIN);

// Button
GpioInPow button_user(BUTTON_USER_PORT, BUTTON_USER_PIN, BUTTON_USER_EDGE);

// Timer
Timer timer0(TIMER0_PERIPHERAL, TIMER0_BASE, TIMER0_SOURCE, TIMER0_CONFIG, TIMER0_INTERRUPT, TIMER0_INTERRUPT_MODE);
Timer timer1(TIMER1_PERIPHERAL, TIMER1_BASE, TIMER1_SOURCE, TIMER1_CONFIG, TIMER1_INTERRUPT, TIMER1_INTERRUPT_MODE);
Timer timer2(TIMER2_PERIPHERAL, TIMER2_BASE, TIMER2_SOURCE, TIMER2_CONFIG, TIMER2_INTERRUPT, TIMER2_INTERRUPT_MODE);
Timer timer3(TIMER3_PERIPHERAL, TIMER3_BASE, TIMER3_SOURCE, TIMER3_CONFIG, TIMER3_INTERRUPT, TIMER3_INTERRUPT_MODE);

// SleepTimer
SleepTimer sleepTimer(SLEEP_TIMER_INTERRUPT);

// RadioTimer
RadioTimer radioTimer(RADIO_TIMER_INTERRUPT);

// I2C peripheral
GpioI2c i2c_scl(I2C_BASE, I2C_SCL);
GpioI2c i2c_sda(I2C_BASE, I2C_SDA);
I2c i2c(I2C_PERIPHERAL, i2c_scl, i2c_sda);

// SPI peripheral
GpioSpi spi_miso(SPI_MISO_BASE, SPI_MISO_PIN, SPI_MISO_IOC);
GpioSpi spi_mosi(SPI_MOSI_BASE, SPI_MOSI_PIN, SPI_MOSI_IOC);
GpioSpi spi_clk(SPI_CLK_BASE, SPI_CLK_PIN, SPI_CLK_IOC);
GpioSpi spi_ncs(SPI_nCS_BASE, SPI_nCS_PIN, SPI_nCS_IOC);
Spi spi(SPI_PERIPHERAL, SPI_BASE, SPI_CLOCK, spi_miso, spi_mosi, spi_clk, spi_ncs);

// UART peripheral
GpioUart uart_rx(UART_RX_PORT, UART_RX_PIN, UART_RX_IOC);
GpioUart uart_tx(UART_TX_PORT, UART_TX_PIN, UART_TX_IOC);
Uart uart(UART_PERIPHERAL, UART_BASE, UART_CLOCK, UART_INTERRUPT, uart_rx, uart_tx);

// IEEE 802.15.4 radio
Radio radio;

// Acceleration sensor
GpioInPow adxl346_int(ADXL346_INT_PORT, ADXL346_INT_PIN, ADXL346_INT_EDGE);
Adxl346 adxl346(i2c, adxl346_int);

// Light sensor
GpioIn max44009_int(MAX44009_INT_PORT, MAX44009_INT_PIN, MAX44009_INT_EDGE);
Max44009 max44009(i2c, max44009_int);

// Temperature + Relative humidity sensor
Sht21 sht21(i2c);

// Ethernet PHY + MAC chip
GpioIn enc28j60_int(ENC28J60_INT_PORT, ENC28J60_INT_PIN, ENC28J60_INT_EDGE);
Enc28j60 enc28j60(spi, enc28j60_int);

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

/*================================ private ==================================*/
