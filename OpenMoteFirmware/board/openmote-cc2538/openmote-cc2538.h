/**
 * @file       openmote-cc2538.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef BOARD_OPENMOTE_CC2538_H_
#define BOARD_OPENMOTE_CC2538_H_

/*================================ include ==================================*/

#include "Board.h"
#include "Gpio.h"
#include "I2c.h"
#include "Radio.h"
#include "RadioTimer.h"
#include "SleepTimer.h"
#include "Spi.h"
#include "Timer.h"
#include "Uart.h"
#include "Watchdog.h"

#include "Adxl346.h"
#include "Max44009.h"
#include "Sht21.h"

#include "Tps62730.h"
#include "Enc28j60.h"

#include "cc2538_include.h"
#include "cc2538_types.h"

/*================================ define ===================================*/

#define BOARD_HAS_32MHz_XTAL    ( TRUE )
#define BOARD_USE_32MHz_XTAL    ( TRUE )
#define BOARD_HAS_32kHz_XTAL    ( TRUE )
#define BOARD_USE_32kHz_XTAL    ( TRUE )
#define SYSTEM_CLOCK_DIVIDER    ( SYS_CTRL_SYSDIV_16MHZ )
#define PERIPH_CLOCK_DIVIDER    ( SYS_CTRL_SYSDIV_16MHZ )
#define WATCHDOG_INTERVAL       ( WATCHDOG_INTERVAL_32768 )

#define LED_RED_PORT            ( GPIO_C_BASE )
#define LED_RED_PIN             ( GPIO_PIN_4 )

#define LED_ORANGE_PORT         ( GPIO_C_BASE )
#define LED_ORANGE_PIN          ( GPIO_PIN_5 )

#define LED_YELLOW_PORT         ( GPIO_C_BASE )
#define LED_YELLOW_PIN          ( GPIO_PIN_6 )

#define LED_GREEN_PORT          ( GPIO_C_BASE )
#define LED_GREEN_PIN           ( GPIO_PIN_7 )

#define GPIO_DEBUG_AD0_PORT     ( GPIO_D_BASE )
#define GPIO_DEBUG_AD0_PIN      ( GPIO_PIN_3 )

#define GPIO_DEBUG_AD1_PORT     ( GPIO_D_BASE )
#define GPIO_DEBUG_AD1_PIN      ( GPIO_PIN_2 )

#define GPIO_DEBUG_AD2_PORT     ( GPIO_D_BASE )
#define GPIO_DEBUG_AD2_PIN      ( GPIO_PIN_1 )

#define BUTTON_USER_PORT        ( GPIO_C_BASE )
#define BUTTON_USER_PIN         ( GPIO_PIN_3 )
#define BUTTON_USER_EDGE        ( GPIO_FALLING_EDGE )

#define TIMER0_PERIPHERAL       ( SYS_CTRL_PERIPH_GPT0 )
#define TIMER0_BASE             ( GPTIMER0_BASE )
#define TIMER0_SOURCE           ( GPTIMER_BOTH )
#define TIMER0_CONFIG           ( GPTIMER_CFG_PERIODIC )
#define TIMER0_INTERRUPT        ( INT_TIMER0A )
#define TIMER0_INTERRUPT_MODE   ( GPTIMER_TIMA_TIMEOUT )

#define TIMER1_PERIPHERAL       ( SYS_CTRL_PERIPH_GPT1 )
#define TIMER1_BASE             ( GPTIMER1_BASE )
#define TIMER1_SOURCE           ( GPTIMER_BOTH )
#define TIMER1_CONFIG           ( GPTIMER_CFG_PERIODIC )
#define TIMER1_INTERRUPT        ( INT_TIMER1A )
#define TIMER1_INTERRUPT_MODE   ( GPTIMER_TIMA_TIMEOUT )

#define TIMER2_PERIPHERAL       ( SYS_CTRL_PERIPH_GPT2 )
#define TIMER2_BASE             ( GPTIMER2_BASE )
#define TIMER2_SOURCE           ( GPTIMER_BOTH )
#define TIMER2_CONFIG           ( GPTIMER_CFG_PERIODIC )
#define TIMER2_INTERRUPT        ( INT_TIMER2A )
#define TIMER2_INTERRUPT_MODE   ( GPTIMER_TIMA_TIMEOUT )

#define TIMER3_PERIPHERAL       ( SYS_CTRL_PERIPH_GPT3 )
#define TIMER3_BASE             ( GPTIMER3_BASE )
#define TIMER3_SOURCE           ( GPTIMER_BOTH )
#define TIMER3_CONFIG           ( GPTIMER_CFG_PERIODIC )
#define TIMER3_INTERRUPT        ( INT_TIMER3A )
#define TIMER3_INTERRUPT_MODE   ( GPTIMER_TIMA_TIMEOUT )

#define SLEEP_TIMER_INTERRUPT   ( INT_SMTIM )

#define RADIO_TIMER_INTERRUPT   ( INT_MACTIMR )

#define TPS62730_PORT           ( GPIO_B_BASE )
#define TPS62730_STATUS_PIN     ( GPIO_PIN_0 )
#define TPS62730_STATUS_EDGE    ( GPIO_BOTH_EDGES )
#define TPS62730_BYPASS_PIN     ( GPIO_PIN_1 )

#define ANTENNA_EXTERNAL_PORT   ( GPIO_D_BASE )
#define ANTENNA_EXTERNAL_PIN    ( GPIO_PIN_4 )
#define ANTENNA_INTERNAL_PORT   ( GPIO_D_BASE )
#define ANTENNA_INTERNAL_PIN    ( GPIO_PIN_5 )

#define UART_PERIPHERAL         ( SYS_CTRL_PERIPH_UART0 )
#define UART_BASE               ( UART0_BASE )
#define UART_CLOCK              ( UART_CLOCK_PIOSC )
#define UART_INTERRUPT          ( INT_UART0 )
#define UART_RX_PORT            ( GPIO_A_BASE )
#define UART_RX_PIN             ( GPIO_PIN_0 )
#define UART_RX_IOC             ( IOC_UARTRXD_UART0 )
#define UART_TX_PORT            ( GPIO_A_BASE )
#define UART_TX_PIN             ( GPIO_PIN_1 )
#define UART_TX_IOC             ( IOC_MUX_OUT_SEL_UART0_TXD )
#define UART_BAUDRATE           ( 115200 )
#define UART_CONFIG             ( UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE )
#define UART_INT_MODE           ( UART_TXINT_MODE_EOT )

#define SPI_PERIPHERAL          ( SYS_CTRL_PERIPH_SSI0 )
#define SPI_BASE                ( SSI0_BASE )
#define SPI_CLOCK               ( SSI_CLOCK_PIOSC )
#define SPI_INTERRUPT           ( INT_SSI0 )
#define SPI_MISO_BASE           ( GPIO_A_BASE )
#define SPI_MISO_PIN            ( GPIO_PIN_4 )
#define SPI_MISO_IOC            ( IOC_SSIRXD_SSI0 )
#define SPI_MOSI_BASE           ( GPIO_A_BASE )
#define SPI_MOSI_PIN            ( GPIO_PIN_5 )
#define SPI_MOSI_IOC            ( IOC_MUX_OUT_SEL_SSI0_TXD )
#define SPI_nCS_BASE            ( GPIO_A_BASE )
#define SPI_nCS_PIN             ( GPIO_PIN_3 )
#define SPI_nCS_IOC             ( IOC_MUX_OUT_SEL_SSI0_FSSOUT )
#define SPI_CLK_BASE            ( GPIO_A_BASE )
#define SPI_CLK_PIN             ( GPIO_PIN_2 )
#define SPI_CLK_IOC             ( IOC_MUX_OUT_SEL_SSI0_CLKOUT )
#define SPI_MODE                ( SSI_MODE_MASTER )
#define SPI_PROTOCOL            ( SSI_FRF_MOTO_MODE_0 )
#define SPI_DATAWIDTH           ( 8 )
#define SPI_BAUDRATE            ( 8000000 )

#define I2C_PERIPHERAL          ( SYS_CTRL_PERIPH_I2C )
#define I2C_BASE                ( GPIO_B_BASE )
#define I2C_SCL                 ( GPIO_PIN_3 )
#define I2C_SDA                 ( GPIO_PIN_4 )
#define I2C_BAUDRATE            ( 100000 )

#define ADXL346_INT_PORT        ( GPIO_B_BASE)
#define ADXL346_INT_PIN         ( GPIO_PIN_5 )
#define ADXL346_INT_EDGE        ( GPIO_FALLING_EDGE )

#define MAX44009_INT_PORT       ( GPIO_B_BASE )
#define MAX44009_INT_PIN        ( GPIO_PIN_2 )
#define MAX44009_INT_EDGE       ( GPIO_FALLING_EDGE )

#define ENC28J60_INT_PORT       ( GPIO_D_BASE )
#define ENC28J60_INT_PIN        ( GPIO_PIN_0 )
#define ENC28J60_INT_EDGE       ( GPIO_FALLING_EDGE )

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

// Board management
extern Board board;
extern Watchdog watchdog;

// Step-down DC/DC converter
extern Tps62730 tps62730;

// Debug pins
extern GpioOut debug_ad0;
extern GpioOut debug_ad1;
extern GpioOut debug_ad2;

// Leds
extern GpioOut led_green;
extern GpioOut led_orange;
extern GpioOut led_red;
extern GpioOut led_yellow;

// Button
extern GpioInPow button_user;

// Antenna
extern GpioOut antenna_external;
extern GpioOut antenna_internal;

// Timer
extern Timer timer0;
extern Timer timer1;
extern Timer timer2;
extern Timer timer3;

// SleepTimer
extern SleepTimer sleepTimer;

// RadioTimer
extern RadioTimer radioTimer;

// I2C peripheral
extern I2c i2c;

// SPI peripheral
extern Spi spi;

// UART peripheral
extern Uart uart;

// IEEE 802.15.4 radio
extern Radio radio;

// Acceleration sensor
extern Adxl346 adxl346;

// Light sensor
extern Max44009 max44009;

// Temperature + Relative humidity sensor
extern Sht21 sht21;

// Ethernet PHY + MAC chip
extern Enc28j60 enc28j60;

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

/*================================ private ==================================*/

#endif /* BOARD_OPENMOTE_CC2538_H_ */
