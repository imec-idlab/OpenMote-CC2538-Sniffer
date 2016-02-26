/**
 * @file       main.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include <string.h>

#include "Callback.h"
#include "Serial.h"

#include "SnifferEthernet.h"
#include "SnifferSerial.h"

#include "openmote-cc2538.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/*================================ define ===================================*/

#define GREEN_LED_TASK_PRIORITY             ( tskIDLE_PRIORITY + 0 )
#define SERIAL_TASK_PRIORITY                ( tskIDLE_PRIORITY + 1 )
#define SNIFFER_TASK_PRIORITY               ( tskIDLE_PRIORITY + 2 )

#define SNIFFER_DEFAULT_CHANNEL             ( 20 )
#define SERIAL_CHANGE_CHANNEL_CMD           ( 0xCC )

#define SNIFFER_ETHERNET                    ( 0 )
#define SNIFFER_SERIAL                      ( 1 )

#define SNIFFER_TYPE                        ( SNIFFER_SERIAL )

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static void prvGreenLedTask(void *pvParameters);
static void prvSnifferTask(void *pvParameters);
static void prvSerialTask(void *pvParamters);

/*=============================== variables =================================*/

static Serial serial(uart);
static Ethernet ethernet(enc28j60);

#if (SNIFFER_TYPE == SNIFFER_SERIAL)
static SnifferSerial   sniffer(board, radio, serial);
#elif  (SNIFFER_TYPE == SNIFFER_ETHERNET)
static SnifferEthernet sniffer(board, radio, ethernet);
#else
#error
#endif

static uint8_t serial_buffer[32];
static uint8_t* serial_buffer_ptr;
static int32_t serial_buffer_len;

static uint8_t sniffer_command;
static uint8_t sniffer_channel;

/*================================= public ==================================*/

int main (void)
{
    // Set the TPS62730 in bypass mode (Vin = 3.3V, Iq < 1 uA)
    tps62730.setBypass();

    // Enable erasing the Flash with the user button
    board.enableFlashErase();

    // Enable the SPI peripheral
    spi.enable(SPI_MODE, SPI_PROTOCOL, SPI_DATAWIDTH, SPI_BAUDRATE);

    // Enable the UART peripheral
    uart.enable(UART_BAUDRATE, UART_CONFIG, UART_INT_MODE);

    // Init the serial
    serial.init();

    // Create the blink task
    xTaskCreate(prvGreenLedTask, (const char *) "LedTask", 128, NULL, GREEN_LED_TASK_PRIORITY, NULL);

    // Create the sniffer task to process packets
    xTaskCreate(prvSnifferTask, (const char *) "SnifferTask", 128, NULL, SNIFFER_TASK_PRIORITY, NULL);

    // Create the serial task to receive commands
    xTaskCreate(prvSerialTask, (const char *) "SerialTask", 128, NULL, SERIAL_TASK_PRIORITY, NULL);

    // Kick the FreeRTOS scheduler
    vTaskStartScheduler();
}

/*================================ private ==================================*/

static void prvGreenLedTask(void *pvParameters)
{
    // Forever
    while (true)
    {
        // Turn off the green LED and keep it for 950 ms
        led_green.off();
        vTaskDelay(950 / portTICK_RATE_MS);

        // Turn on the green LED and keep it for 50 ms
        led_green.on();
        vTaskDelay(50 / portTICK_RATE_MS);
    }
}

static void prvSerialTask(void *pvParamters)
{
    while (true)
    {
        // Reset the buffer pointer and length
        serial_buffer_ptr = serial_buffer;
        serial_buffer_len = sizeof(serial_buffer);

        // Wait until we receive a command
        serial_buffer_len = serial.read(serial_buffer_ptr, serial_buffer_len);

        // Check the length of the buffer and update variables
        if (serial_buffer_len == 2)
        {
            sniffer_command = serial_buffer[0];
            sniffer_channel = serial_buffer[1];
        }

        // Check if the received command is valid
        if (sniffer_command == SERIAL_CHANGE_CHANNEL_CMD) {
            // Stop the sniffer prior to updating the channel
            sniffer.stop();

            // Change the sniffer channel
            sniffer.setChannel(sniffer_channel);

            // Re-start the sniffer
            sniffer.start();
        }

        // Reset the sniffer command and channel
        sniffer_command = 0x00;
        sniffer_channel = 0x00;
    }
}

static void prvSnifferTask(void *pvParameters)
{
    // Initialize the sniffer
    sniffer.init();

    // Set the default sniffer channel
    sniffer.setChannel(SNIFFER_DEFAULT_CHANNEL);

    while (true)
    {
        // Start the sniffer
        sniffer.start();

        // Process a frame
        sniffer.processRadioFrame();
    }
}
