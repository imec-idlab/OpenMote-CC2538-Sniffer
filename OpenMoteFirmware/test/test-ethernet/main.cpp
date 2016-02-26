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

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "openmote-cc2538.h"

/*================================ define ===================================*/

#define GREEN_LED_TASK_PRIORITY             ( tskIDLE_PRIORITY + 0 )
#define ETHERNET_TASK_PRIORITY              ( tskIDLE_PRIORITY + 1 )

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static void prvGreenLedTask(void *pvParameters);
static void prvEthernetTask(void *pvParameters);

static void ethernet_process_rx(void);

/*=============================== variables =================================*/

static uint8_t eui48_address[6];

static uint8_t  tx_frame[] =
                    { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                      0x08, 0x00,
                      0x53, 0x68, 0x69, 0x6E, 0x69, 0x6E, 0x67, 0x20, 0x42, 0x69,
                      0x74, 0x73, 0x2C, 0x20, 0x65, 0x6D, 0x62, 0x65, 0x64, 0x64,
                      0x65, 0x64, 0x20, 0x69, 0x6E, 0x74, 0x65, 0x6C, 0x6C, 0x69,
                      0x67, 0x65, 0x6E, 0x63, 0x65, 0x20, 0x21 };
static uint8_t* tx_frame_ptr = tx_frame;
static uint32_t tx_frame_len = sizeof(tx_frame);

static uint8_t  rx_frame[128];
static uint8_t* rx_frame_ptr = rx_frame;
static uint32_t rx_frame_len = sizeof(rx_frame);

static xSemaphoreHandle ethernet_rx_semaphore;
static PlainCallback    ethernet_rx_callback(&ethernet_process_rx);

static Ethernet ethernet(enc28j60);

/*================================= public ==================================*/

int main (void)
{
    // Set the TPS62730 in bypass mode (Vin = 3.3V, Iq < 1 uA)
    tps62730.setBypass();
    
    // Enable erasing the Flash with the user button
    board.enableFlashErase();

    // Enable the SPI peripheral
    spi.enable(SPI_MODE, SPI_PROTOCOL, SPI_DATAWIDTH, SPI_BAUDRATE);

    // Create two FreeRTOS tasks
    xTaskCreate(prvGreenLedTask, (const char *) "Green", 128, NULL, GREEN_LED_TASK_PRIORITY, NULL);
    xTaskCreate(prvEthernetTask, (const char *) "Ethernet", 128, NULL, ETHERNET_TASK_PRIORITY, NULL);

    // Kick the FreeRTOS scheduler
    vTaskStartScheduler();
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

static void prvGreenLedTask(void *pvParameters)
{
    // Forever
    while (true)
    {
        // Turn off green LED for 950 ms
        led_green.off();
        vTaskDelay(950 / portTICK_RATE_MS);

        // Turn on green LED for 50 ms
        led_green.on();
        vTaskDelay(50 / portTICK_RATE_MS);
    }
}

static void prvEthernetTask(void *pvParameters)
{
    // Get EUI-48 address
    board.getEUI48(eui48_address);

    // Set the EUI-48 address to the transmit frame
    memcpy(&tx_frame[6], eui48_address, 6);

    // Use the EUI-48 address as the MAC address
    ethernet.init(eui48_address);

    // Register Ethernet callback
    ethernet.setCallback(&ethernet_rx_callback);

    // Create and initialize semaphore
    ethernet_rx_semaphore = xSemaphoreCreateMutex();
    xSemaphoreTake(ethernet_rx_semaphore, (TickType_t) portMAX_DELAY);

    vTaskDelay(2500 / portTICK_RATE_MS);

    // Forever
    while (true)
    {
        // Send Ethernet payload
        ethernet.transmitFrame(tx_frame_ptr, tx_frame_len);

        // Try to take the semaphore
        if (xSemaphoreTake(ethernet_rx_semaphore, (TickType_t) portMAX_DELAY) == pdTRUE)
        {
            // Receive the frame
            ethernet.receiveFrame(rx_frame_ptr, &rx_frame_len);

            // Copy destination addresses to the transmit packet
            memcpy(&tx_frame_ptr[0], &rx_frame_ptr[6], 6);

            // Restore frame pointers
            rx_frame_ptr = rx_frame;
            rx_frame_len = sizeof(rx_frame);

            vTaskDelay(250 / portTICK_RATE_MS);
        }
    }
}

static void ethernet_process_rx(void)
{
    // Determines if the interrupt triggers a context switch
    static BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    // Give the transmit semaphore as the packet has been transmitted
    xSemaphoreGiveFromISR(ethernet_rx_semaphore, &xHigherPriorityTaskWoken);

    // Force a context switch after the interrupt if required
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
