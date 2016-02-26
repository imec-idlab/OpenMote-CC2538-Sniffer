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

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "openmote-cc2538.h"

/*================================ define ===================================*/

#define GREEN_LED_TASK_PRIORITY             ( tskIDLE_PRIORITY + 1 )
#define SPI_TASK_PRIORITY                   ( tskIDLE_PRIORITY + 0 )

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static void prvGreenLedTask(void *pvParameters);
static void prvSpiTask(void *pvParameters);

/*=============================== variables =================================*/

uint8_t spi_buffer[] = {'O','p','e','n','M','o','t','e','-','C','C','2','5','3','8','\n'};
uint8_t* spi_ptr = spi_buffer;
uint8_t  spi_len = sizeof(spi_buffer);

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
    xTaskCreate(prvSpiTask, (const char *) "Spi", 128, NULL, SPI_TASK_PRIORITY, NULL);

    // Kick the FreeRTOS scheduler
    vTaskStartScheduler();
}

static void prvSpiTask(void *pvParameters)
{
    // Forever
    while (true)
    {
        // Turn on red LED
        led_red.on();

        // Print buffer via SPI
        spi.writeByte(spi_ptr, spi_len);

        // Turn off red LED
        led_red.off();

        // Delay for 250 ms
        vTaskDelay(250 / portTICK_RATE_MS);
    }
}

static void prvGreenLedTask(void *pvParameters)
{
    // Forever
    while(true)
    {
        // Turn off green LED for 950 ms
        led_green.off();
        vTaskDelay(950 / portTICK_RATE_MS);

        // Turn on green LED for 50 ms
        led_green.on();
        vTaskDelay(50 / portTICK_RATE_MS);
    }
}

/*================================ private ==================================*/

