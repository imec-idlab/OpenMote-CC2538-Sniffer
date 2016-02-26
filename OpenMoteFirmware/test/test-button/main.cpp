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

#include "Callback.h"

#include "openmote-cc2538.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/*================================ define ===================================*/

// Defines the tasks priorities
#define GREEN_LED_TASK_PRIORITY             ( tskIDLE_PRIORITY + 1 )
#define BUTTON_TASK_PRIORITY                ( tskIDLE_PRIORITY + 0 )

// Defines the Flash address
#define CC2538_FLASH_ADDRESS                ( 0x0027F800 )

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static void prvGreenLedTask(void *pvParameters);
static void prvButtonTask(void *pvParameters);

static void buttonCallback(void);

/*=============================== variables =================================*/

static xSemaphoreHandle buttonSemaphore;
static PlainCallback userCallback(buttonCallback);

/*================================= public ==================================*/

int main (void)
{
    // Set the TPS62730 in bypass mode (Vin = 3.3V, Iq < 1 uA)
    tps62730.setBypass();

    // Create two FreeRTOS tasks
    xTaskCreate(prvGreenLedTask, (const char *) "Green", 128, NULL, GREEN_LED_TASK_PRIORITY, NULL);
    xTaskCreate(prvButtonTask, (const char *) "Button", 128, NULL, BUTTON_TASK_PRIORITY, NULL);

    // Kick the FreeRTOS scheduler
    vTaskStartScheduler();
}

/*================================ private ==================================*/

static void buttonCallback(void)
{
    // Determines if the interrupt triggers a context switch
    static BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    // Give the button semaphore as the button has been pressed
    xSemaphoreGiveFromISR(buttonSemaphore, &xHigherPriorityTaskWoken);

    // Force a context switch after the interrupt if required
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void prvButtonTask(void *pvParameters)
{
    static bool flashErase = false;

    // Create and take the button semaphore
    buttonSemaphore = xSemaphoreCreateMutex();
    xSemaphoreTake(buttonSemaphore, (TickType_t)portMAX_DELAY);

    // Configure the user button
    button_user.setCallback(&userCallback);
    button_user.enableInterrupts();

    // Forever
    while (true)
    {
        // Take the buttonSemaphore, block until available
        if (xSemaphoreTake(buttonSemaphore, (TickType_t) portMAX_DELAY) == pdTRUE)
        {
            // Check if we need to erase the Flash
            if (flashErase == true)
            {
                // Erase the Flash page
                FlashMainPageErase(CC2538_FLASH_ADDRESS);

                // Reset the system
                SysCtrlReset();
            }

            // Now we need to erase the Flash
            flashErase = true;

            // Notify that we will erase the Flash
            led_red.on();
        }
    }
}

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
