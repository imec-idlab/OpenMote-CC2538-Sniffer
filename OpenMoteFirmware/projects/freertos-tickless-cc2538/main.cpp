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

#include "Callback.h"

/*================================ define ===================================*/

#define GREEN_LED_TASK_PRIORITY             ( tskIDLE_PRIORITY + 0 )
#define BUTTON_TASK_PRIORITY                ( tskIDLE_PRIORITY + 1 )

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

extern "C" TickType_t board_sleep(TickType_t xModifiableIdleTime);
extern "C" TickType_t board_wakeup(TickType_t xModifiableIdleTime);

static void prvGreenLedTask(void *pvParameters);
static void prvButtonTask(void *pvParameters);

static void buttonCallback(void);

/*=============================== variables =================================*/

static SemaphoreHandle_t buttonSemaphore;
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

TickType_t board_sleep(TickType_t xModifiableIdleTime)
{
    return xModifiableIdleTime;
}

TickType_t board_wakeup(TickType_t xModifiableIdleTime)
{
    return xModifiableIdleTime;
}

/*================================ private ==================================*/

static void buttonCallback(void)
{
    static BaseType_t xHigherPriorityTaskWoken;
    
    // Initialize the xHigherPriorityTaskWoken variable
    xHigherPriorityTaskWoken = pdFALSE;
    
    // Give the buttonSemaphore from the interrupt
    xSemaphoreGiveFromISR(buttonSemaphore, &xHigherPriorityTaskWoken);
    
    // Decide if we need to do a context switch
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void prvButtonTask(void *pvParameters)
{
    // Create the buttonSemaphore
    buttonSemaphore = xSemaphoreCreateMutex();

    // Set the button callback and enable interrupts
    button_user.setCallback(&userCallback);
    button_user.enableInterrupts();

    // Forever
    while(true)
    {
        // If we can take the buttonSemaphore
        if (xSemaphoreTake(buttonSemaphore, (TickType_t) portMAX_DELAY) == pdTRUE)
        {
            // Toggle red LED
            led_red.toggle();
        }
    }
}

static void prvGreenLedTask(void *pvParameters)
{
    // Forever
    while(true)
    {
        // Turn off green LED for 1950 ms
        led_green.off();
        vTaskDelay(1950 / portTICK_RATE_MS);

        // Turn on green LED for 50 ms
        led_green.on();
        vTaskDelay(50 / portTICK_RATE_MS);
    }
}

