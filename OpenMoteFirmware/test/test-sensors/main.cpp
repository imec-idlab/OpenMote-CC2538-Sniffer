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

#define GREEN_LED_TASK_PRIORITY         ( tskIDLE_PRIORITY + 0 )
#define LIGHT_TASK_PRIORITY             ( tskIDLE_PRIORITY + 1 )
#define TEMPERATURE_TASK_PRIORITY       ( tskIDLE_PRIORITY + 2 )
#define ACCELERATION_TASK_PRIORITY      ( tskIDLE_PRIORITY + 3 )

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

extern "C" TickType_t board_sleep(TickType_t xModifiableIdleTime);
extern "C" TickType_t board_wakeup(TickType_t xModifiableIdleTime);

static void prvGreenLedTask(void *pvParameters);
static void prvLightTask(void *pvParameters);
static void prvTemperatureTask(void *pvParameters);
static void prvAccelerationTask(void *pvParameters);

/*=============================== variables =================================*/

/*================================= public ==================================*/

int main (void)
{
    // Set the TPS62730 in bypass mode (Vin = 3.3V, Iq < 1 uA)
    tps62730.setBypass();

    // Enable erasing the Flash with the user button
    board.enableFlashErase();

    // Enable the I2C interface
    i2c.enable();

    // Create the FreeRTOS tasks
    xTaskCreate(prvGreenLedTask, (const char *) "GreenLed", 128, NULL, GREEN_LED_TASK_PRIORITY, NULL);
    xTaskCreate(prvAccelerationTask, (const char *) "Acceleration", 128, NULL, ACCELERATION_TASK_PRIORITY, NULL);
    xTaskCreate(prvTemperatureTask, (const char *) "Temperature", 128, NULL, TEMPERATURE_TASK_PRIORITY, NULL);
    xTaskCreate(prvLightTask, (const char *) "Light", 128, NULL, LIGHT_TASK_PRIORITY, NULL);

    // Kick the FreeRTOS scheduler
    vTaskStartScheduler();
}

TickType_t board_sleep(TickType_t xModifiableIdleTime)
{
    i2c.sleep();
    return xModifiableIdleTime;
}

TickType_t board_wakeup(TickType_t xModifiableIdleTime)
{
    i2c.wakeup();
    return xModifiableIdleTime;
}

/*================================ private ==================================*/

static void prvTemperatureTask(void *pvParameters)
{
    uint16_t temperature;
    uint16_t humidity;

    if (sht21.isPresent() == true)
    {
        sht21.enable();

        while(true)
        {
            led_orange.on();

            sht21.readTemperature();
            temperature = sht21.getTemperatureRaw();

            sht21.readHumidity();
            humidity = sht21.getHumidityRaw();

            led_orange.off();

            vTaskDelay(2000 / portTICK_RATE_MS);
        }
    }
    else
    {
        led_orange.on();
        vTaskDelete(NULL);
    }
}

static void prvLightTask(void *pvParameters)
{
    uint16_t light;

    if (max44009.isPresent() == true)
    {
        max44009.enable();

        while(true)
        {
            led_yellow.on();

            max44009.readLux();
            light = max44009.getLuxRaw();

            led_yellow.off();

            vTaskDelay(2000 / portTICK_RATE_MS);
        }
    }
    else
    {
        led_yellow.on();
        vTaskDelete(NULL);
    }
}

static void prvAccelerationTask(void *pvParameters) {
    uint16_t x, y, z;

    if (adxl346.isPresent() == true)
    {
        adxl346.enable();

        while(true)
        {
            led_red.on();

            adxl346.wakeup();
            adxl346.readSample(&x, &y, &z);
            adxl346.suspend();

            led_red.off();

            vTaskDelay(2000 / portTICK_RATE_MS);
        }
    }
    else
    {
        led_red.on();
        vTaskDelete(NULL);
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

